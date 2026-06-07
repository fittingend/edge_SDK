import argparse
import glob
import json
import os

import numpy as np
import pandas as pd
from sklearn.metrics import average_precision_score, roc_auc_score
from sklearn.model_selection import GroupShuffleSplit
from sklearn.preprocessing import StandardScaler

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def resolve_path(path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(BASE_DIR, path)


ap = argparse.ArgumentParser(description="Train risk model and export ONNX.")
ap.add_argument(
    "--log-dir",
    default="log",
    help="directory with auto_labels_*.csv and label*.csv",
)
ap.add_argument("--out-dir", default="artifacts", help="output directory for model/meta")
ap.add_argument("--train-files", nargs="*", default=None, help="basenames to use for train split")
ap.add_argument("--val-files", nargs="*", default=None, help="basenames to use for validation split")
ap.add_argument("--test-files", nargs="*", default=None, help="basenames to use for test split")
ap.add_argument("--model-type", choices=["mlp", "temporal_mlp"], default="mlp")
ap.add_argument("--seq-len", type=int, default=5, help="sequence length for temporal_mlp")
ap.add_argument("--epochs", type=int, default=20)
ap.add_argument("--use-nan-mask", action="store_true", help="append per-feature NaN mask features")
args = ap.parse_args()

if args.seq_len < 1:
    raise ValueError("--seq-len must be >= 1")

log_folder = resolve_path(args.log_dir)
out_dir = resolve_path(args.out_dir)
os.makedirs(out_dir, exist_ok=True)


def normalize_file_list(file_list):
    if not file_list:
        return set()
    return {os.path.basename(path) for path in file_list}


def get_time_sort_cols(df_part: pd.DataFrame) -> list[str]:
    cols = ["__source_file", "obstacle_id"]
    if "obs_timestamp_ms" in df_part.columns:
        cols.append("obs_timestamp_ms")
    cols.append("frame_id")
    return cols


def build_sequences(df_part, feature_cols, label_cols, seq_len):
    if df_part.empty:
        return (
            np.zeros((0, seq_len, len(feature_cols)), dtype=np.float32),
            np.zeros((0, len(label_cols)), dtype=np.float32),
        )

    df_part = df_part.sort_values(get_time_sort_cols(df_part)).reset_index(drop=True)

    x_seq = []
    y_seq = []
    for (_, _), group in df_part.groupby(["__source_file", "obstacle_id"], sort=False):
        feats = group[feature_cols].to_numpy(dtype=np.float32)
        labels = group[label_cols].to_numpy(dtype=np.float32)

        for i in range(len(group)):
            start = max(0, i - seq_len + 1)
            window = feats[start : i + 1]

            if len(window) < seq_len:
                pad = np.repeat(window[:1], seq_len - len(window), axis=0)
                window = np.vstack([pad, window])

            x_seq.append(window)
            y_seq.append(labels[i])

    return np.asarray(x_seq, dtype=np.float32), np.asarray(y_seq, dtype=np.float32)


def evaluate_split(split_name, loader):
    model.eval()
    split_loss = 0.0
    split_logits = []
    split_targets = []
    with torch.no_grad():
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            logits = model(xb)
            loss = criterion(logits, yb)
            split_loss += loss.item() * xb.size(0)
            split_logits.append(logits.detach().cpu())
            split_targets.append(yb.detach().cpu())

    split_loss /= len(loader.dataset)
    y_true = torch.cat(split_targets).numpy()
    y_true_bin = (y_true > 0).astype(np.uint8)
    y_prob = torch.sigmoid(torch.cat(split_logits)).numpy()

    print(f"{split_name}_loss={split_loss:.4f}")
    for i, label in enumerate(label_cols):
        try:
            roc = roc_auc_score(y_true_bin[:, i], y_prob[:, i])
        except ValueError:
            roc = float("nan")
        try:
            score_ap = average_precision_score(y_true_bin[:, i], y_prob[:, i])
        except ValueError:
            score_ap = float("nan")
        print(f"  {label}: ROC-AUC={roc:.4f} PR-AUC={score_ap:.4f}")

    return split_loss


csv_patterns = ["auto_labels_*.csv", "label*.csv"]
csv_files = sorted(
    {
        path
        for pattern in csv_patterns
        for path in glob.glob(os.path.join(log_folder, pattern))
    }
)

if not csv_files:
    patterns = ", ".join(csv_patterns)
    raise FileNotFoundError(f"No CSV files found in {log_folder} for patterns: {patterns}")

print("Available CSV files:")
for i, f in enumerate(csv_files):
    print(f"  {i}: {os.path.basename(f)}")

dfs = []
for f in csv_files:
    df_part = pd.read_csv(f)
    df_part["__source_file"] = os.path.basename(f)
    dfs.append(df_part)
df = pd.concat(dfs, ignore_index=True)
print(f"\nLoaded {len(csv_files)} files, total rows: {len(df)}")

# 1) Dedup inside each source file
df = df.drop_duplicates(subset=["__source_file", "frame_id", "obstacle_id"], keep="last").reset_index(drop=True)

# 2) stop_count binary
if "stop_count" in df.columns:
    df["stop_count"] = (df["stop_count"] >= 1).astype(np.float32)


def add_sincos(col):
    rad = np.deg2rad(df[col].values % 360.0)
    df[col + "_sin"] = np.sin(rad)
    df[col + "_cos"] = np.cos(rad)


add_sincos("obs_heading")
add_sincos("ego_heading")
add_sincos("heading_vs_path")

valid_classes = [1, 20, 41, 42]
for cls in valid_classes:
    df[f"cls_{cls}"] = (df["obstacle_class"] == cls).astype(np.float32)
df.loc[~df["obstacle_class"].isin(valid_classes), [f"cls_{cls}" for cls in valid_classes]] = 0

label_cols = ["c_s1", "c_s2", "c_s3", "c_s4", "c_s5", "c_s6", "c_s9", "c_s10"]
missing_labels = [c for c in label_cols if c not in df.columns]
if missing_labels:
    raise KeyError(f"Missing label columns: {missing_labels}")

df[label_cols] = df[label_cols].fillna(0)
y = df[label_cols].values.astype(np.float32)

print("Label positive counts:")
for i, c in enumerate(label_cols):
    print(f"  {c}: {int((y[:, i] > 0).sum())} / {len(y)}")

class_feature_cols = [f"cls_{cls}" for cls in valid_classes]
continuous_feature_cols = [
    "stop_count",
    "cuboid_x", "cuboid_y", "cuboid_z",
    "obs_x", "obs_y", "obs_vx", "obs_vy", "obs_speed",
    "rel_x", "rel_y", "dist_to_ego", "rel_vx", "rel_vy",
    "ego_speed", "ego_yaw_rate",
    "dist_to_path", "along_path_s",
    "obs_heading_sin", "obs_heading_cos",
    "ego_heading_sin", "ego_heading_cos",
    "heading_vs_path_sin", "heading_vs_path_cos",
]

nan_mask = df[continuous_feature_cols].isna().astype(np.float32).to_numpy()
df = df.fillna(0)
nan_feature_cols = []
if args.use_nan_mask:
    seen = {}
    for i, col in enumerate(continuous_feature_cols):
        cnt = seen.get(col, 0) + 1
        seen[col] = cnt
        name = f"{col}_nan" if cnt == 1 else f"{col}_nan_{cnt}"
        nan_feature_cols.append(name)
        df[name] = nan_mask[:, i]

all_feature_cols = class_feature_cols + nan_feature_cols + continuous_feature_cols

print("=" * 60)
print("Feature Configuration")
print("=" * 60)
print(f"Class features ({len(class_feature_cols)}): {class_feature_cols}")
print(f"Continuous features ({len(continuous_feature_cols)}): {continuous_feature_cols}")
print(f"Total features: {len(all_feature_cols)}")
print("=" * 60)

selected_train_files = normalize_file_list(args.train_files)
selected_val_files = normalize_file_list(args.val_files)
selected_test_files = normalize_file_list(args.test_files)

overlap = (
    (selected_train_files & selected_val_files)
    | (selected_train_files & selected_test_files)
    | (selected_val_files & selected_test_files)
)
if overlap:
    raise ValueError(f"Split file lists overlap: {sorted(overlap)}")

available_files = set(df["__source_file"].unique())
unknown_files = (selected_train_files | selected_val_files | selected_test_files) - available_files
if unknown_files:
    raise ValueError(f"Unknown split files: {sorted(unknown_files)}")

if selected_val_files or selected_test_files or selected_train_files:
    if not selected_train_files:
        selected_train_files = available_files - selected_val_files - selected_test_files

    if not selected_train_files:
        raise ValueError("No training files remain after applying split selection")

    train_mask = df["__source_file"].isin(selected_train_files).to_numpy()
    val_mask = df["__source_file"].isin(selected_val_files).to_numpy()
    test_mask = df["__source_file"].isin(selected_test_files).to_numpy()

    train_idx = np.flatnonzero(train_mask)
    val_idx = np.flatnonzero(val_mask)
    test_idx = np.flatnonzero(test_mask)

    print("Using file-based split:")
    print(f"  train files ({len(selected_train_files)}): {sorted(selected_train_files)}")
    print(f"  val files ({len(selected_val_files)}): {sorted(selected_val_files)}")
    print(f"  test files ({len(selected_test_files)}): {sorted(selected_test_files)}")

    if len(val_idx) == 0:
        raise ValueError("Validation split is empty; provide --val-files or use default split")
else:
    gss = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=42)
    dummy_x = np.zeros((len(df), 1), dtype=np.float32)
    train_idx, val_idx = next(gss.split(dummy_x, y, groups=df["frame_id"].values))
    test_idx = np.array([], dtype=np.int64)

df_train = df.iloc[train_idx].copy()
df_val = df.iloc[val_idx].copy()
df_test = df.iloc[test_idx].copy() if len(test_idx) > 0 else df.iloc[:0].copy()

scaler = StandardScaler()
train_cont = df_train[continuous_feature_cols].to_numpy(dtype=np.float32)
val_cont = df_val[continuous_feature_cols].to_numpy(dtype=np.float32)
test_cont = df_test[continuous_feature_cols].to_numpy(dtype=np.float32) if len(df_test) > 0 else np.zeros((0, len(continuous_feature_cols)), dtype=np.float32)

train_cont_scaled = scaler.fit_transform(train_cont).astype(np.float32)
val_cont_scaled = scaler.transform(val_cont).astype(np.float32)
test_cont_scaled = scaler.transform(test_cont).astype(np.float32) if len(df_test) > 0 else test_cont


def attach_scaled_features(df_part, cont_scaled):
    cls = df_part[class_feature_cols].to_numpy(dtype=np.float32)
    parts = [cls]
    if nan_feature_cols:
        parts.append(df_part[nan_feature_cols].to_numpy(dtype=np.float32))
    parts.append(cont_scaled)
    full = np.hstack(parts).astype(np.float32)
    out = df_part.copy()
    for i, col in enumerate(all_feature_cols):
        out[col] = full[:, i]
    return out


df_train = attach_scaled_features(df_train, train_cont_scaled)
df_val = attach_scaled_features(df_val, val_cont_scaled)
df_test = attach_scaled_features(df_test, test_cont_scaled) if len(df_test) > 0 else df_test

if args.model_type == "temporal_mlp":
    X_train, y_train = build_sequences(df_train, all_feature_cols, label_cols, args.seq_len)
    X_val, y_val = build_sequences(df_val, all_feature_cols, label_cols, args.seq_len)
    X_test, y_test = build_sequences(df_test, all_feature_cols, label_cols, args.seq_len) if len(df_test) > 0 else (
        np.zeros((0, args.seq_len, len(all_feature_cols)), dtype=np.float32),
        np.zeros((0, len(label_cols)), dtype=np.float32),
    )
else:
    X_train = df_train[all_feature_cols].to_numpy(dtype=np.float32)
    y_train = df_train[label_cols].to_numpy(dtype=np.float32)
    X_val = df_val[all_feature_cols].to_numpy(dtype=np.float32)
    y_val = df_val[label_cols].to_numpy(dtype=np.float32)
    X_test = df_test[all_feature_cols].to_numpy(dtype=np.float32) if len(df_test) > 0 else np.zeros((0, len(all_feature_cols)), dtype=np.float32)
    y_test = df_test[label_cols].to_numpy(dtype=np.float32) if len(df_test) > 0 else np.zeros((0, len(label_cols)), dtype=np.float32)

meta = {
    "model_type": args.model_type,
    "seq_len": args.seq_len,
    "label_cols": label_cols,
    "valid_classes": valid_classes,
    "class_feature_cols": class_feature_cols,
    "nan_feature_cols": nan_feature_cols,
    "continuous_feature_cols": continuous_feature_cols,
    "all_feature_cols": all_feature_cols,
    "scaler_mean": scaler.mean_.tolist(),
    "scaler_scale": scaler.scale_.tolist(),
}
with open(os.path.join(out_dir, "risk_meta.json"), "w", encoding="utf-8") as f:
    json.dump(meta, f, ensure_ascii=False, indent=2)
print(f"Saved: {os.path.join(out_dir, 'risk_meta.json')}")

print(f"Model type: {args.model_type}")
print(f"Train shape: {X_train.shape}, Val shape: {X_val.shape}, Test shape: {X_test.shape}")


class RiskDataset(Dataset):
    def __init__(self, x, y):
        self.x = torch.from_numpy(x)
        self.y = torch.from_numpy(y)

    def __len__(self):
        return len(self.x)

    def __getitem__(self, i):
        return self.x[i], self.y[i]


train_loader = DataLoader(RiskDataset(X_train, y_train), batch_size=256, shuffle=True)
val_loader = DataLoader(RiskDataset(X_val, y_val), batch_size=512, shuffle=False)
test_loader = DataLoader(RiskDataset(X_test, y_test), batch_size=512, shuffle=False) if len(X_test) > 0 else None


class MLP(nn.Module):
    def __init__(self, in_dim, out_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, 64), nn.ReLU(),
            nn.Linear(64, out_dim),
        )

    def forward(self, x):
        return self.net(x)


class TemporalMLP(nn.Module):
    def __init__(self, seq_len, feat_dim, out_dim):
        super().__init__()
        self.flatten = nn.Flatten(start_dim=1)
        self.net = nn.Sequential(
            nn.Linear(seq_len * feat_dim, 256), nn.ReLU(),
            nn.Linear(256, 128), nn.ReLU(),
            nn.Linear(128, out_dim),
        )

    def forward(self, x):
        return self.net(self.flatten(x))


device = "cuda" if torch.cuda.is_available() else "cpu"
if args.model_type == "temporal_mlp":
    model = TemporalMLP(args.seq_len, X_train.shape[2], len(label_cols)).to(device)
else:
    model = MLP(X_train.shape[1], len(label_cols)).to(device)

criterion = nn.BCEWithLogitsLoss()
opt = torch.optim.Adam(model.parameters(), lr=1e-3)

for epoch in range(args.epochs):
    model.train()
    tr_loss = 0.0
    for xb, yb in train_loader:
        xb, yb = xb.to(device), yb.to(device)
        opt.zero_grad()
        logits = model(xb)
        loss = criterion(logits, yb)
        loss.backward()
        opt.step()
        tr_loss += loss.item() * xb.size(0)
    tr_loss /= len(train_loader.dataset)

    print(f"epoch={epoch:02d} train_loss={tr_loss:.4f}", end=" ")
    evaluate_split("val", val_loader)

if test_loader is not None:
    print("Final test metrics")
    evaluate_split("test", test_loader)

model.eval()
dummy = torch.from_numpy(X_train[:1]).to(device)
torch.onnx.export(
    model,
    dummy,
    os.path.join(out_dir, "risk_mlp.onnx"),
    opset_version=17,
    input_names=["input"],
    output_names=["logits"],
    dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}},
)

print(f"Saved: {os.path.join(out_dir, 'risk_mlp.onnx')}")
