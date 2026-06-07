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


ap = argparse.ArgumentParser(description="Train risk MLP and export ONNX.")
ap.add_argument(
    "--log-dir",
    default="log",
    help="directory with auto_labels_*.csv and label*.csv",
)
ap.add_argument("--out-dir", default="artifacts", help="output directory for model/meta")
ap.add_argument("--train-files", nargs="*", default=None, help="basenames to use for train split")
ap.add_argument("--val-files", nargs="*", default=None, help="basenames to use for validation split")
ap.add_argument("--test-files", nargs="*", default=None, help="basenames to use for test split")
args = ap.parse_args()

log_folder = resolve_path(args.log_dir)
out_dir = resolve_path(args.out_dir)
os.makedirs(out_dir, exist_ok=True)

# 1) Load ALL CSVs from log folder
csv_patterns = ["auto_labels_*.csv", "label*.csv"]
csv_files = sorted(
    {
        path
        for pattern in csv_patterns
        for path in glob.glob(os.path.join(log_folder, pattern))
    }
)

if csv_files:
    print("Available CSV files:")
    for i, f in enumerate(csv_files):
        print(f"  {i}: {os.path.basename(f)}")

    # 모든 CSV를 로드해서 concat
    dfs = []
    for f in csv_files:
        df_part = pd.read_csv(f)
        df_part["__source_file"] = os.path.basename(f)
        dfs.append(df_part)
    df = pd.concat(dfs, ignore_index=True)
    print(f"\nLoaded {len(csv_files)} files, total rows: {len(df)}")
else:
    patterns = ", ".join(csv_patterns)
    raise FileNotFoundError(f"No CSV files found in {log_folder} for patterns: {patterns}")


def normalize_file_list(file_list):
    if not file_list:
        return set()
    return {os.path.basename(path) for path in file_list}


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
            ap = average_precision_score(y_true_bin[:, i], y_prob[:, i])
        except ValueError:
            ap = float("nan")
        print(f"  {label}: ROC-AUC={roc:.4f} PR-AUC={ap:.4f}")

    return split_loss

# 2) Dedup (frame_id, obstacle_id)
df = df.drop_duplicates(subset=["frame_id", "obstacle_id"], keep="last").reset_index(drop=True)

# 3) NaN 처리 (마스크는 feature 정의 이후 생성)
# 결측치는 마스크 생성 후 0으로 채움

# stop_count: 0이면 이동, 1 이상이면 동적 장애물 (binary)
if "stop_count" in df.columns:
    df["stop_count"] = (df["stop_count"] >= 1).astype(np.float32)

# 4) heading sin/cos 변환 (deg -> rad)
def add_sincos(col):
    rad = np.deg2rad(df[col].values % 360.0)
    df[col + "_sin"] = np.sin(rad)
    df[col + "_cos"] = np.cos(rad)

add_sincos("obs_heading")
add_sincos("ego_heading")
add_sincos("heading_vs_path")

# 5) obstacle_class one-hot 인코딩
# 실제 사용하는 클래스: 1, 20, 41, 42
valid_classes = [1, 20, 41, 42]
for cls in valid_classes:
    df[f"cls_{cls}"] = (df["obstacle_class"] == cls).astype(np.float32)

# 유효하지 않은 클래스값 명시적으로 0 처리
df.loc[~df["obstacle_class"].isin(valid_classes), [f"cls_{cls}" for cls in valid_classes]] = 0

# 6) 학습할 라벨 선택 (confidence scores)
label_cols = ["c_s1", "c_s2", "c_s3", "c_s4", "c_s5", "c_s6", "c_s9", "c_s10"]
# 라벨 NaN은 0으로 처리
df[label_cols] = df[label_cols].fillna(0)
y = df[label_cols].values.astype(np.float32)

# 라벨 컬럼 존재 여부/분포 확인
missing_labels = [c for c in label_cols if c not in df.columns]
if missing_labels:
    raise KeyError(f"Missing label columns: {missing_labels}")

print("Label positive counts:")
for i, c in enumerate(label_cols):
    print(f"  {c}: {int((y[:, i] > 0).sum())} / {len(y)}")

# 7) feature 컬럼 구성
# 7-1) class one-hot features
class_feature_cols = [f"cls_{cls}" for cls in valid_classes]

# 7-2) continuous features (one-hot 미적용)
continuous_feature_cols = [
    "stop_count",
    "cuboid_x","cuboid_y","cuboid_z",
    "obs_x","obs_y","obs_vx","obs_vy","obs_speed",
    "rel_x","rel_y","dist_to_ego","rel_vx","rel_vy",
    "ego_speed","ego_yaw_rate",
    "dist_to_path","along_path_s",
    "obs_heading_sin","obs_heading_cos",
    "ego_heading_sin","ego_heading_cos",
    "heading_vs_path_sin","heading_vs_path_cos",
]
# 7-2b) NaN mask for continuous features (will not be scaled)
# NOTE: handle duplicate column names by writing masks from ndarray
nan_mask = df[continuous_feature_cols].isna().astype(np.float32).to_numpy()

nan_feature_cols = []
seen = {}
for i, col in enumerate(continuous_feature_cols):
    cnt = seen.get(col, 0) + 1
    seen[col] = cnt
    name = f"{col}_nan" if cnt == 1 else f"{col}_nan_{cnt}"
    nan_feature_cols.append(name)
    df[name] = nan_mask[:, i]

# 이제 결측치를 0으로 채움
df = df.fillna(0)



# 7-3) 최종 feature (one-hot + continuous 순)
all_feature_cols = class_feature_cols + nan_feature_cols + continuous_feature_cols

print("=" * 60)
print("Feature Configuration")
print("=" * 60)
print(f"Class features ({len(class_feature_cols)}): {class_feature_cols}")
print(f"Continuous features ({len(continuous_feature_cols)}): {continuous_feature_cols}")
print(f"Total features: {len(all_feature_cols)}")
print("=" * 60)

X = df[all_feature_cols].values.astype(np.float32)

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
    # 8) 프레임 단위 split (group = frame_id)
    gss = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=df["frame_id"].values))
    test_idx = np.array([], dtype=np.int64)

X_train, y_train = X[train_idx], y[train_idx]
X_val, y_val = X[val_idx], y[val_idx]
X_test, y_test = X[test_idx], y[test_idx]

# 9) 표준화 (연속값만, one-hot은 제외)
# one-hot features: 처음 len(class_feature_cols)개 열
# continuous features: 나머지
n_class_features = len(class_feature_cols) + len(nan_feature_cols)

X_train_cls = X_train[:, :n_class_features]  # one-hot (변환 없음)
X_train_cont = X_train[:, n_class_features:]  # continuous

X_val_cls = X_val[:, :n_class_features]
X_val_cont = X_val[:, n_class_features:]
X_test_cls = X_test[:, :n_class_features]
X_test_cont = X_test[:, n_class_features:]

scaler = StandardScaler()
X_train_cont_scaled = scaler.fit_transform(X_train_cont).astype(np.float32)
X_val_cont_scaled = scaler.transform(X_val_cont).astype(np.float32)
X_test_cont_scaled = scaler.transform(X_test_cont).astype(np.float32) if len(X_test) > 0 else X_test_cont.astype(np.float32)

# 최종 X: [one-hot + scaled continuous]
X_train = np.hstack([X_train_cls, X_train_cont_scaled]).astype(np.float32)
X_val = np.hstack([X_val_cls, X_val_cont_scaled]).astype(np.float32)
X_test = np.hstack([X_test_cls, X_test_cont_scaled]).astype(np.float32) if len(X_test) > 0 else X_test.astype(np.float32)


# 메타 저장 (feature 순서/스케일러 보존)
meta = {
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

# 10) Dataset
class RiskDataset(Dataset):
    def __init__(self, X, y):
        self.X = torch.from_numpy(X)
        self.y = torch.from_numpy(y)
    def __len__(self):
        return len(self.X)
    def __getitem__(self, i):
        return self.X[i], self.y[i]

train_loader = DataLoader(RiskDataset(X_train, y_train), batch_size=256, shuffle=True)
val_loader = DataLoader(RiskDataset(X_val, y_val), batch_size=512, shuffle=False)
test_loader = None
if len(X_test) > 0:
    test_loader = DataLoader(RiskDataset(X_test, y_test), batch_size=512, shuffle=False)

# 12) Model
class MLP(nn.Module):
    def __init__(self, in_dim, out_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, 64), nn.ReLU(),
            nn.Linear(64, out_dim)
        )
    def forward(self, x):
        return self.net(x)

device = "cuda" if torch.cuda.is_available() else "cpu"
model = MLP(X_train.shape[1], len(label_cols)).to(device)

criterion = nn.BCEWithLogitsLoss()
opt = torch.optim.Adam(model.parameters(), lr=1e-3)

# 13) Train loop (간단)
for epoch in range(20):
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

# 14) ONNX export
model.eval()
dummy = torch.from_numpy(X_train[:1]).to(device)
torch.onnx.export(
    model,
    dummy,
    os.path.join(out_dir, "risk_mlp.onnx"),
    opset_version=17,
    input_names=["input"],
    output_names=["logits"],
    dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}}
)

print(f"Saved: {os.path.join(out_dir, 'risk_mlp.onnx')}")
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


ap = argparse.ArgumentParser(description="Train risk MLP and export ONNX.")
ap.add_argument(
    "--log-dir",
    default="log",
    help="directory with auto_labels_*.csv and label*.csv",
)
ap.add_argument("--out-dir", default="artifacts", help="output directory for model/meta")
ap.add_argument("--train-files", nargs="*", default=None, help="basenames to use for train split")
ap.add_argument("--val-files", nargs="*", default=None, help="basenames to use for validation split")
ap.add_argument("--test-files", nargs="*", default=None, help="basenames to use for test split")
args = ap.parse_args()

log_folder = resolve_path(args.log_dir)
out_dir = resolve_path(args.out_dir)
os.makedirs(out_dir, exist_ok=True)

# 1) Load ALL CSVs from log folder
csv_patterns = ["auto_labels_*.csv", "label*.csv"]
csv_files = sorted(
    {
        path
        for pattern in csv_patterns
        for path in glob.glob(os.path.join(log_folder, pattern))
    }
)

if csv_files:
    print("Available CSV files:")
    for i, f in enumerate(csv_files):
        print(f"  {i}: {os.path.basename(f)}")

    # 모든 CSV를 로드해서 concat
    dfs = []
    for f in csv_files:
        df_part = pd.read_csv(f)
        df_part["__source_file"] = os.path.basename(f)
        dfs.append(df_part)
    df = pd.concat(dfs, ignore_index=True)
    print(f"\nLoaded {len(csv_files)} files, total rows: {len(df)}")
else:
    patterns = ", ".join(csv_patterns)
    raise FileNotFoundError(f"No CSV files found in {log_folder} for patterns: {patterns}")


def normalize_file_list(file_list):
    if not file_list:
        return set()
    return {os.path.basename(path) for path in file_list}


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
            ap = average_precision_score(y_true_bin[:, i], y_prob[:, i])
        except ValueError:
            ap = float("nan")
        print(f"  {label}: ROC-AUC={roc:.4f} PR-AUC={ap:.4f}")

    return split_loss

# 2) Dedup (frame_id, obstacle_id)
df = df.drop_duplicates(subset=["frame_id", "obstacle_id"], keep="last").reset_index(drop=True)

# 3) NaN 처리 (마스크는 feature 정의 이후 생성)
# 결측치는 마스크 생성 후 0으로 채움

# stop_count: 0이면 이동, 1 이상이면 동적 장애물 (binary)
if "stop_count" in df.columns:
    df["stop_count"] = (df["stop_count"] >= 1).astype(np.float32)

# 4) heading sin/cos 변환 (deg -> rad)
def add_sincos(col):
    rad = np.deg2rad(df[col].values % 360.0)
    df[col + "_sin"] = np.sin(rad)
    df[col + "_cos"] = np.cos(rad)

add_sincos("obs_heading")
add_sincos("ego_heading")
add_sincos("heading_vs_path")

# 5) obstacle_class one-hot 인코딩
# 실제 사용하는 클래스: 1, 20, 41, 42
valid_classes = [1, 20, 41, 42]
for cls in valid_classes:
    df[f"cls_{cls}"] = (df["obstacle_class"] == cls).astype(np.float32)

# 유효하지 않은 클래스값 명시적으로 0 처리
df.loc[~df["obstacle_class"].isin(valid_classes), [f"cls_{cls}" for cls in valid_classes]] = 0

# 6) 학습할 라벨 선택 (confidence scores)
label_cols = ["c_s1", "c_s2", "c_s3", "c_s4", "c_s5", "c_s6", "c_s9", "c_s10"]
# 라벨 NaN은 0으로 처리
df[label_cols] = df[label_cols].fillna(0)
y = df[label_cols].values.astype(np.float32)

# 라벨 컬럼 존재 여부/분포 확인
missing_labels = [c for c in label_cols if c not in df.columns]
if missing_labels:
    raise KeyError(f"Missing label columns: {missing_labels}")

print("Label positive counts:")
for i, c in enumerate(label_cols):
    print(f"  {c}: {int((y[:, i] > 0).sum())} / {len(y)}")

# 7) feature 컬럼 구성
# 7-1) class one-hot features
class_feature_cols = [f"cls_{cls}" for cls in valid_classes]

# 7-2) continuous features (one-hot 미적용)
continuous_feature_cols = [
    "stop_count",
    "cuboid_x","cuboid_y","cuboid_z",
    "obs_x","obs_y","obs_vx","obs_vy","obs_speed",
    "rel_x","rel_y","dist_to_ego","rel_vx","rel_vy",
    "ego_speed","ego_yaw_rate",
    "dist_to_path","along_path_s",
    "obs_heading_sin","obs_heading_cos",
    "ego_heading_sin","ego_heading_cos",
    "heading_vs_path_sin","heading_vs_path_cos",
]
# 7-2b) NaN mask for continuous features (will not be scaled)
# NOTE: handle duplicate column names by writing masks from ndarray
nan_mask = df[continuous_feature_cols].isna().astype(np.float32).to_numpy()

nan_feature_cols = []
seen = {}
for i, col in enumerate(continuous_feature_cols):
    cnt = seen.get(col, 0) + 1
    seen[col] = cnt
    name = f"{col}_nan" if cnt == 1 else f"{col}_nan_{cnt}"
    nan_feature_cols.append(name)
    df[name] = nan_mask[:, i]

# 이제 결측치를 0으로 채움
df = df.fillna(0)



# 7-3) 최종 feature (one-hot + continuous 순)
all_feature_cols = class_feature_cols + nan_feature_cols + continuous_feature_cols

print("=" * 60)
print("Feature Configuration")
print("=" * 60)
print(f"Class features ({len(class_feature_cols)}): {class_feature_cols}")
print(f"Continuous features ({len(continuous_feature_cols)}): {continuous_feature_cols}")
print(f"Total features: {len(all_feature_cols)}")
print("=" * 60)

X = df[all_feature_cols].values.astype(np.float32)

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
    # 8) 프레임 단위 split (group = frame_id)
    gss = GroupShuffleSplit(n_splits=1, test_size=0.2, random_state=42)
    train_idx, val_idx = next(gss.split(X, y, groups=df["frame_id"].values))
    test_idx = np.array([], dtype=np.int64)

X_train, y_train = X[train_idx], y[train_idx]
X_val, y_val = X[val_idx], y[val_idx]
X_test, y_test = X[test_idx], y[test_idx]

# 9) 표준화 (연속값만, one-hot은 제외)
# one-hot features: 처음 len(class_feature_cols)개 열
# continuous features: 나머지
n_class_features = len(class_feature_cols) + len(nan_feature_cols)

X_train_cls = X_train[:, :n_class_features]  # one-hot (변환 없음)
X_train_cont = X_train[:, n_class_features:]  # continuous

X_val_cls = X_val[:, :n_class_features]
X_val_cont = X_val[:, n_class_features:]
X_test_cls = X_test[:, :n_class_features]
X_test_cont = X_test[:, n_class_features:]

scaler = StandardScaler()
X_train_cont_scaled = scaler.fit_transform(X_train_cont).astype(np.float32)
X_val_cont_scaled = scaler.transform(X_val_cont).astype(np.float32)
X_test_cont_scaled = scaler.transform(X_test_cont).astype(np.float32) if len(X_test) > 0 else X_test_cont.astype(np.float32)

# 최종 X: [one-hot + scaled continuous]
X_train = np.hstack([X_train_cls, X_train_cont_scaled]).astype(np.float32)
X_val = np.hstack([X_val_cls, X_val_cont_scaled]).astype(np.float32)
X_test = np.hstack([X_test_cls, X_test_cont_scaled]).astype(np.float32) if len(X_test) > 0 else X_test.astype(np.float32)


# 메타 저장 (feature 순서/스케일러 보존)
meta = {
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

# 10) Dataset
class RiskDataset(Dataset):
    def __init__(self, X, y):
        self.X = torch.from_numpy(X)
        self.y = torch.from_numpy(y)
    def __len__(self):
        return len(self.X)
    def __getitem__(self, i):
        return self.X[i], self.y[i]

train_loader = DataLoader(RiskDataset(X_train, y_train), batch_size=256, shuffle=True)
val_loader = DataLoader(RiskDataset(X_val, y_val), batch_size=512, shuffle=False)
test_loader = None
if len(X_test) > 0:
    test_loader = DataLoader(RiskDataset(X_test, y_test), batch_size=512, shuffle=False)

# 12) Model
class MLP(nn.Module):
    def __init__(self, in_dim, out_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, 64), nn.ReLU(),
            nn.Linear(64, out_dim)
        )
    def forward(self, x):
        return self.net(x)

device = "cuda" if torch.cuda.is_available() else "cpu"
model = MLP(X_train.shape[1], len(label_cols)).to(device)

criterion = nn.BCEWithLogitsLoss()
opt = torch.optim.Adam(model.parameters(), lr=1e-3)

# 13) Train loop (간단)
for epoch in range(20):
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

# 14) ONNX export
model.eval()
dummy = torch.from_numpy(X_train[:1]).to(device)
torch.onnx.export(
    model,
    dummy,
    os.path.join(out_dir, "risk_mlp.onnx"),
    opset_version=17,
    input_names=["input"],
    output_names=["logits"],
    dynamic_axes={"input": {0: "batch"}, "logits": {0: "batch"}}
)

print(f"Saved: {os.path.join(out_dir, 'risk_mlp.onnx')}")
