#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
from typing import Dict, List

import numpy as np
import onnxruntime as ort


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def parse_float(s: str) -> float:
    if s is None:
        return float('nan')
    s = s.strip()
    if s == "" or s.lower() == "nan":
        return float('nan')
    try:
        return float(s)
    except ValueError:
        return float('nan')


def parse_int(s: str) -> int:
    if s is None:
        return 0
    s = s.strip()
    if s == "" or s.lower() == "nan":
        return 0
    try:
        return int(float(s))
    except ValueError:
        return 0


def is_finite(x: float) -> bool:
    return not math.isnan(x) and not math.isinf(x)


def load_meta(path: str) -> Dict:
    with open(path, "r") as f:
        meta = json.load(f)
    required = [
        "label_cols",
        "valid_classes",
        "class_feature_cols",
        "nan_feature_cols",
        "continuous_feature_cols",
        "scaler_mean",
        "scaler_scale",
    ]
    for k in required:
        if k not in meta:
            raise ValueError(f"meta missing key: {k}")
    if len(meta["continuous_feature_cols"]) != len(meta["scaler_mean"]) or \
       len(meta["continuous_feature_cols"]) != len(meta["scaler_scale"]):
        raise ValueError("meta mismatch: continuous_feature_cols / scaler sizes")
    return meta


def get_continuous(row: Dict[str, str], col: str) -> float:
    # Mirrors OnnxRiskInfer::inferFromRawRows logic
    if col == "stop_count":
        return 1.0 if parse_float(row.get("stop_count", "")) >= 1.0 else 0.0
    if col == "cuboid_x":
        return parse_float(row.get("cuboid_x", ""))
    if col == "cuboid_y":
        return parse_float(row.get("cuboid_y", ""))
    if col == "cuboid_z":
        return parse_float(row.get("cuboid_z", ""))
    if col == "obs_x":
        return parse_float(row.get("obs_x", ""))
    if col == "obs_y":
        return parse_float(row.get("obs_y", ""))
    if col == "obs_vx":
        return parse_float(row.get("obs_vx", ""))
    if col == "obs_vy":
        return parse_float(row.get("obs_vy", ""))
    if col == "obs_speed":
        vx = parse_float(row.get("obs_vx", ""))
        vy = parse_float(row.get("obs_vy", ""))
        return math.hypot(vx, vy) if is_finite(vx) and is_finite(vy) else float('nan')
    if col == "rel_x":
        return parse_float(row.get("obs_x", "")) - parse_float(row.get("ego_x", ""))
    if col == "rel_y":
        return parse_float(row.get("obs_y", "")) - parse_float(row.get("ego_y", ""))
    if col == "dist_to_ego":
        ox = parse_float(row.get("obs_x", ""))
        oy = parse_float(row.get("obs_y", ""))
        ex = parse_float(row.get("ego_x", ""))
        ey = parse_float(row.get("ego_y", ""))
        return math.hypot(ox - ex, oy - ey) if all(is_finite(v) for v in [ox, oy, ex, ey]) else float('nan')
    if col == "rel_vx":
        return parse_float(row.get("obs_vx", "")) - parse_float(row.get("ego_vx", ""))
    if col == "rel_vy":
        return parse_float(row.get("obs_vy", "")) - parse_float(row.get("ego_vy", ""))
    if col == "ego_speed":
        evx = parse_float(row.get("ego_vx", ""))
        evy = parse_float(row.get("ego_vy", ""))
        return math.hypot(evx, evy) if is_finite(evx) and is_finite(evy) else float('nan')
    if col == "ego_yaw_rate":
        return parse_float(row.get("ego_yaw_rate", ""))
    if col == "dist_to_path":
        return parse_float(row.get("dist_to_path", ""))
    if col == "along_path_s":
        return parse_float(row.get("along_path_s", ""))
    if col == "obs_heading_sin":
        deg = parse_float(row.get("obs_heading", ""))
        return math.sin(math.radians(deg)) if is_finite(deg) else float('nan')
    if col == "obs_heading_cos":
        deg = parse_float(row.get("obs_heading", ""))
        return math.cos(math.radians(deg)) if is_finite(deg) else float('nan')
    if col == "ego_heading_sin":
        deg = parse_float(row.get("ego_heading", ""))
        return math.sin(math.radians(deg)) if is_finite(deg) else float('nan')
    if col == "ego_heading_cos":
        deg = parse_float(row.get("ego_heading", ""))
        return math.cos(math.radians(deg)) if is_finite(deg) else float('nan')
    if col == "heading_vs_path_sin":
        deg = parse_float(row.get("heading_vs_path", ""))
        return math.sin(math.radians(deg)) if is_finite(deg) else float('nan')
    if col == "heading_vs_path_cos":
        deg = parse_float(row.get("heading_vs_path", ""))
        return math.cos(math.radians(deg)) if is_finite(deg) else float('nan')
    return float('nan')


def build_features(rows: List[Dict[str, str]], meta: Dict) -> np.ndarray:
    valid_classes = meta["valid_classes"]
    cont_cols = meta["continuous_feature_cols"]
    mean = np.array(meta["scaler_mean"], dtype=np.float32)
    scale = np.array(meta["scaler_scale"], dtype=np.float32)

    feature_dim = len(meta["class_feature_cols"]) + len(meta["nan_feature_cols"]) + len(cont_cols)
    out = np.zeros((len(rows), feature_dim), dtype=np.float32)

    for i, row in enumerate(rows):
        cls = parse_int(row.get("obstacle_class", "0"))
        offset = 0
        for v in valid_classes:
            out[i, offset] = 1.0 if cls == v else 0.0
            offset += 1

        cont_vals = []
        nan_mask = []
        for col in cont_cols:
            val = get_continuous(row, col)
            if is_finite(val):
                nan_mask.append(0.0)
                cont_vals.append(val)
            else:
                nan_mask.append(1.0)
                cont_vals.append(0.0)

        out[i, offset:offset + len(nan_mask)] = np.array(nan_mask, dtype=np.float32)
        offset += len(nan_mask)

        cont = np.array(cont_vals, dtype=np.float32)
        denom = np.where(scale == 0.0, 1.0, scale)
        scaled = (cont - mean) / denom
        out[i, offset:offset + len(cont)] = scaled

    return out


def read_csv_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        return list(reader)


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline ONNX inference (python).")
    ap.add_argument("--csv", required=True, help="input CSV (raw features)")
    ap.add_argument("--meta", required=True, help="risk_meta.json")
    ap.add_argument("--model", required=True, help="risk_mlp.onnx")
    ap.add_argument("--out", default="onnx_inference_result.csv", help="output CSV")
    ap.add_argument("--threshold", type=float, default=0.5, help="prediction threshold")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        raise SystemExit(f"CSV not found: {args.csv}")
    if not os.path.exists(args.meta):
        raise SystemExit(f"Meta not found: {args.meta}")
    if not os.path.exists(args.model):
        raise SystemExit(f"Model not found: {args.model}")

    meta = load_meta(args.meta)
    rows = read_csv_rows(args.csv)
    if not rows:
        raise SystemExit("CSV has no rows")

    features = build_features(rows, meta)

    sess = ort.InferenceSession(args.model, providers=["CPUExecutionProvider"])
    input_name = sess.get_inputs()[0].name
    output_name = sess.get_outputs()[0].name

    logits = sess.run([output_name], {input_name: features})[0]
    probs = sigmoid(logits)
    preds = (probs >= args.threshold).astype(np.int32)

    labels = meta["label_cols"]
    with open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["row_idx"]
        if "obstacle_id" in rows[0]:
            header.append("obstacle_id")
        for lbl in labels:
            header += [f"{lbl}_logit", f"{lbl}_prob", f"{lbl}_pred"]
        writer.writerow(header)

        for i in range(len(rows)):
            line = [i]
            if "obstacle_id" in rows[0]:
                line.append(rows[i].get("obstacle_id", ""))
            for j in range(len(labels)):
                line += [float(logits[i, j]), float(probs[i, j]), int(preds[i, j])]
            writer.writerow(line)

    print(f"Saved: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
