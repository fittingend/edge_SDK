#!/usr/bin/env python3
import csv
import math
import sys
from typing import Dict, List

PEDESTRIAN_CLASS = 20
VEHICLE_CLASS_MIN = 1
VEHICLE_CLASS_MAX = 19

EPS = 1e-9


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


def is_finite(x: float) -> bool:
    return not math.isnan(x) and not math.isinf(x)


def compute_ttc(rx, ry, rvx, rvy):
    v2 = rvx * rvx + rvy * rvy
    if v2 < EPS:
        return float('inf')
    rv = rx * rvx + ry * rvy
    if rv >= 0.0:
        return float('inf')
    return -rv / v2


def compute_min_linear_dist(rx, ry, rvx, rvy):
    v2 = rvx * rvx + rvy * rvy
    r2 = rx * rx + ry * ry
    if v2 < EPS:
        return math.sqrt(r2)
    rv = rx * rvx + ry * rvy
    d2 = r2 - (rv * rv) / v2
    if d2 < 0.0:
        d2 = 0.0
    return math.sqrt(d2)


def finalize_frame(rows: List[Dict[str, str]], writer, first_seen: Dict[str, float]):
    if not rows:
        return

    # Pre-compute pairwise min distances for pedestrians and vehicles
    ped_indices = []
    veh_indices = []

    for i, r in enumerate(rows):
        cls = int(parse_float(r["obstacle_class"])) if r.get("obstacle_class") else -1
        if cls == PEDESTRIAN_CLASS:
            ped_indices.append(i)
        if VEHICLE_CLASS_MIN <= cls <= VEHICLE_CLASS_MAX:
            veh_indices.append(i)

    min_ped = [float('inf')] * len(rows)
    min_veh = [float('inf')] * len(rows)

    def update_min(indices, min_arr):
        for i in range(len(indices)):
            a = indices[i]
            ax = parse_float(rows[a]["obs_x"])
            ay = parse_float(rows[a]["obs_y"])
            for j in range(i + 1, len(indices)):
                b = indices[j]
                bx = parse_float(rows[b]["obs_x"])
                by = parse_float(rows[b]["obs_y"])
                if not (is_finite(ax) and is_finite(ay) and is_finite(bx) and is_finite(by)):
                    continue
                d = math.hypot(ax - bx, ay - by)
                if d < min_arr[a]:
                    min_arr[a] = d
                if d < min_arr[b]:
                    min_arr[b] = d

    update_min(ped_indices, min_ped)
    update_min(veh_indices, min_veh)

    for idx, r in enumerate(rows):
        ox = parse_float(r["obs_x"])
        oy = parse_float(r["obs_y"])
        ovx = parse_float(r["obs_vx"])
        ovy = parse_float(r["obs_vy"])
        ex = parse_float(r["ego_x"])
        ey = parse_float(r["ego_y"])
        evx = parse_float(r["ego_vx"])
        evy = parse_float(r["ego_vy"])
        gx = parse_float(r.get("goal_x", "nan"))
        gy = parse_float(r.get("goal_y", "nan"))

        rel_x = ox - ex
        rel_y = oy - ey
        rel_vx = ovx - evx
        rel_vy = ovy - evy
        dist_to_ego = math.hypot(rel_x, rel_y) if is_finite(rel_x) and is_finite(rel_y) else float('nan')
        obs_speed = math.hypot(ovx, ovy) if is_finite(ovx) and is_finite(ovy) else float('nan')
        ego_speed = math.hypot(evx, evy) if is_finite(evx) and is_finite(evy) else float('nan')

        dist_to_goal = float('nan')
        if is_finite(gx) and is_finite(gy) and is_finite(ox) and is_finite(oy):
            dist_to_goal = math.hypot(ox - gx, oy - gy)

        ttc = compute_ttc(rel_x, rel_y, rel_vx, rel_vy) if is_finite(rel_x) and is_finite(rel_y) and is_finite(rel_vx) and is_finite(rel_vy) else float('nan')
        lin_min = compute_min_linear_dist(rel_x, rel_y, rel_vx, rel_vy) if is_finite(rel_x) and is_finite(rel_y) and is_finite(rel_vx) and is_finite(rel_vy) else float('nan')

        oid = r.get("obstacle_id", "")
        ts = parse_float(r.get("obs_timestamp_ms", "nan"))
        if oid and is_finite(ts):
            if oid not in first_seen:
                first_seen[oid] = ts
            first_ts = first_seen[oid]
            time_since_fs = ts - first_ts
        else:
            time_since_fs = float('nan')

        r["rel_x"] = f"{rel_x:.6f}" if is_finite(rel_x) else "nan"
        r["rel_y"] = f"{rel_y:.6f}" if is_finite(rel_y) else "nan"
        r["dist_to_ego"] = f"{dist_to_ego:.6f}" if is_finite(dist_to_ego) else "nan"
        r["rel_vx"] = f"{rel_vx:.6f}" if is_finite(rel_vx) else "nan"
        r["rel_vy"] = f"{rel_vy:.6f}" if is_finite(rel_vy) else "nan"
        r["obs_speed"] = f"{obs_speed:.6f}" if is_finite(obs_speed) else "nan"
        r["ego_speed"] = f"{ego_speed:.6f}" if is_finite(ego_speed) else "nan"
        r["dist_to_goal"] = f"{dist_to_goal:.6f}" if is_finite(dist_to_goal) else "nan"
        r["ttc"] = f"{ttc:.6f}" if is_finite(ttc) else "inf"
        r["lin_min_dist"] = f"{lin_min:.6f}" if is_finite(lin_min) else "nan"
        r["time_since_first_seen_ms"] = f"{time_since_fs:.3f}" if is_finite(time_since_fs) else "nan"

        mp = min_ped[idx]
        mv = min_veh[idx]
        r["min_pair_dist_ped"] = f"{mp:.6f}" if math.isfinite(mp) else "nan"
        r["min_pair_dist_vehicle"] = f"{mv:.6f}" if math.isfinite(mv) else "nan"

        writer.writerow(r)


def main():
    if len(sys.argv) != 3:
        print("Usage: preprocess_autolabels.py <input.csv> <output.csv>")
        sys.exit(1)

    in_path = sys.argv[1]
    out_path = sys.argv[2]

    with open(in_path, "r", newline="") as f_in, open(out_path, "w", newline="") as f_out:
        reader = csv.DictReader(f_in)
        if reader.fieldnames is None:
            print("Empty input CSV")
            sys.exit(1)

        base_fields = list(reader.fieldnames)
        extra_fields = [
            "rel_x", "rel_y", "dist_to_ego",
            "rel_vx", "rel_vy",
            "obs_speed", "ego_speed",
            "dist_to_goal", "ttc", "lin_min_dist",
            "time_since_first_seen_ms",
            "min_pair_dist_ped", "min_pair_dist_vehicle",
        ]
        writer = csv.DictWriter(f_out, fieldnames=base_fields + extra_fields)
        writer.writeheader()

        current_frame = None
        buffer = []
        first_seen = {}

        for row in reader:
            frame_id = row.get("frame_id")
            if current_frame is None:
                current_frame = frame_id
            if frame_id != current_frame:
                finalize_frame(buffer, writer, first_seen)
                buffer = []
                current_frame = frame_id
            buffer.append(row)

        finalize_frame(buffer, writer, first_seen)


if __name__ == "__main__":
    main()
