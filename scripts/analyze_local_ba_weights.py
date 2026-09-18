#!/usr/bin/env python3
"""Compare Local-BA IMU weight experiments from FAST-LIO CSV logs.

Example:
  python3 scripts/analyze_local_ba_weights.py \
    A=FAST-LIO2_local_ba_A.csv B=FAST-LIO2_local_ba_B.csv C=FAST-LIO2_local_ba_C.csv
"""
import csv
import math
import statistics
import sys
import argparse
from pathlib import Path


METRICS = (
    "ba_translation_correction", "ba_rotation_correction_deg", "ba_velocity_correction",
    "lidar_rmse_before", "lidar_rmse_after",
    "imu_rot_rmse_before", "imu_rot_rmse_after",
    "imu_vel_rmse_before", "imu_vel_rmse_after",
    "imu_pos_rmse_before", "imu_pos_rmse_after",
    "total_cost_before", "total_cost_after",
)


def number(row, key):
    try:
        value = float(row.get(key, "nan"))
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def flag(row, key):
    return row.get(key, "0").strip().lower() in ("1", "true")


def pct_improved(rows, before, after):
    pairs = [(number(r, before), number(r, after)) for r in rows]
    pairs = [(b, a) for b, a in pairs if b is not None and a is not None]
    return None if not pairs else 100.0 * sum(a <= b for b, a in pairs) / len(pairs)


def paired_medians(rows, before, after):
    pairs = [(number(r, before), number(r, after)) for r in rows]
    pairs = [(b, a) for b, a in pairs if b is not None and a is not None]
    if not pairs:
        return None
    return statistics.median(b for b, _ in pairs), statistics.median(a for _, a in pairs)


def summary(rows):
    executed = [r for r in rows if flag(r, "ba_executed")]
    accepted = [r for r in executed if flag(r, "ba_accepted")]
    out = {"executed": len(executed), "accepted": len(accepted)}
    out["reject_pct"] = None if not executed else 100.0 * (1.0 - len(accepted) / len(executed))
    for metric in METRICS:
        values = [number(r, metric) for r in executed]
        values = [v for v in values if v is not None]
        if values:
            out[metric] = (statistics.median(values), statistics.pvariance(values), statistics.mean(values))
    out["lidar_improve_pct"] = pct_improved(executed, "lidar_rmse_before", "lidar_rmse_after")
    out["rot_improve_pct"] = pct_improved(executed, "imu_rot_rmse_before", "imu_rot_rmse_after")
    out["vel_improve_pct"] = pct_improved(executed, "imu_vel_rmse_before", "imu_vel_rmse_after")
    out["pos_improve_pct"] = pct_improved(executed, "imu_pos_rmse_before", "imu_pos_rmse_after")
    out["cost_improve_pct"] = pct_improved(executed, "total_cost_before", "total_cost_after")
    for name, before, after in (
        ("lidar", "lidar_rmse_before", "lidar_rmse_after"),
        ("imu_rot", "imu_rot_rmse_before", "imu_rot_rmse_after"),
        ("imu_vel", "imu_vel_rmse_before", "imu_vel_rmse_after"),
        ("imu_pos", "imu_pos_rmse_before", "imu_pos_rmse_after"),
        ("total_cost", "total_cost_before", "total_cost_after"),
    ):
        out[name + "_medians"] = paired_medians(executed, before, after)
    return out


def print_summary(label, section, data):
    print(f"\n[{label}] {section}: executed={data['executed']} accepted={data['accepted']} "
          f"reject={data['reject_pct'] if data['reject_pct'] is not None else float('nan'):.2f}%")
    for key in ("ba_translation_correction", "ba_rotation_correction_deg", "ba_velocity_correction"):
        if key in data:
            med, var, mean = data[key]
            print(f"  {key}: median={med:.6f}, variance={var:.6f}, mean={mean:.6f}")
    print("  before -> after median; improvement rate (%):")
    for name, med_key, pct_key in (
        ("lidar", "lidar_medians", "lidar_improve_pct"),
        ("imu_rot", "imu_rot_medians", "rot_improve_pct"),
        ("imu_vel", "imu_vel_medians", "vel_improve_pct"),
        ("imu_pos", "imu_pos_medians", "pos_improve_pct"),
        ("total_cost", "total_cost_medians", "cost_improve_pct"),
    ):
        medians = data[med_key]
        pct = data[pct_key]
        if medians is not None and pct is not None:
            print(f"    {name}: {medians[0]:.6g} -> {medians[1]:.6g}; {pct:.2f}%")


def load(path):
    with Path(path).open(newline="") as stream:
        raw_rows = list(csv.DictReader(stream))
    if not raw_rows:
        raise RuntimeError(f"empty CSV: {path}")
    # A ROS node normally truncates its log at startup.  If a stale process
    # survives a manual restart, however, two recordings can become appended.
    # Never mix such trajectories: retain the longest monotonic timestamp run.
    runs, current = [], []
    previous = None
    for row in raw_rows:
        timestamp = number(row, "timestamp")
        if previous is not None and timestamp is not None and timestamp < previous:
            if current:
                runs.append(current)
            current = []
        current.append(row)
        if timestamp is not None:
            previous = timestamp
    if current:
        runs.append(current)
    rows = max(runs, key=len)
    if len(runs) > 1:
        print(f"warning: {path} contains {len(runs)} timestamp segments; "
              f"using longest contiguous segment ({len(rows)}/{len(raw_rows)} rows)", file=sys.stderr)
    t0 = number(rows[0], "timestamp")
    for row in rows:
        timestamp = number(row, "timestamp")
        row["_relative_time"] = None if timestamp is None or t0 is None else timestamp - t0
    return rows


def sections(rows, final_return_start):
    return {
        "normal": [r for r in rows if not flag(r, "early_warning")],
        "early_warning": [r for r in rows if flag(r, "early_warning")],
        "persistent_degenerate": [r for r in rows if flag(r, "persistent_degenerate")],
        # Override this boundary if a different bag is used.
        "final_return_drift": [r for r in rows if r["_relative_time"] is not None and r["_relative_time"] >= final_return_start],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--final-return-start", type=float, default=450.0,
                        help="relative bag time (s) at which final return/drift begins (default: 450)")
    parser.add_argument("experiments", nargs="*", metavar="LABEL=CSV")
    args = parser.parse_args()
    specs = args.experiments
    if not specs:
        specs = ["A=FAST-LIO2_local_ba_A.csv", "B=FAST-LIO2_local_ba_B.csv", "C=FAST-LIO2_local_ba_C.csv"]
    for spec in specs:
        label, path = spec.split("=", 1) if "=" in spec else (Path(spec).stem, spec)
        rows = load(path)
        first = rows[0]
        print(f"\n=== {label}: {path} ===")
        print("weights:", first.get("imu_rot_weight", "missing"), first.get("imu_vel_weight", "missing"), first.get("imu_pos_weight", "missing"))
        for name, subset in sections(rows, args.final_return_start).items():
            print_summary(label, name, summary(subset))


if __name__ == "__main__":
    main()
