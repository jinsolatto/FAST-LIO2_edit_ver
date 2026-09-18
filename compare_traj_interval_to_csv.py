#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path

import numpy as np


def load_tum(path: Path):
    times = []
    positions = []
    with path.open() as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            times.append(float(parts[0]))
            positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.asarray(times), np.asarray(positions)


def interp_positions(query_times: np.ndarray, source_times: np.ndarray, source_positions: np.ndarray):
    cols = []
    for dim in range(3):
        cols.append(np.interp(query_times, source_times, source_positions[:, dim]))
    return np.stack(cols, axis=1)


def path_length(positions: np.ndarray) -> float:
    if len(positions) < 2:
        return 0.0
    diffs = positions[1:] - positions[:-1]
    return float(np.linalg.norm(diffs, axis=1).sum())


def main():
    if len(sys.argv) != 6:
        print(
            "usage: compare_traj_interval_to_csv.py <traj_lidar.txt> <FAST-LIO2.tum> <IMU.tum> <start_sec> <end_sec>",
            file=sys.stderr,
        )
        sys.exit(1)

    traj_path = Path(sys.argv[1]).resolve()
    fastlio_path = Path(sys.argv[2]).resolve()
    imu_path = Path(sys.argv[3]).resolve()
    start_sec = float(sys.argv[4])
    end_sec = float(sys.argv[5])

    traj_times, traj_positions = load_tum(traj_path)
    fast_times, fast_positions = load_tum(fastlio_path)
    imu_times, imu_positions = load_tum(imu_path)

    window_start = traj_times[0] + start_sec
    window_end = traj_times[0] + end_sec
    mask = (traj_times >= window_start) & (traj_times <= window_end)
    ref_times = traj_times[mask]
    ref_positions = traj_positions[mask]
    ref_relative = ref_times - traj_times[0]

    if len(ref_times) == 0:
        raise RuntimeError("no traj_lidar samples in the requested interval")

    fast_interp = interp_positions(ref_times, fast_times, fast_positions)
    imu_interp = interp_positions(ref_times, imu_times, imu_positions)

    fast_err_vec = fast_interp - ref_positions
    imu_err_vec = imu_interp - ref_positions
    fast_err = np.linalg.norm(fast_err_vec, axis=1)
    imu_err = np.linalg.norm(imu_err_vec, axis=1)

    interval_label = f"{int(start_sec)}s_{int(end_sec)}s"
    out_dir = traj_path.parent
    samples_csv = out_dir / f"traj_similarity_{interval_label}_samples.csv"
    summary_csv = out_dir / f"traj_similarity_{interval_label}_summary.csv"

    with samples_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "relative_time_sec",
                "traj_x",
                "traj_y",
                "traj_z",
                "fastlio2_x",
                "fastlio2_y",
                "fastlio2_z",
                "imu_x",
                "imu_y",
                "imu_z",
                "fastlio2_error_m",
                "imu_error_m",
                "winner",
            ]
        )
        for i in range(len(ref_times)):
            writer.writerow(
                [
                    f"{ref_relative[i]:.9f}",
                    f"{ref_positions[i,0]:.9f}",
                    f"{ref_positions[i,1]:.9f}",
                    f"{ref_positions[i,2]:.9f}",
                    f"{fast_interp[i,0]:.9f}",
                    f"{fast_interp[i,1]:.9f}",
                    f"{fast_interp[i,2]:.9f}",
                    f"{imu_interp[i,0]:.9f}",
                    f"{imu_interp[i,1]:.9f}",
                    f"{imu_interp[i,2]:.9f}",
                    f"{fast_err[i]:.9f}",
                    f"{imu_err[i]:.9f}",
                    "FAST-LIO2" if fast_err[i] < imu_err[i] else ("IMU" if imu_err[i] < fast_err[i] else "TIE"),
                ]
            )

    rows = []
    for name, err, positions in [
        ("FAST-LIO2", fast_err, fast_interp),
        ("IMU", imu_err, imu_interp),
    ]:
        rows.append(
            {
                "candidate": name,
                "sample_count": len(err),
                "rmse_m": math.sqrt(float(np.mean(err ** 2))),
                "mean_error_m": float(np.mean(err)),
                "median_error_m": float(np.median(err)),
                "max_error_m": float(np.max(err)),
                "min_error_m": float(np.min(err)),
                "path_length_m": path_length(positions),
            }
        )

    winner = min(rows, key=lambda row: row["rmse_m"])["candidate"]
    with summary_csv.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "interval_start_sec",
                "interval_end_sec",
                "reference",
                "candidate",
                "sample_count",
                "rmse_m",
                "mean_error_m",
                "median_error_m",
                "max_error_m",
                "min_error_m",
                "path_length_m",
                "winner_by_rmse",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    f"{start_sec:.3f}",
                    f"{end_sec:.3f}",
                    "traj_lidar",
                    row["candidate"],
                    row["sample_count"],
                    f"{row['rmse_m']:.9f}",
                    f"{row['mean_error_m']:.9f}",
                    f"{row['median_error_m']:.9f}",
                    f"{row['max_error_m']:.9f}",
                    f"{row['min_error_m']:.9f}",
                    f"{row['path_length_m']:.9f}",
                    winner,
                ]
            )

    print(summary_csv)
    print(samples_csv)


if __name__ == "__main__":
    main()
