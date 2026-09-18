import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_PATH = Path("/home/jschoi/edit_fastlio2/corridor02_original/scan_metrics.csv")
EST_TRAJ_PATH = Path("/home/jschoi/edit_fastlio2/corridor02_original/trajectory.tum")
GT_TRAJ_PATH = Path("/home/jschoi/edit_fastlio2/corridor02_original/corridor02_gt.txt")
OUT_DIR = Path("/home/jschoi/edit_fastlio2/results/hidden_degeneracy_corridor02")
A_WINDOW = 10
TIME_TOL = 3.0
EPS = 1e-9
B_INIT_SECONDS_GRID = (20.0,)
B_ENTER_THRESHOLD_QUANTILE_GRID = (0.99,)
B_EXIT_THRESHOLD_QUANTILE_GRID = (0.95,)
B_LAMBDA_GRID = (0.3,)
BASELINE_UPDATE_ALPHA = 0.02
THRESHOLD_BUFFER_MULTIPLIER = 3
ENTER_COUNT_GRID = (5,)
ENTER_WINDOW_GRID = (5,)
EXIT_COUNT_GRID = (40,)
EXIT_WINDOW_GRID = (80,)
GT_INIT_SECONDS = 20.0
GT_ERROR_QUANTILE = 0.99
GT_ERROR_FLOOR_M = 0.03


@dataclass
class Metrics:
    precision: float
    recall: float
    f1: float
    tp: int
    fp: int
    fn: int


def load_data(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    required = [
        "relative_time",
        "translation_condition_number",
        "translation_lambda_min",
        "translation_cov_trace",
        "effective_feature_count",
    ]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")
    return df


def load_tum_positions(path: Path) -> tuple[np.ndarray, np.ndarray]:
    arr = np.loadtxt(path)
    if arr.ndim != 2 or arr.shape[1] < 4:
        raise ValueError(f"Unexpected TUM trajectory format: {path}")
    return arr[:, 0].astype(float), arr[:, 1:4].astype(float)


def interpolate_positions(query_t: np.ndarray, src_t: np.ndarray, src_p: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query_t, src_t, src_p[:, i]) for i in range(3)])


def build_ground_truth(df: pd.DataFrame) -> tuple[pd.Series, pd.DataFrame]:
    est_t, est_p = load_tum_positions(EST_TRAJ_PATH)
    gt_t, gt_p = load_tum_positions(GT_TRAJ_PATH)
    scan_t = df["scan_time"].to_numpy(float)

    overlap_mask = (scan_t >= max(est_t.min(), gt_t.min())) & (scan_t <= min(est_t.max(), gt_t.max()))
    if overlap_mask.sum() < 5:
        raise ValueError("Insufficient time overlap between estimated and GT trajectories")

    overlap_t = scan_t[overlap_mask]
    est_interp = interpolate_positions(overlap_t, est_t, est_p)
    gt_interp = interpolate_positions(overlap_t, gt_t, gt_p)

    init_mask_overlap = overlap_t <= (overlap_t[0] + GT_INIT_SECONDS)
    if init_mask_overlap.sum() < 5:
        raise ValueError("Insufficient initial overlap for GT initialization")

    align_offset = np.median(gt_interp[init_mask_overlap] - est_interp[init_mask_overlap], axis=0)
    est_aligned = est_interp + align_offset
    pos_error = np.linalg.norm(est_aligned - gt_interp, axis=1)
    gt_threshold = max(
        GT_ERROR_FLOOR_M,
        float(np.quantile(pos_error[init_mask_overlap], GT_ERROR_QUANTILE)),
    )
    gt_flags_overlap = pos_error > gt_threshold

    gt_flags = np.zeros(len(df), dtype=bool)
    gt_error = np.full(len(df), np.nan)
    gt_flags[overlap_mask] = gt_flags_overlap
    gt_error[overlap_mask] = pos_error

    gt_meta = pd.DataFrame(
        {
            "gt_position_error_m": gt_error,
            "gt_error_threshold_m": np.full(len(df), gt_threshold),
            "gt_has_overlap": overlap_mask,
        }
    )
    return pd.Series(gt_flags, index=df.index), gt_meta


def evaluate_scan_validation(
    times: np.ndarray,
    pred_flags: np.ndarray,
    gt_flags: np.ndarray,
    tol_s: float = TIME_TOL,
) -> Metrics:
    pred_times = times[pred_flags]
    gt_times = times[gt_flags]

    # Reproduce the user's existing Candidate A benchmark: a prediction is a TP
    # if any GT lies within tolerance. Recall is measured with the same TP count.
    tp = int(sum(np.any(np.abs(gt_times - pt) <= tol_s) for pt in pred_times))
    fp = int(len(pred_times) - tp)
    fn = int(len(gt_times) - tp)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if (precision + recall)
        else 0.0
    )
    return Metrics(precision, recall, f1, tp, fp, fn)


def evaluate_event_1to1(
    times: np.ndarray,
    pred_flags: np.ndarray,
    gt_flags: np.ndarray,
    tol_s: float = TIME_TOL,
) -> Metrics:
    pred_times = times[pred_flags]
    gt_times = times[gt_flags]
    i = j = tp = 0
    while i < len(pred_times) and j < len(gt_times):
        dt = pred_times[i] - gt_times[j]
        if abs(dt) <= tol_s:
            tp += 1
            i += 1
            j += 1
        elif pred_times[i] < gt_times[j] - tol_s:
            i += 1
        else:
            j += 1
    fp = int(len(pred_times) - tp)
    fn = int(len(gt_times) - tp)
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if (precision + recall)
        else 0.0
    )
    return Metrics(precision, recall, f1, tp, fp, fn)


def candidate_a(df: pd.DataFrame) -> pd.DataFrame:
    cov_avg_recent = (
        df["translation_cov_trace"]
        .shift(1)
        .rolling(A_WINDOW, min_periods=A_WINDOW)
        .mean()
    )
    cn_normal = df["translation_condition_number"] < 8.0
    cov_spike_ratio = df["translation_cov_trace"] / cov_avg_recent
    cov_spike = df["translation_cov_trace"] > 1.5 * cov_avg_recent
    flag = (cn_normal & cov_spike).fillna(False)
    return pd.DataFrame(
        {
            "cov_avg_recent": cov_avg_recent,
            "cov_spike_ratio": cov_spike_ratio,
            "flag_a": flag,
        }
    )


def apply_state_machine(
    score_ratio: np.ndarray,
    hidden_dir: np.ndarray,
    enter_score: float,
    enter_count: int,
    enter_window: int,
    exit_score: float,
    exit_count: int,
    exit_window: int,
) -> tuple[np.ndarray, np.ndarray]:
    state = False
    final_flags = np.zeros(len(score_ratio), dtype=bool)
    raw_flags = np.zeros(len(score_ratio), dtype=bool)
    for k in range(len(score_ratio)):
        raw_flags[k] = bool(score_ratio[k] >= enter_score and hidden_dir[k])
        if not state:
            start = max(0, k - enter_window + 1)
            evidence = ((score_ratio[start : k + 1] >= enter_score) & hidden_dir[start : k + 1]).sum()
            if evidence >= enter_count:
                state = True
        else:
            start = max(0, k - exit_window + 1)
            calm = ((score_ratio[start : k + 1] < exit_score) | (~hidden_dir[start : k + 1])).sum()
            if calm >= exit_count:
                state = False
        final_flags[k] = state
    return raw_flags, final_flags


def run_mewma(
    df: pd.DataFrame,
    lam: float,
    init_seconds: float,
    enter_threshold_quantile: float,
    exit_threshold_quantile: float,
    enter_count: int,
    enter_window: int,
    exit_count: int,
    exit_window: int,
    require_both: bool = True,
    baseline_alpha: float = BASELINE_UPDATE_ALPHA,
) -> pd.DataFrame:
    # Online detector: uses only scan metrics. GT is not referenced here.
    times = df["relative_time"].to_numpy(float)
    z = df[["translation_condition_number", "translation_cov_trace"]].to_numpy(float)
    n = len(df)
    init_mask = times <= (times[0] + init_seconds)
    init_count = int(init_mask.sum())
    if init_count < 5:
        raise ValueError(
            f"Initial normal window is too short: {init_count} scans in first {init_seconds}s"
        )
    assert enter_count <= enter_window, "enter_count must be <= enter_window"
    assert exit_count <= exit_window, "exit_count must be <= exit_window"

    init_z = z[init_mask]
    mu0 = init_z.mean(axis=0)
    sigma0 = np.cov(init_z.T, bias=False)
    sigma0 = np.atleast_2d(sigma0) + EPS * np.eye(2)

    threshold_buffer_size = max(init_count, int(init_count * THRESHOLD_BUFFER_MULTIPLIER))

    mu = np.zeros((n, 2))
    sigma_diag = np.zeros((n, 2))
    signed_residual = np.zeros((n, 2))
    directional_residual = np.zeros((n, 2))
    Z = np.zeros((n, 2))
    T2 = np.full(n, np.nan)
    init_T2 = np.full(n, np.nan)
    h4_series = np.full(n, np.nan)
    flags_raw = np.zeros(n, dtype=bool)
    flags = np.zeros(n, dtype=bool)
    hidden = np.zeros(n, dtype=bool)
    accepted_normal = np.zeros(n, dtype=bool)
    state_series = np.zeros(n, dtype=bool)
    Z_prev = np.zeros(2)
    mu_curr = mu0.copy()
    sigma_curr = sigma0.copy()
    normal_t2_buffer: list[float] = []
    state = False
    raw_history: list[bool] = []
    calm_history: list[bool] = []
    high_threshold_series = np.full(n, np.nan)
    low_threshold_series = np.full(n, np.nan)

    for k in range(n):
        mu[k] = mu_curr
        sigma_diag[k] = np.diag(sigma_curr)
        centered = z[k] - mu_curr
        signed_residual[k] = centered

        # Emphasize the hidden-degeneracy direction:
        # lower-than-normal CN and higher-than-normal covariance.
        direction = np.array([mu_curr[0] - z[k, 0], z[k, 1] - mu_curr[1]])
        if require_both:
            if direction[0] <= 0.0 or direction[1] <= 0.0:
                direction[:] = 0.0
            else:
                direction = np.maximum(direction, 0.0)
        else:
            direction = np.maximum(direction, 0.0)
        directional_residual[k] = direction

        Z_curr = lam * direction + (1.0 - lam) * Z_prev
        Z[k] = Z_curr

        sigma_z = (lam / (2.0 - lam)) * sigma_curr
        sigma_z = sigma_z + EPS * np.eye(2)
        T2[k] = float(Z_curr.T @ np.linalg.pinv(sigma_z) @ Z_curr)

        if init_mask[k]:
            init_T2[k] = T2[k]
            accepted_normal[k] = True
            normal_t2_buffer.append(float(T2[k]))
        elif normal_t2_buffer:
            high_threshold = float(np.quantile(normal_t2_buffer, enter_threshold_quantile))
            low_threshold = float(np.quantile(normal_t2_buffer, exit_threshold_quantile))
            high_threshold_series[k] = high_threshold
            low_threshold_series[k] = low_threshold

            hidden_dir = (z[k, 0] < mu_curr[0]) and (z[k, 1] > mu_curr[1])
            raw_hit = T2[k] > high_threshold
            if require_both:
                raw_hit = raw_hit and hidden_dir
            flags_raw[k] = raw_hit

            if not state:
                raw_history.append(raw_hit)
                if len(raw_history) > enter_window:
                    raw_history.pop(0)
                if sum(raw_history) >= enter_count:
                    state = True
            else:
                calm_hit = T2[k] < low_threshold
                calm_history.append(calm_hit)
                if len(calm_history) > exit_window:
                    calm_history.pop(0)
                if sum(calm_history) >= exit_count:
                    state = False
                    raw_history.clear()
                    calm_history.clear()

            flags[k] = state
            state_series[k] = state
            hidden[k] = state and hidden_dir

            # Freeze adaptation while the detector is in a degenerate state.
            if not state:
                accepted_normal[k] = True
                normal_t2_buffer.append(float(T2[k]))
                if len(normal_t2_buffer) > threshold_buffer_size:
                    normal_t2_buffer.pop(0)

                delta = z[k] - mu_curr
                mu_curr = (1.0 - baseline_alpha) * mu_curr + baseline_alpha * z[k]
                sigma_curr = (
                    (1.0 - baseline_alpha) * sigma_curr
                    + baseline_alpha * np.outer(delta, delta)
                )
                sigma_curr = sigma_curr + EPS * np.eye(2)
        Z_prev = Z_curr
        if normal_t2_buffer:
            h4_series[k] = float(np.quantile(normal_t2_buffer, enter_threshold_quantile))

    final_h4 = float(h4_series[np.isfinite(h4_series)][-1])
    score_ratio = np.divide(
        T2,
        high_threshold_series,
        out=np.zeros_like(T2),
        where=np.isfinite(high_threshold_series) & (high_threshold_series > 0),
    )
    flags[init_mask] = False
    state_series[init_mask] = False
    hidden_direction = (z[:, 0] < mu[:, 0]) & (z[:, 1] > mu[:, 1])
    hidden = flags & hidden_direction
    adaptive_baseline_shift = float(np.linalg.norm(mu_curr - mu0))
    adaptive_threshold_shift = float(final_h4 - h4_series[init_count - 1])

    return pd.DataFrame(
        {
            "mu_cn": mu[:, 0],
            "mu_cov": mu[:, 1],
            "sigma_cn": sigma_diag[:, 0],
            "sigma_cov": sigma_diag[:, 1],
            "cn_signed_residual": signed_residual[:, 0],
            "cov_signed_residual": signed_residual[:, 1],
            "hidden_cn_drop": directional_residual[:, 0],
            "hidden_cov_rise": directional_residual[:, 1],
            "mewma_Z_cn": Z[:, 0],
            "mewma_Z_cov": Z[:, 1],
            "T2": T2,
            "score_ratio": score_ratio,
            "init_T2": init_T2,
            "accepted_normal": accepted_normal,
            "flag_b_raw": flags_raw,
            "flag_b": flags,
            "state_b": state_series,
            "hidden_b": hidden,
            "h4": h4_series,
            "t_high": high_threshold_series,
            "t_low": low_threshold_series,
            "h4_final": final_h4,
            "lambda": lam,
            "enter_threshold_quantile": enter_threshold_quantile,
            "exit_threshold_quantile": exit_threshold_quantile,
            "init_seconds": init_seconds,
            "init_count": init_count,
            "init_is_normal": init_mask,
            "enter_count": enter_count,
            "enter_window": enter_window,
            "exit_count": exit_count,
            "exit_window": exit_window,
            "adaptive_baseline_shift": adaptive_baseline_shift,
            "adaptive_threshold_shift": adaptive_threshold_shift,
        }
    )


def build_gt_spans(times: np.ndarray, gt_flags: np.ndarray) -> list[tuple[float, float]]:
    spans = []
    start = None
    for t, flag in zip(times, gt_flags):
        if flag and start is None:
            start = t
        elif not flag and start is not None:
            spans.append((start, prev_t))
            start = None
        prev_t = t
    if start is not None:
        spans.append((start, prev_t))
    return spans


def plot_candidate_a(
    times: np.ndarray,
    a_df: pd.DataFrame,
    gt_flags: np.ndarray,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(14, 5))
    ratio = a_df["cov_spike_ratio"].to_numpy()
    ax.plot(times, ratio, color="tab:blue", linewidth=1.2, label="cov_spike_ratio")
    ax.axhline(1.5, color="tab:orange", linestyle="--", linewidth=1.0, label="threshold=1.5")
    for start, end in build_gt_spans(times, gt_flags):
        ax.axvspan(start, end, color="tab:red", alpha=0.15)
    flagged_t = times[a_df["flag_a"].to_numpy()]
    flagged_y = ratio[a_df["flag_a"].to_numpy()]
    ax.scatter(flagged_t, flagged_y, s=10, color="black", label="Candidate A flag")
    ax.set_title("Candidate A: Covariance Spike Ratio vs Ground Truth")
    ax.set_xlabel("Relative Time [s]")
    ax.set_ylabel("cov_trace / recent_mean")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def plot_candidate_b(
    times: np.ndarray,
    b_df: pd.DataFrame,
    gt_flags: np.ndarray,
    out_path: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(14, 5))
    t2 = b_df["T2"].to_numpy()
    h4 = b_df["h4"].to_numpy()
    init_mask = b_df["init_is_normal"].to_numpy(dtype=bool)
    ax.plot(times, t2, color="tab:green", linewidth=1.2, label="T2_k")
    ax.plot(times, h4, color="tab:orange", linestyle="--", linewidth=1.0, label="adaptive threshold")
    if init_mask.any():
        ax.axvspan(
            times[init_mask][0],
            times[init_mask][-1],
            color="tab:blue",
            alpha=0.08,
            label="Initial normal window",
        )
    for start, end in build_gt_spans(times, gt_flags):
        ax.axvspan(start, end, color="tab:red", alpha=0.15)
    flag_mask = b_df["flag_b"].to_numpy()
    raw_mask = b_df["flag_b_raw"].to_numpy()
    hidden_mask = b_df["hidden_b"].to_numpy()
    ax.scatter(times[raw_mask], t2[raw_mask], s=8, color="tab:gray", alpha=0.35, label="Raw threshold hit")
    ax.scatter(times[flag_mask], t2[flag_mask], s=10, color="black", label="Candidate B flag")
    ax.scatter(
        times[hidden_mask],
        t2[hidden_mask],
        s=16,
        color="tab:purple",
        label="Hidden label",
    )
    ax.set_title("Candidate B: Initial-Baseline Hidden-Degeneracy MEWMA")
    ax.set_xlabel("Relative Time [s]")
    ax.set_ylabel("T2 statistic")
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def summarize_event_timing(
    times: np.ndarray,
    pred_flags: np.ndarray,
    gt_flags: np.ndarray,
) -> dict:
    pred_times = times[pred_flags]
    gt_times = times[gt_flags]
    if len(pred_times) == 0 or len(gt_times) == 0:
        return {"median_lead_s": None, "mean_lead_s": None}
    lead_times = []
    for gt_time in gt_times:
        earlier_preds = pred_times[pred_times <= gt_time]
        if len(earlier_preds):
            lead_times.append(float(gt_time - earlier_preds[-1]))
    if not lead_times:
        return {"median_lead_s": None, "mean_lead_s": None}
    return {
        "median_lead_s": float(np.median(lead_times)),
        "mean_lead_s": float(np.mean(lead_times)),
    }


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    df = load_data(DATA_PATH)
    gt, gt_meta = build_ground_truth(df)
    a_df = candidate_a(df)

    times = df["relative_time"].to_numpy()
    gt_flags = gt.to_numpy()
    a_flags = a_df["flag_a"].to_numpy()

    metrics_a_validation = evaluate_scan_validation(times, a_flags, gt_flags)
    metrics_a_event = evaluate_event_1to1(times, a_flags, gt_flags)

    best = None
    best_run = None
    grid_rows = []
    for init_seconds in B_INIT_SECONDS_GRID:
        for lam in B_LAMBDA_GRID:
            for enter_threshold_quantile in B_ENTER_THRESHOLD_QUANTILE_GRID:
                for exit_threshold_quantile in B_EXIT_THRESHOLD_QUANTILE_GRID:
                    for enter_count in ENTER_COUNT_GRID:
                        for enter_window in ENTER_WINDOW_GRID:
                            for exit_count in EXIT_COUNT_GRID:
                                for exit_window in EXIT_WINDOW_GRID:
                                        run_df = run_mewma(
                                            df,
                                            lam=float(lam),
                                            init_seconds=float(init_seconds),
                                            enter_threshold_quantile=float(enter_threshold_quantile),
                                            exit_threshold_quantile=float(exit_threshold_quantile),
                                            enter_count=int(enter_count),
                                            enter_window=int(enter_window),
                                            exit_count=int(exit_count),
                                            exit_window=int(exit_window),
                                        )
                                        metrics_validation = evaluate_scan_validation(
                                            times, run_df["flag_b"].to_numpy(), gt_flags
                                        )
                                        metrics_event = evaluate_event_1to1(
                                            times, run_df["flag_b"].to_numpy(), gt_flags
                                        )
                                        row = {
                                            "init_seconds": float(init_seconds),
                                            "lambda": float(lam),
                                            "enter_threshold_quantile": float(enter_threshold_quantile),
                                            "exit_threshold_quantile": float(exit_threshold_quantile),
                                            "enter_count": int(enter_count),
                                            "enter_window": int(enter_window),
                                            "exit_count": int(exit_count),
                                            "exit_window": int(exit_window),
                                            "h4_final": float(run_df["h4_final"].iloc[0]),
                                            "init_count": int(run_df["init_count"].iloc[0]),
                                            "adaptive_baseline_shift": float(run_df["adaptive_baseline_shift"].iloc[0]),
                                            "adaptive_threshold_shift": float(run_df["adaptive_threshold_shift"].iloc[0]),
                                            "validation_precision": metrics_validation.precision,
                                            "validation_recall": metrics_validation.recall,
                                            "validation_f1": metrics_validation.f1,
                                            "event_precision": metrics_event.precision,
                                            "event_recall": metrics_event.recall,
                                            "event_f1": metrics_event.f1,
                                            "event_tp": metrics_event.tp,
                                            "event_fp": metrics_event.fp,
                                            "event_fn": metrics_event.fn,
                                            "hidden_count": int(run_df["hidden_b"].sum()),
                                            "raw_flag_count": int(run_df["flag_b_raw"].sum()),
                                            "flag_count": int(run_df["flag_b"].sum()),
                                        }
                                        grid_rows.append(row)
                                        if best is None or metrics_event.f1 > best["event_f1"]:
                                            best = row
                                            best_run = run_df

    comparison = pd.DataFrame(
        [
            {
                "candidate": "A_fixed_threshold",
                "validation_precision": metrics_a_validation.precision,
                "validation_recall": metrics_a_validation.recall,
                "validation_f1": metrics_a_validation.f1,
                "event_precision": metrics_a_event.precision,
                "event_recall": metrics_a_event.recall,
                "event_f1": metrics_a_event.f1,
                "event_tp": metrics_a_event.tp,
                "event_fp": metrics_a_event.fp,
                "event_fn": metrics_a_event.fn,
                "flag_count": int(a_flags.sum()),
                "notes": "cn<8 and cov > 1.5 * mean(prev 10 cov)",
            },
            {
                "candidate": "B_mewma_best",
                "validation_precision": best["validation_precision"],
                "validation_recall": best["validation_recall"],
                "validation_f1": best["validation_f1"],
                "event_precision": best["event_precision"],
                "event_recall": best["event_recall"],
                "event_f1": best["event_f1"],
                "event_tp": best["event_tp"],
                "event_fp": best["event_fp"],
                "event_fn": best["event_fn"],
                "flag_count": best["flag_count"],
                "notes": (
                    "initial-normal-baseline MEWMA, "
                    f"init_seconds={best['init_seconds']}, "
                    f"lambda={best['lambda']}, "
                    f"t_high_q={best['enter_threshold_quantile']}, "
                    f"t_low_q={best['exit_threshold_quantile']}, "
                    f"enter_count={best['enter_count']}, "
                    f"enter_window={best['enter_window']}, "
                    f"exit_count={best['exit_count']}, "
                    f"exit_window={best['exit_window']}, "
                    f"h4_final={best['h4_final']:.3f}"
                ),
            },
        ]
    )

    comparison.to_csv(OUT_DIR / "comparison_metrics.csv", index=False)
    pd.DataFrame(grid_rows).sort_values(
        ["event_f1", "event_precision", "event_recall"], ascending=False
    ).to_csv(
        OUT_DIR / "candidate_b_grid_search.csv",
        index=False,
    )

    details = pd.concat(
        [
            df[[
                "relative_time",
                "scan_time",
                "translation_condition_number",
                "translation_lambda_min",
                "translation_cov_trace",
                "effective_feature_count",
            ]],
            gt.rename("gt_hidden_deg"),
            gt_meta,
            a_df,
            best_run,
        ],
        axis=1,
    )
    details.to_csv(OUT_DIR / "per_scan_analysis.csv", index=False)

    plot_candidate_a(times, a_df, gt_flags, OUT_DIR / "candidate_a_cov_spike.png")
    plot_candidate_b(times, best_run, gt_flags, OUT_DIR / "candidate_b_t2.png")

    timing_summary = {
        "candidate_a": summarize_event_timing(times, a_flags, gt_flags),
        "candidate_b": summarize_event_timing(times, best_run["flag_b"].to_numpy(), gt_flags),
    }

    summary = {
        "data_path": str(DATA_PATH),
        "row_count": int(len(df)),
        "gt_count": int(gt_flags.sum()),
        "pipeline_validation_note": (
            "validation_f1 reproduces the user's Candidate A benchmark style "
            "by counting a predicted scan as TP when any GT scan exists within +/-3 s."
        ),
        "comparison_note": (
            "event_f1 is the fair detector-to-detector comparison metric used for recommendation. "
            "It uses 1:1 time-tolerant matching to avoid double-counting dense flag bursts."
        ),
        "candidate_b_design_note": (
            "Candidate B initializes its baseline from the initial normal window, "
            "tracks the hidden-degeneracy direction (CN drop, covariance rise), and "
            "freezes baseline and threshold adaptation while in the degenerate state. "
            "A two-threshold persistence state machine turns sparse raw alarms into a "
            "stable online degeneracy state."
        ),
        "detector_uses_gt": False,
        "gt_note": (
            "Ground truth is derived from the actual GT trajectory corridor02_gt.txt by "
            "time-aligning trajectory.tum to GT, aligning the initial 20 s with a constant "
            "translation offset, and flagging scans whose position error exceeds the "
            "initial-window 99th-percentile error threshold."
        ),
        "candidate_a": comparison.iloc[0].to_dict(),
        "candidate_b_best": comparison.iloc[1].to_dict(),
        "candidate_b_improvement_validation_f1": float(
            best["validation_f1"] - metrics_a_validation.f1
        ),
        "candidate_b_improvement_event_f1": float(
            best["event_f1"] - metrics_a_event.f1
        ),
        "timing_summary": timing_summary,
        "implementation_note": (
            "Candidate B starts from a guaranteed-normal initial window, runs MEWMA only "
            "on the hidden-degeneracy direction (lower CN, higher covariance), adapts its "
            "baseline and empirical thresholds only while state=False, and freezes both "
            "while state=True. Final alarms are produced by a hysteretic state machine "
            "with separate T_high and T_low thresholds."
        ),
        "candidate_b_adaptive_baseline_shift": float(best_run["adaptive_baseline_shift"].iloc[0]),
        "candidate_b_adaptive_threshold_shift": float(best_run["adaptive_threshold_shift"].iloc[0]),
        "gt_threshold_m": float(gt_meta["gt_error_threshold_m"].iloc[0]),
        "gt_overlap_count": int(gt_meta["gt_has_overlap"].sum()),
    }
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2))

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
