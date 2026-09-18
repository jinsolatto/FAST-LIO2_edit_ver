from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


OUTPUT_HTML = "metrics_and_pose_errors.html"
OUTPUT_PNG = "metrics_and_pose_errors.png"


def load_tum(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(x) for x in line.split()[:8]])
    arr = np.array(rows, dtype=float)
    return arr[:, 0], arr[:, 1:4], arr[:, 4:8]


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n == 0:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )


def associate(ts_a: np.ndarray, ts_b: np.ndarray, max_diff: float = 0.02) -> np.ndarray:
    matches: list[tuple[int, int]] = []
    j = 0
    used_b: set[int] = set()
    for i, ta in enumerate(ts_a):
        while j + 1 < len(ts_b) and ts_b[j + 1] <= ta:
            j += 1
        candidates = []
        for k in (j - 1, j, j + 1):
            if 0 <= k < len(ts_b) and k not in used_b:
                candidates.append((abs(ts_b[k] - ta), i, k))
        if not candidates:
            continue
        diff, i_sel, k_sel = min(candidates, key=lambda x: x[0])
        if diff <= max_diff:
            matches.append((i_sel, k_sel))
            used_b.add(k_sel)
    return np.array(matches, dtype=int)


def umeyama_alignment(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mu_x = x.mean(axis=0)
    mu_y = y.mean(axis=0)
    x_c = x - mu_x
    y_c = y - mu_y
    cov = (y_c.T @ x_c) / x.shape[0]
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1, -1] = -1
    r = u @ s @ vt
    t = mu_y - r @ mu_x
    return r, t


def make_poses(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    poses = []
    for p, q in zip(pos, quat):
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(q)
        T[:3, 3] = p
        poses.append(T)
    return np.array(poses)


def compute_ape_rpe(gt_path: Path, est_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gt_t, gt_p, gt_q = load_tum(gt_path)
    est_t, est_p, est_q = load_tum(est_path)
    matches = associate(gt_t, est_t)
    if len(matches) < 3:
        raise RuntimeError("Not enough timestamp matches to compute APE/RPE.")

    gt_pos = gt_p[matches[:, 0]]
    est_pos = est_p[matches[:, 1]]
    r_align, t_align = umeyama_alignment(est_pos, gt_pos)
    est_pos_aligned = (r_align @ est_pos.T).T + t_align
    ape_time = gt_t[matches[:, 0]] - gt_t[matches[0, 0]]
    ape = np.linalg.norm(gt_pos - est_pos_aligned, axis=1)

    A = np.eye(4)
    A[:3, :3] = r_align
    A[:3, 3] = t_align
    est_poses = np.einsum("ij,njk->nik", A, make_poses(est_p[matches[:, 1]], est_q[matches[:, 1]]))
    gt_poses = make_poses(gt_p[matches[:, 0]], gt_q[matches[:, 0]])

    rpe_time = gt_t[matches[1:, 0]] - gt_t[matches[1, 0]]
    rpe = []
    for i in range(len(matches) - 1):
        gt_rel = np.linalg.inv(gt_poses[i]) @ gt_poses[i + 1]
        est_rel = np.linalg.inv(est_poses[i]) @ est_poses[i + 1]
        err = np.linalg.inv(gt_rel) @ est_rel
        rpe.append(np.linalg.norm(err[:3, 3]))
    return ape_time, ape, rpe_time, np.array(rpe)


def choose_gt_file(gt_files: list[Path], traj: Path) -> Path:
    if len(gt_files) == 1:
        return gt_files[0]

    est_t, _, _ = load_tum(traj)
    scored: list[tuple[int, float, Path]] = []
    for gt_path in gt_files:
        gt_t, _, _ = load_tum(gt_path)
        matches = associate(gt_t, est_t)
        match_count = int(len(matches))
        start_diff = abs(float(gt_t[0]) - float(est_t[0])) if len(gt_t) and len(est_t) else float("inf")
        scored.append((match_count, -start_diff, gt_path))

    best_matches, _, best_path = max(scored, key=lambda item: (item[0], item[1], item[2].name))
    if best_matches < 3:
        names = ", ".join(path.name for path in gt_files)
        raise FileNotFoundError(
            f"Found multiple *_gt.txt files in {traj.parent}, but none match {traj.name} well enough: {names}"
        )
    print(f"[input] selected ground truth: {best_path.name} ({best_matches} timestamp matches)")
    return best_path


def score_traj_and_gt(traj: Path, gt_files: list[Path]) -> tuple[int, float, Path]:
    est_t, _, _ = load_tum(traj)
    scored: list[tuple[int, float, Path]] = []
    for gt_path in gt_files:
        gt_t, _, _ = load_tum(gt_path)
        matches = associate(gt_t, est_t)
        match_count = int(len(matches))
        start_diff = abs(float(gt_t[0]) - float(est_t[0])) if len(gt_t) and len(est_t) else float("inf")
        scored.append((match_count, -start_diff, gt_path))
    return max(scored, key=lambda item: (item[0], item[1], item[2].name))


def choose_traj_and_gt(traj_files: list[Path], gt_files: list[Path]) -> tuple[Path, Path]:
    candidates: list[tuple[int, float, Path, Path]] = []
    for traj in traj_files:
        match_count, neg_start_diff, gt_path = score_traj_and_gt(traj, gt_files)
        candidates.append((match_count, neg_start_diff, traj, gt_path))

    best_matches, _, best_traj, best_gt = max(candidates, key=lambda item: (item[0], item[1], item[2].name))
    if best_matches < 3:
        traj_names = ", ".join(path.name for path in traj_files)
        gt_names = ", ".join(path.name for path in gt_files)
        raise FileNotFoundError(
            f"Could not find a matching trajectory/GT pair in {best_traj.parent}. "
            f"Trajectory candidates: {traj_names}. GT candidates: {gt_names}."
        )
    print(f"[input] selected trajectory: {best_traj.name} ({best_matches} timestamp matches)")
    print(f"[input] selected ground truth: {best_gt.name}")
    return best_traj, best_gt


def find_inputs(cwd: Path) -> tuple[Path, Path, Path]:
    metrics = cwd / "scan_metrics.csv"
    traj_files = [path for path in (cwd / "trajectory.tum", cwd / "fastlio2_traj.txt") if path.exists()]
    gt_files = sorted(cwd.glob("*_gt.txt"))
    if not metrics.exists():
        raise FileNotFoundError(f"Missing {metrics}")
    if not traj_files:
        raise FileNotFoundError(f"Missing trajectory file in {cwd} (expected trajectory.tum or fastlio2_traj.txt)")
    if not gt_files:
        raise FileNotFoundError(f"Missing *_gt.txt in {cwd}")
    traj, gt = choose_traj_and_gt(traj_files, gt_files)
    return metrics, traj, gt


def resolve_target_dir(argv: list[str]) -> Path:
    if len(argv) > 2:
        raise SystemExit("Usage: python3 plot_metrics_and_pose_errors.py [target_dir]")
    if len(argv) == 2:
        target = Path(argv[1]).expanduser().resolve()
    else:
        target = Path.cwd().resolve()
    if not target.is_dir():
        raise NotADirectoryError(f"Target directory does not exist: {target}")
    return target


def to_serializable(values: np.ndarray, log_scale: bool = False) -> list[float | None]:
    if log_scale:
        out = []
        for v in values:
            fv = float(v)
            out.append(fv if math.isfinite(fv) and fv > 0 else None)
        return out
    return [float(v) if math.isfinite(float(v)) else None for v in values]


def make_chart(title: str, x: np.ndarray, y: np.ndarray, color: str, ylabel: str, log_scale: bool = False) -> dict:
    return {
        "title": title,
        "xlabel": "Time [s]",
        "ylabel": ylabel,
        "color": color,
        "logScale": log_scale,
        "x": [float(v) for v in x],
        "y": to_serializable(y, log_scale=log_scale),
    }


def build_html(title: str, stats: dict[str, float], charts: list[dict]) -> str:
    payload = {"title": title, "stats": stats, "charts": charts}
    data_json = json.dumps(payload, separators=(",", ":"))
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{title}</title>
  <style>
    :root {{
      --bg: #f7f7f5;
      --panel: #ffffff;
      --ink: #1f2937;
      --muted: #6b7280;
      --grid: #d1d5db;
      --accent: #111827;
    }}
    body {{
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, #eef2ff 0, transparent 28%),
        linear-gradient(180deg, #fafaf9 0%, #f3f4f6 100%);
    }}
    .wrap {{
      max-width: 1280px;
      margin: 0 auto;
      padding: 24px;
    }}
    h1 {{
      margin: 0 0 10px;
      font-size: 28px;
    }}
    .sub {{
      margin: 0 0 18px;
      color: var(--muted);
      font-size: 14px;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 18px;
    }}
    .card {{
      background: rgba(255,255,255,0.92);
      border: 1px solid #e5e7eb;
      border-radius: 14px;
      padding: 14px 16px;
      box-shadow: 0 8px 24px rgba(15, 23, 42, 0.06);
    }}
    .card .label {{
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 6px;
    }}
    .card .value {{
      font-size: 24px;
      font-weight: 700;
    }}
    .chart {{
      background: rgba(255,255,255,0.94);
      border: 1px solid #e5e7eb;
      border-radius: 16px;
      padding: 14px 16px 8px;
      margin-bottom: 14px;
      box-shadow: 0 8px 24px rgba(15, 23, 42, 0.05);
    }}
    .chart-title {{
      font-size: 18px;
      font-weight: 700;
      margin-bottom: 8px;
    }}
    .chart-head {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 8px;
    }}
    .chart-actions {{
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
    }}
    .zoom-hint {{
      color: var(--muted);
      font-size: 12px;
    }}
    .zoom-reset {{
      border: 1px solid #d1d5db;
      background: #fff;
      color: var(--ink);
      border-radius: 999px;
      padding: 6px 10px;
      font-size: 12px;
      cursor: pointer;
    }}
    .zoom-reset:disabled {{
      opacity: 0.45;
      cursor: default;
    }}
    .plot-box {{
      position: relative;
    }}
    svg {{
      width: 100%;
      height: auto;
      display: block;
    }}
    .tooltip {{
      position: absolute;
      pointer-events: none;
      background: rgba(17, 24, 39, 0.92);
      color: #fff;
      border-radius: 10px;
      padding: 8px 10px;
      font-size: 12px;
      line-height: 1.4;
      transform: translate(12px, -12px);
      display: none;
      white-space: nowrap;
      box-shadow: 0 8px 24px rgba(0, 0, 0, 0.22);
    }}
    @media (max-width: 900px) {{
      .stats {{
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <h1>{title}</h1>
    <p class="sub">Hover to inspect values. Drag on a chart to zoom into that time range.</p>
    <section class="stats">
      <div class="card"><div class="label">APE RMSE</div><div class="value">{stats["ape_rmse"]:.6f}</div></div>
      <div class="card"><div class="label">APE Mean</div><div class="value">{stats["ape_mean"]:.6f}</div></div>
      <div class="card"><div class="label">RPE RMSE</div><div class="value">{stats["rpe_rmse"]:.6f}</div></div>
      <div class="card"><div class="label">RPE Mean</div><div class="value">{stats["rpe_mean"]:.6f}</div></div>
    </section>
    <div id="charts"></div>
  </div>
  <script>
    const payload = {data_json};
    const width = 1180;
    const height = 220;
    const margin = {{ top: 16, right: 20, bottom: 38, left: 72 }};

    function fmt(v) {{
      if (v === null || Number.isNaN(v)) return "N/A";
      if (Math.abs(v) >= 1000 || (Math.abs(v) > 0 && Math.abs(v) < 0.001)) return v.toExponential(3);
      return v.toFixed(6);
    }}

    function buildTicks(min, max, count) {{
      if (max <= min) return [min];
      const step = (max - min) / (count - 1);
      return Array.from({{ length: count }}, (_, i) => min + step * i);
    }}

    function buildLogTicks(min, max) {{
      const ticks = [];
      const lo = Math.floor(Math.log10(min));
      const hi = Math.ceil(Math.log10(max));
      for (let p = lo; p <= hi; p += 1) ticks.push(10 ** p);
      return ticks;
    }}

    function pathFromData(xs, ys, xToPx, yToPx) {{
      let d = "";
      let started = false;
      for (let i = 0; i < xs.length; i += 1) {{
        if (ys[i] === null) {{
          started = false;
          continue;
        }}
        const cmd = started ? "L" : "M";
        d += `${{cmd}}${{xToPx(xs[i]).toFixed(2)}},${{yToPx(ys[i]).toFixed(2)}} `;
        started = true;
      }}
      return d.trim();
    }}

    function nearestIndex(xs, target) {{
      let lo = 0;
      let hi = xs.length - 1;
      while (lo < hi) {{
        const mid = Math.floor((lo + hi) / 2);
        if (xs[mid] < target) lo = mid + 1;
        else hi = mid;
      }}
      if (lo === 0) return 0;
      return Math.abs(xs[lo] - target) < Math.abs(xs[lo - 1] - target) ? lo : lo - 1;
    }}

    function renderChart(chart) {{
      const panel = document.createElement("section");
      panel.className = "chart";
      panel.innerHTML = `
        <div class="chart-head">
          <div class="chart-title">${{chart.title}}</div>
          <div class="chart-actions">
            <span class="zoom-hint">Drag to zoom</span>
            <button type="button" class="zoom-reset" disabled>Reset zoom</button>
          </div>
        </div>
        <div class="plot-box"></div>
      `;
      const box = panel.querySelector(".plot-box");
      const resetBtn = panel.querySelector(".zoom-reset");

      const xs = chart.x;
      const ys = chart.y;
      const fullXMin = Math.min(...xs);
      const fullXMax = Math.max(...xs);

      const plotW = width - margin.left - margin.right;
      const plotH = height - margin.top - margin.bottom;
      let domainMin = fullXMin;
      let domainMax = fullXMax;

      const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
      svg.setAttribute("viewBox", `0 0 ${{width}} ${{height}}`);
      const plotLayer = document.createElementNS(svg.namespaceURI, "g");
      svg.appendChild(plotLayer);

      const yLabel = document.createElementNS(svg.namespaceURI, "text");
      yLabel.setAttribute("x", 18);
      yLabel.setAttribute("y", height / 2);
      yLabel.setAttribute("transform", `rotate(-90 18 ${{height / 2}})`);
      yLabel.setAttribute("fill", "#374151");
      yLabel.setAttribute("font-size", "12");
      yLabel.textContent = chart.ylabel;
      svg.appendChild(yLabel);

      const xLabel = document.createElementNS(svg.namespaceURI, "text");
      xLabel.setAttribute("x", width / 2);
      xLabel.setAttribute("y", height - 2);
      xLabel.setAttribute("text-anchor", "middle");
      xLabel.setAttribute("fill", "#374151");
      xLabel.setAttribute("font-size", "12");
      xLabel.textContent = chart.xlabel;
      svg.appendChild(xLabel);

      const hoverLine = document.createElementNS(svg.namespaceURI, "line");
      hoverLine.setAttribute("y1", margin.top);
      hoverLine.setAttribute("y2", height - margin.bottom);
      hoverLine.setAttribute("stroke", "#111827");
      hoverLine.setAttribute("stroke-dasharray", "5 4");
      hoverLine.style.display = "none";
      svg.appendChild(hoverLine);

      const hoverPoint = document.createElementNS(svg.namespaceURI, "circle");
      hoverPoint.setAttribute("r", "4");
      hoverPoint.setAttribute("fill", chart.color);
      hoverPoint.setAttribute("stroke", "#111827");
      hoverPoint.setAttribute("stroke-width", "1");
      hoverPoint.style.display = "none";
      svg.appendChild(hoverPoint);

      const selection = document.createElementNS(svg.namespaceURI, "rect");
      selection.setAttribute("y", margin.top);
      selection.setAttribute("height", plotH);
      selection.setAttribute("fill", "rgba(37, 99, 235, 0.14)");
      selection.setAttribute("stroke", "#2563eb");
      selection.setAttribute("stroke-dasharray", "4 4");
      selection.style.display = "none";
      svg.appendChild(selection);

      const overlay = document.createElementNS(svg.namespaceURI, "rect");
      overlay.setAttribute("x", margin.left);
      overlay.setAttribute("y", margin.top);
      overlay.setAttribute("width", plotW);
      overlay.setAttribute("height", plotH);
      overlay.setAttribute("fill", "transparent");
      svg.appendChild(overlay);

      const tooltip = document.createElement("div");
      tooltip.className = "tooltip";
      box.appendChild(svg);
      box.appendChild(tooltip);

      function clampTime(value) {{
        return Math.max(fullXMin, Math.min(fullXMax, value));
      }}

      function getVisibleYRange() {{
        const visible = [];
        for (let i = 0; i < xs.length; i += 1) {{
          const y = ys[i];
          if (y === null) continue;
          if (xs[i] < domainMin || xs[i] > domainMax) continue;
          visible.push(y);
        }}
        const fallback = ys.filter(v => v !== null);
        const source = visible.length ? visible : fallback;
        let yMin = Math.min(...source);
        let yMax = Math.max(...source);
        if (chart.logScale) {{
          yMin *= 0.9;
          yMax *= 1.1;
        }} else {{
          const pad = (yMax - yMin || 1) * 0.08;
          yMin -= pad;
          yMax += pad;
        }}
        return [yMin, yMax];
      }}

      function xToPxFactory(xMin, xMax) {{
        return x => margin.left + ((x - xMin) / (xMax - xMin || 1)) * plotW;
      }}

      function yToPxFactory(yMin, yMax) {{
        if (chart.logScale) {{
          const lmin = Math.log10(yMin);
          const lmax = Math.log10(yMax);
          return y => {{
            const ly = Math.log10(y);
            return margin.top + (1 - (ly - lmin) / (lmax - lmin || 1)) * plotH;
          }};
        }}
        return y => margin.top + (1 - (y - yMin) / (yMax - yMin || 1)) * plotH;
      }}

      function redraw() {{
        plotLayer.replaceChildren();
        const [yMin, yMax] = getVisibleYRange();
        const xToPx = xToPxFactory(domainMin, domainMax);
        const yToPx = yToPxFactory(yMin, yMax);
        const yTicks = chart.logScale ? buildLogTicks(yMin, yMax) : buildTicks(yMin, yMax, 5);
        const xTicks = buildTicks(domainMin, domainMax, 6);
        const linePath = pathFromData(xs, ys, xToPx, yToPx);

        const bg = document.createElementNS(svg.namespaceURI, "rect");
        bg.setAttribute("x", margin.left);
        bg.setAttribute("y", margin.top);
        bg.setAttribute("width", plotW);
        bg.setAttribute("height", plotH);
        bg.setAttribute("fill", "#fff");
        plotLayer.appendChild(bg);

        yTicks.forEach(tick => {{
          const y = yToPx(tick);
          const grid = document.createElementNS(svg.namespaceURI, "line");
          grid.setAttribute("x1", margin.left);
          grid.setAttribute("x2", width - margin.right);
          grid.setAttribute("y1", y);
          grid.setAttribute("y2", y);
          grid.setAttribute("stroke", "#d1d5db");
          grid.setAttribute("stroke-dasharray", "4 4");
          plotLayer.appendChild(grid);

          const label = document.createElementNS(svg.namespaceURI, "text");
          label.setAttribute("x", margin.left - 10);
          label.setAttribute("y", y + 4);
          label.setAttribute("text-anchor", "end");
          label.setAttribute("fill", "#6b7280");
          label.setAttribute("font-size", "11");
          label.textContent = chart.logScale ? tick.toExponential(0) : tick.toFixed(3);
          plotLayer.appendChild(label);
        }});

        xTicks.forEach(tick => {{
          const x = xToPx(tick);
          const grid = document.createElementNS(svg.namespaceURI, "line");
          grid.setAttribute("x1", x);
          grid.setAttribute("x2", x);
          grid.setAttribute("y1", margin.top);
          grid.setAttribute("y2", height - margin.bottom);
          grid.setAttribute("stroke", "#e5e7eb");
          plotLayer.appendChild(grid);

          const label = document.createElementNS(svg.namespaceURI, "text");
          label.setAttribute("x", x);
          label.setAttribute("y", height - 12);
          label.setAttribute("text-anchor", "middle");
          label.setAttribute("fill", "#6b7280");
          label.setAttribute("font-size", "11");
          label.textContent = tick.toFixed(0);
          plotLayer.appendChild(label);
        }});

        const axes = document.createElementNS(svg.namespaceURI, "path");
        axes.setAttribute("d", `M${{margin.left}},${{margin.top}} V${{height - margin.bottom}} H${{width - margin.right}}`);
        axes.setAttribute("fill", "none");
        axes.setAttribute("stroke", "#374151");
        axes.setAttribute("stroke-width", "1.2");
        plotLayer.appendChild(axes);

        const line = document.createElementNS(svg.namespaceURI, "path");
        line.setAttribute("d", linePath);
        line.setAttribute("fill", "none");
        line.setAttribute("stroke", chart.color);
        line.setAttribute("stroke-width", "1.7");
        plotLayer.appendChild(line);
      }}

      function hideHover() {{
        hoverLine.style.display = "none";
        hoverPoint.style.display = "none";
        tooltip.style.display = "none";
      }}

      function pxToTime(clientX) {{
        const rect = svg.getBoundingClientRect();
        const scaleX = width / rect.width;
        const x = (clientX - rect.left) * scaleX;
        const clampedX = Math.max(margin.left, Math.min(width - margin.right, x));
        return domainMin + ((clampedX - margin.left) / plotW) * (domainMax - domainMin || 1);
      }}

      redraw();

      let dragStartX = null;
      let isDragging = false;

      overlay.addEventListener("mousemove", event => {{
        if (isDragging) {{
          const currentTime = clampTime(pxToTime(event.clientX));
          const leftTime = Math.min(dragStartX, currentTime);
          const rightTime = Math.max(dragStartX, currentTime);
          const xToPx = xToPxFactory(domainMin, domainMax);
          const leftPx = xToPx(leftTime);
          const rightPx = xToPx(rightTime);
          selection.setAttribute("x", leftPx);
          selection.setAttribute("width", Math.max(0, rightPx - leftPx));
          selection.style.display = "block";
          hideHover();
          return;
        }}

        const [yMin, yMax] = getVisibleYRange();
        const xToPx = xToPxFactory(domainMin, domainMax);
        const yToPx = yToPxFactory(yMin, yMax);
        const time = clampTime(pxToTime(event.clientX));
        const idx = nearestIndex(xs, time);
        if (ys[idx] === null) return;
        const px = xToPx(xs[idx]);
        const py = yToPx(ys[idx]);
        hoverLine.setAttribute("x1", px);
        hoverLine.setAttribute("x2", px);
        hoverLine.style.display = "block";
        hoverPoint.setAttribute("cx", px);
        hoverPoint.setAttribute("cy", py);
        hoverPoint.style.display = "block";
        tooltip.style.display = "block";
        tooltip.style.left = `${{event.offsetX + 6}}px`;
        tooltip.style.top = `${{event.offsetY + 6}}px`;
        tooltip.innerHTML = `t=${{xs[idx].toFixed(3)}} s<br>value=${{fmt(ys[idx])}}`;
      }});
      overlay.addEventListener("mouseleave", () => {{
        if (!isDragging) hideHover();
      }});

      overlay.addEventListener("mousedown", event => {{
        if (event.button !== 0) return;
        isDragging = true;
        dragStartX = clampTime(pxToTime(event.clientX));
        selection.setAttribute("x", xToPxFactory(domainMin, domainMax)(dragStartX));
        selection.setAttribute("width", 0);
        selection.style.display = "block";
        hideHover();
      }});

      overlay.addEventListener("mouseup", event => {{
        if (!isDragging) return;
        isDragging = false;
        selection.style.display = "none";
        const dragEndX = clampTime(pxToTime(event.clientX));
        const nextMin = Math.min(dragStartX, dragEndX);
        const nextMax = Math.max(dragStartX, dragEndX);
        if (nextMax - nextMin > (fullXMax - fullXMin) * 0.01) {{
          domainMin = nextMin;
          domainMax = nextMax;
          resetBtn.disabled = false;
          redraw();
        }}
      }});

      overlay.addEventListener("dblclick", () => {{
        domainMin = fullXMin;
        domainMax = fullXMax;
        resetBtn.disabled = true;
        selection.style.display = "none";
        hideHover();
        redraw();
      }});

      resetBtn.addEventListener("click", () => {{
        domainMin = fullXMin;
        domainMax = fullXMax;
        resetBtn.disabled = true;
        selection.style.display = "none";
        hideHover();
        redraw();
      }});

      return panel;
    }}

    const root = document.getElementById("charts");
    payload.charts.forEach(chart => root.appendChild(renderChart(chart)));
  </script>
</body>
</html>
"""


def save_png(cwd: Path, title: str, charts: list[dict]) -> None:
    fig, axes = plt.subplots(len(charts), 1, figsize=(13, 12), dpi=160, sharex=False)
    for ax, chart in zip(axes, charts):
        x = np.array(chart["x"], dtype=float)
        y = np.array([np.nan if v is None else float(v) for v in chart["y"]], dtype=float)
        ax.plot(x, y, color=chart["color"], linewidth=0.9)
        ax.set_title(chart["title"])
        ax.set_ylabel(chart["ylabel"])
        ax.set_xlabel(chart["xlabel"])
        if chart["logScale"]:
            ax.set_yscale("log")
        ax.tick_params(axis="x", labelbottom=True)
        ax.grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(cwd / OUTPUT_PNG, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    target_dir = resolve_target_dir(sys.argv)
    metrics_path, traj_path, gt_path = find_inputs(target_dir)

    metrics = pd.read_csv(metrics_path)
    metric_time = metrics["relative_time"].to_numpy(dtype=float)
    trans_cn = metrics["translation_condition_number"].to_numpy(dtype=float)
    trans_lambda_min = metrics["translation_lambda_min"].to_numpy(dtype=float)
    trans_cov_trace = metrics["translation_cov_trace"].to_numpy(dtype=float)

    ape_time, ape, rpe_time, rpe = compute_ape_rpe(gt_path, traj_path)
    stats = {
        "ape_rmse": float(np.sqrt(np.mean(ape**2))),
        "ape_mean": float(np.mean(ape)),
        "rpe_rmse": float(np.sqrt(np.mean(rpe**2))),
        "rpe_mean": float(np.mean(rpe)),
    }

    charts = [
        make_chart("Translation Condition Number", metric_time, trans_cn, "#9467bd", "Trans. CN", log_scale=True),
        make_chart("Translation Lambda Min", metric_time, trans_lambda_min, "#2ca02c", "Lambda Min"),
        make_chart("Translation Covariance Trace", metric_time, trans_cov_trace, "#8c564b", "Cov Trace", log_scale=True),
        make_chart("Absolute Pose Error (APE)", ape_time, ape, "#1f77b4", "APE [m]"),
        make_chart("Relative Pose Error (RPE)", rpe_time, rpe, "#d62728", "RPE [m]"),
    ]

    title = f"{target_dir.name}: Metrics and Pose Errors"
    html = build_html(title, stats, charts)
    out_path = target_dir / OUTPUT_HTML
    out_path.write_text(html, encoding="utf-8")
    save_png(target_dir, title, charts)
    print(f"Saved {out_path}")
    print(f"Saved {target_dir / OUTPUT_PNG}")


if __name__ == "__main__":
    main()
