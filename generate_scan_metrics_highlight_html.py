#!/usr/bin/env python3
"""Generate an HTML report for scan metrics with a highlighted time region."""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path


def parse_float(value: str) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def load_csv(csv_path: Path) -> tuple[list[float], list[str], dict[str, list[float | None]]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        if "relative_time" not in fieldnames:
            raise ValueError("CSV must contain a 'relative_time' column.")

        metric_columns = [
            name
            for name in fieldnames
            if name != "relative_time" and name != "front_rejected" and "med" not in name
        ]

        times: list[float] = []
        series: dict[str, list[float | None]] = {name: [] for name in metric_columns}

        for row in reader:
            time_value = parse_float(row.get("relative_time", ""))
            if time_value is None:
                continue
            times.append(time_value)
            for name in metric_columns:
                series[name].append(parse_float(row.get(name, "")))

    return times, metric_columns, series


def build_payload(csv_path: Path, times: list[float], metric_columns: list[str], series: dict[str, list[float | None]], highlight_start: float) -> dict:
    charts = []
    for name in metric_columns:
        values = series[name]
        finite_values = [value for value in values if value is not None]
        charts.append(
            {
                "title": name,
                "x": times,
                "y": values,
                "min": min(finite_values) if finite_values else 0.0,
                "max": max(finite_values) if finite_values else 1.0,
            }
        )

    return {
        "title": csv_path.name,
        "highlightStart": highlight_start,
        "pointCount": len(times),
        "timeMin": min(times) if times else 0.0,
        "timeMax": max(times) if times else 1.0,
        "chartCount": len(charts),
        "charts": charts,
    }


def build_html(payload: dict) -> str:
    data_json = json.dumps(payload, separators=(",", ":"))
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{payload["title"]}</title>
  <style>
    :root {{
      --bg: #f4f1ea;
      --panel: rgba(255, 252, 247, 0.94);
      --ink: #1d2733;
      --muted: #5f6b76;
      --grid: #d7d1c7;
      --line: #1f5c4a;
      --highlight: rgba(235, 87, 87, 0.18);
      --border: #e4ddd2;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(245, 198, 125, 0.20), transparent 24%),
        radial-gradient(circle at top right, rgba(133, 181, 160, 0.18), transparent 20%),
        linear-gradient(180deg, #fbf8f2 0%, #f1ede5 100%);
    }}
    .wrap {{
      max-width: 1480px;
      margin: 0 auto;
      padding: 24px;
    }}
    .hero {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 20px;
      padding: 22px 24px;
      box-shadow: 0 16px 40px rgba(49, 46, 38, 0.08);
      margin-bottom: 18px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 30px;
      line-height: 1.15;
    }}
    .sub {{
      margin: 0;
      color: var(--muted);
      font-size: 14px;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
      margin: 18px 0 22px;
    }}
    .stat {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 14px 16px;
      box-shadow: 0 10px 26px rgba(49, 46, 38, 0.05);
    }}
    .stat .label {{
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
      margin-bottom: 6px;
    }}
    .stat .value {{
      font-size: 23px;
      font-weight: 700;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }}
    .chart {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 14px 16px 12px;
      box-shadow: 0 10px 26px rgba(49, 46, 38, 0.05);
    }}
    .chart h2 {{
      margin: 0 0 8px;
      font-size: 17px;
    }}
    .chart .meta {{
      margin: 0 0 8px;
      font-size: 12px;
      color: var(--muted);
    }}
    svg {{
      width: 100%;
      height: auto;
      display: block;
      overflow: visible;
    }}
    .axis text {{
      fill: var(--muted);
      font-size: 11px;
    }}
    .gridline {{
      stroke: var(--grid);
      stroke-width: 1;
    }}
    .series {{
      fill: none;
      stroke: var(--line);
      stroke-width: 1.8;
      vector-effect: non-scaling-stroke;
    }}
    .highlight {{
      fill: var(--highlight);
    }}
    .legend {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin-top: 8px;
      color: var(--muted);
      font-size: 12px;
    }}
    .swatch {{
      width: 14px;
      height: 14px;
      border-radius: 4px;
      background: var(--highlight);
      border: 1px solid rgba(235, 87, 87, 0.35);
    }}
    @media (max-width: 980px) {{
      .stats {{
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }}
      .grid {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>{payload["title"]}</h1>
      <p class="sub">All numeric columns except <code>front_rejected</code> and columns containing <code>med</code>. Highlight starts at 440.0 s.</p>
    </section>
    <section class="stats">
      <div class="stat"><div class="label">Points</div><div class="value">{payload["pointCount"]}</div></div>
      <div class="stat"><div class="label">Charts</div><div class="value">{payload["chartCount"]}</div></div>
      <div class="stat"><div class="label">Time Start</div><div class="value">{payload["timeMin"]:.3f}</div></div>
      <div class="stat"><div class="label">Time End</div><div class="value">{payload["timeMax"]:.3f}</div></div>
    </section>
    <section class="grid" id="charts"></section>
  </div>
  <script>
    const payload = {data_json};
    const chartsRoot = document.getElementById("charts");

    function fmt(value) {{
      if (!Number.isFinite(value)) return "n/a";
      const abs = Math.abs(value);
      if (abs >= 1000 || (abs > 0 && abs < 0.001)) return value.toExponential(2);
      return value.toFixed(3).replace(/\\.000$/, "");
    }}

    function svgEl(name, attrs) {{
      const el = document.createElementNS("http://www.w3.org/2000/svg", name);
      for (const [key, value] of Object.entries(attrs || {{}})) {{
        el.setAttribute(key, String(value));
      }}
      return el;
    }}

    function buildPath(xs, ys, xMin, xMax, yMin, yMax, plot) {{
      const dx = xMax - xMin || 1;
      const dy = yMax - yMin || 1;
      let path = "";
      let drawing = false;
      for (let i = 0; i < xs.length; i++) {{
        const y = ys[i];
        if (!Number.isFinite(y)) {{
          drawing = false;
          continue;
        }}
        const px = plot.left + ((xs[i] - xMin) / dx) * plot.width;
        const py = plot.top + plot.height - ((y - yMin) / dy) * plot.height;
        path += `${{drawing ? "L" : "M"}}${{px.toFixed(2)}},${{py.toFixed(2)}} `;
        drawing = true;
      }}
      return path.trim();
    }}

    function drawChart(chart) {{
      const card = document.createElement("article");
      card.className = "chart";
      card.innerHTML = `<h2>${{chart.title}}</h2><p class="meta">range: ${{fmt(chart.min)}} to ${{fmt(chart.max)}}</p>`;

      const width = 680;
      const height = 260;
      const plot = {{ left: 58, right: 16, top: 10, bottom: 34 }};
      plot.width = width - plot.left - plot.right;
      plot.height = height - plot.top - plot.bottom;

      const xMin = payload.timeMin;
      const xMax = payload.timeMax;
      const yMin = Number.isFinite(chart.min) ? chart.min : 0;
      let yMax = Number.isFinite(chart.max) ? chart.max : 1;
      if (yMax === yMin) yMax = yMin + 1;

      const svg = svgEl("svg", {{ viewBox: `0 0 ${{width}} ${{height}}`, role: "img", "aria-label": chart.title }});

      const highlightStart = Math.max(payload.highlightStart, xMin);
      if (highlightStart < xMax) {{
        const startX = plot.left + ((highlightStart - xMin) / (xMax - xMin || 1)) * plot.width;
        svg.appendChild(svgEl("rect", {{
          x: startX,
          y: plot.top,
          width: plot.left + plot.width - startX,
          height: plot.height,
          class: "highlight"
        }}));
      }}

      const gridTicks = 4;
      for (let i = 0; i <= gridTicks; i++) {{
        const y = plot.top + (plot.height * i / gridTicks);
        svg.appendChild(svgEl("line", {{
          x1: plot.left, y1: y, x2: plot.left + plot.width, y2: y, class: "gridline"
        }}));
        const tickValue = yMax - ((yMax - yMin) * i / gridTicks);
        const text = svgEl("text", {{ x: plot.left - 8, y: y + 4, "text-anchor": "end", class: "axis" }});
        text.textContent = fmt(tickValue);
        svg.appendChild(text);
      }}

      const xTicks = 5;
      for (let i = 0; i <= xTicks; i++) {{
        const x = plot.left + (plot.width * i / xTicks);
        svg.appendChild(svgEl("line", {{
          x1: x, y1: plot.top, x2: x, y2: plot.top + plot.height, class: "gridline"
        }}));
        const tickValue = xMin + ((xMax - xMin) * i / xTicks);
        const text = svgEl("text", {{ x: x, y: plot.top + plot.height + 22, "text-anchor": "middle", class: "axis" }});
        text.textContent = fmt(tickValue);
        svg.appendChild(text);
      }}

      svg.appendChild(svgEl("line", {{
        x1: plot.left, y1: plot.top + plot.height, x2: plot.left + plot.width, y2: plot.top + plot.height, stroke: "#7a7f85"
      }}));
      svg.appendChild(svgEl("line", {{
        x1: plot.left, y1: plot.top, x2: plot.left, y2: plot.top + plot.height, stroke: "#7a7f85"
      }}));

      const path = buildPath(chart.x, chart.y, xMin, xMax, yMin, yMax, plot);
      svg.appendChild(svgEl("path", {{ d: path, class: "series" }}));

      card.appendChild(svg);
      const legend = document.createElement("div");
      legend.className = "legend";
      legend.innerHTML = `<span class="swatch"></span><span>highlighted region: relative_time ≥ 440.0 s</span>`;
      card.appendChild(legend);
      return card;
    }}

    for (const chart of payload.charts) {{
      chartsRoot.appendChild(drawChart(chart));
    }}
  </script>
</body>
</html>
"""


def main() -> int:
    if len(sys.argv) < 3:
        print("Usage: generate_scan_metrics_highlight_html.py <csv_path> <out_html_path> [highlight_start]", file=sys.stderr)
        return 1

    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    highlight_start = float(sys.argv[3]) if len(sys.argv) > 3 else 440.0

    times, metric_columns, series = load_csv(csv_path)
    payload = build_payload(csv_path, times, metric_columns, series, highlight_start)
    html = build_html(payload)
    out_path.write_text(html, encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
