#!/usr/bin/env python3
"""Generate graph-based comparison HTML for front/back effL over selected ranges."""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path


RANGES = [(280.0, 375.0), (440.0, 470.0)]


def parse_float(value: str) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def parse_table_html(path: Path) -> list[dict[str, str]]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    headers = re.findall(r"<th>(.*?)</th>", text, flags=re.S)
    match = re.search(r"<tbody>\s*(.*?)\s*</tbody>", text, flags=re.S)
    if not match:
        raise ValueError(f"No table body found in {path}")
    rows = []
    for tr in re.findall(r"<tr(?: class=\"[^\"]*\")?>(.*?)</tr>", match.group(1), flags=re.S):
        values = re.findall(r"<td>(.*?)</td>", tr, flags=re.S)
        if len(values) != len(headers):
            continue
        rows.append(dict(zip(headers, values)))
    return rows


def select_rows(rows: list[dict[str, str]], start: float, end: float) -> list[dict[str, float]]:
    selected = []
    for row in rows:
        t = parse_float(row.get("relative_time", ""))
        if t is None or not (start <= t <= end):
            continue
        selected.append(
            {
                "relative_time": t,
                "front_effL": parse_float(row.get("front_effL", "")),
                "back_effL": parse_float(row.get("back_effL", "")),
            }
        )
    return selected


def build_payload(custom_path: Path, fastlio_path: Path) -> dict:
    custom_rows = parse_table_html(custom_path)
    fastlio_rows = parse_table_html(fastlio_path)

    sections = []
    for start, end in RANGES:
        custom_sel = select_rows(custom_rows, start, end)
        fastlio_sel = select_rows(fastlio_rows, start, end)
        sections.append(
            {
                "title": f"{int(start)}-{int(end)} s",
                "start": start,
                "end": end,
                "custom": custom_sel,
                "fastlio": fastlio_sel,
            }
        )

    return {
        "title": "front_effL / back_effL Graph Comparison",
        "customName": custom_path.name,
        "fastlioName": fastlio_path.name,
        "sections": sections,
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
      --bg: #f4efe7;
      --panel: rgba(255, 252, 246, 0.96);
      --ink: #1f2937;
      --muted: #667085;
      --border: #dfd5c8;
      --custom-front: #d9485f;
      --custom-back: #1d7f5f;
      --fast-front: #f2a65a;
      --fast-back: #5b8def;
      --grid: #e8dfd3;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top right, rgba(91, 141, 239, 0.14), transparent 24%),
        radial-gradient(circle at top left, rgba(233, 185, 73, 0.18), transparent 22%),
        linear-gradient(180deg, #fbf8f3 0%, #f1ebe2 100%);
    }}
    .wrap {{
      max-width: 1540px;
      margin: 0 auto;
      padding: 24px;
    }}
    .hero {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 22px;
      padding: 22px 24px;
      box-shadow: 0 16px 38px rgba(44, 38, 29, 0.08);
      margin-bottom: 18px;
    }}
    h1 {{
      margin: 0 0 10px;
      font-size: 30px;
    }}
    .hero p {{
      margin: 0;
      color: var(--muted);
      line-height: 1.5;
    }}
    .legend {{
      display: flex;
      gap: 14px;
      flex-wrap: wrap;
      margin-top: 12px;
      font-size: 13px;
      color: var(--muted);
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .swatch {{
      width: 12px;
      height: 12px;
      border-radius: 999px;
    }}
    .section {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 22px;
      padding: 18px;
      box-shadow: 0 14px 32px rgba(44, 38, 29, 0.06);
      margin-bottom: 18px;
    }}
    .section h2 {{
      margin: 0 0 6px;
      font-size: 24px;
    }}
    .section p {{
      margin: 0 0 16px;
      color: var(--muted);
    }}
    .chart-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }}
    .chart-card {{
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 12px 14px 10px;
      background: rgba(255,255,255,0.58);
    }}
    .chart-card h3 {{
      margin: 0 0 8px;
      font-size: 17px;
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
    .line {{
      fill: none;
      stroke-width: 2;
      vector-effect: non-scaling-stroke;
    }}
    .meta {{
      margin-top: 8px;
      font-size: 12px;
      color: var(--muted);
    }}
    @media (max-width: 980px) {{
      .chart-grid {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>{payload["title"]}</h1>
      <p>Comparing <code>{payload["customName"]}</code> and <code>{payload["fastlioName"]}</code> over the 280-375 s and 440-470 s windows. Each plot uses actual relative time on the x-axis.</p>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: var(--custom-front)"></span>Custom front_effL</span>
        <span class="legend-item"><span class="swatch" style="background: var(--custom-back)"></span>Custom back_effL</span>
        <span class="legend-item"><span class="swatch" style="background: var(--fast-front)"></span>FAST-LIO2 front_effL</span>
        <span class="legend-item"><span class="swatch" style="background: var(--fast-back)"></span>FAST-LIO2 back_effL</span>
      </div>
    </section>
    <div id="sections"></div>
  </div>
  <script>
    const payload = {data_json};

    function svgEl(name, attrs) {{
      const el = document.createElementNS("http://www.w3.org/2000/svg", name);
      for (const [key, value] of Object.entries(attrs || {{}})) {{
        el.setAttribute(key, String(value));
      }}
      return el;
    }}

    function fmt(value) {{
      if (!Number.isFinite(value)) return "n/a";
      if (Math.abs(value) >= 1000 || (Math.abs(value) > 0 && Math.abs(value) < 0.001)) return value.toExponential(2);
      return value.toFixed(3).replace(/\\.000$/, "");
    }}

    function pathFromSeries(rows, key, xMin, xMax, yMin, yMax, plot) {{
      const dx = xMax - xMin || 1;
      const dy = yMax - yMin || 1;
      let d = "";
      let drawing = false;
      for (const row of rows) {{
        const x = row.relative_time;
        const y = row[key];
        if (!Number.isFinite(x) || !Number.isFinite(y)) {{
          drawing = false;
          continue;
        }}
        const px = plot.left + ((x - xMin) / dx) * plot.width;
        const py = plot.top + plot.height - ((y - yMin) / dy) * plot.height;
        d += `${{drawing ? "L" : "M"}}${{px.toFixed(2)}},${{py.toFixed(2)}} `;
        drawing = true;
      }}
      return d.trim();
    }}

    function buildChart(title, section, key, colors) {{
      const card = document.createElement("div");
      card.className = "chart-card";
      card.innerHTML = `<h3>${{title}}</h3>`;

      const width = 700;
      const height = 260;
      const plot = {{ left: 56, right: 16, top: 10, bottom: 34 }};
      plot.width = width - plot.left - plot.right;
      plot.height = height - plot.top - plot.bottom;

      const xs = [...section.custom.map(r => r.relative_time), ...section.fastlio.map(r => r.relative_time)].filter(Number.isFinite);
      const ys = [...section.custom.map(r => r[key]), ...section.fastlio.map(r => r[key])].filter(Number.isFinite);
      const xMin = xs.length ? Math.min(...xs) : section.start;
      const xMax = xs.length ? Math.max(...xs) : section.end;
      const yMin = ys.length ? Math.min(...ys) : 0;
      const yMax = ys.length ? Math.max(...ys) : 1;
      const yPad = Math.max((yMax - yMin) * 0.08, 1);
      const yLo = Math.max(0, yMin - yPad);
      const yHi = yMax + yPad;

      const svg = svgEl("svg", {{ viewBox: `0 0 ${{width}} ${{height}}`, role: "img", "aria-label": title }});

      for (let i = 0; i <= 4; i++) {{
        const y = plot.top + (plot.height * i / 4);
        svg.appendChild(svgEl("line", {{ x1: plot.left, y1: y, x2: plot.left + plot.width, y2: y, class: "gridline" }}));
        const tickValue = yHi - ((yHi - yLo) * i / 4);
        const label = svgEl("text", {{ x: plot.left - 8, y: y + 4, "text-anchor": "end", class: "axis" }});
        label.textContent = fmt(tickValue);
        svg.appendChild(label);
      }}

      for (let i = 0; i <= 5; i++) {{
        const x = plot.left + (plot.width * i / 5);
        svg.appendChild(svgEl("line", {{ x1: x, y1: plot.top, x2: x, y2: plot.top + plot.height, class: "gridline" }}));
        const tickValue = xMin + ((xMax - xMin) * i / 5);
        const label = svgEl("text", {{ x: x, y: plot.top + plot.height + 22, "text-anchor": "middle", class: "axis" }});
        label.textContent = fmt(tickValue);
        svg.appendChild(label);
      }}

      svg.appendChild(svgEl("line", {{ x1: plot.left, y1: plot.top + plot.height, x2: plot.left + plot.width, y2: plot.top + plot.height, stroke: "#7a7f85" }}));
      svg.appendChild(svgEl("line", {{ x1: plot.left, y1: plot.top, x2: plot.left, y2: plot.top + plot.height, stroke: "#7a7f85" }}));

      const customPath = pathFromSeries(section.custom, key, xMin, xMax, yLo, yHi, plot);
      const fastPath = pathFromSeries(section.fastlio, key, xMin, xMax, yLo, yHi, plot);
      svg.appendChild(svgEl("path", {{ d: customPath, class: "line", stroke: colors.custom }}));
      svg.appendChild(svgEl("path", {{ d: fastPath, class: "line", stroke: colors.fast }}));

      card.appendChild(svg);
      const meta = document.createElement("div");
      meta.className = "meta";
      meta.textContent = `Custom rows: ${{section.custom.length}} | FAST-LIO2 rows: ${{section.fastlio.length}}`;
      card.appendChild(meta);
      return card;
    }}

    const root = document.getElementById("sections");
    for (const section of payload.sections) {{
      const el = document.createElement("section");
      el.className = "section";
      el.innerHTML = `<h2>${{section.title}}</h2><p>Time window ${{fmt(section.start)}}s to ${{fmt(section.end)}}s.</p>`;
      const grid = document.createElement("div");
      grid.className = "chart-grid";
      grid.appendChild(buildChart("front_effL", section, "front_effL", {{ custom: "var(--custom-front)", fast: "var(--fast-front)" }}));
      grid.appendChild(buildChart("back_effL", section, "back_effL", {{ custom: "var(--custom-back)", fast: "var(--fast-back)" }}));
      el.appendChild(grid);
      root.appendChild(el);
    }}
  </script>
</body>
</html>
"""


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "Usage: python3 generate_effl_range_comparison_graph_html.py <custom_table_html> <fastlio_table_html> <out_html>",
            file=sys.stderr,
        )
        return 1

    custom_path = Path(sys.argv[1])
    fastlio_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3])
    out_path.write_text(build_html(build_payload(custom_path, fastlio_path)), encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
