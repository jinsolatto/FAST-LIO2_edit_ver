#!/usr/bin/env python3
"""Generate a 'scan table' HTML report from a scan_metrics*.csv file.

Reproduces the point_rejection_280_313.html layout and adds the
translation/rotation minimum eigenvalue columns:
t, raw, front_rejected, front_raw_count, back_raw_count, undist, down, eff1, effL,
t_lambda_min, r_lambda_min.

Usage:
    python3 generate_scan_table_html.py <csv_path> <out_html_path> [start_time] [end_time]
"""
import csv
import sys

QUADRANT_SVG = """
<svg viewBox="0 0 220 220" width="180" height="180" aria-label="quadrant legend" role="img">
  <defs>
    <filter id="shadow" x="-20%" y="-20%" width="140%" height="140%">
      <feDropShadow dx="0" dy="1" stdDeviation="1.5" flood-color="#999" flood-opacity="0.25"/>
    </filter>
  </defs>
  <circle cx="110" cy="110" r="92" fill="#fff" stroke="#ddd" stroke-width="1"/>
  <path d="M110 110 L110 18 A92 92 0 0 1 175.1 45.0 Z" fill="#f9d6d3"/>
  <path d="M110 110 L175.1 45.0 A92 92 0 0 1 202 110 Z" fill="#fae6bf"/>
  <path d="M110 110 L202 110 A92 92 0 0 1 45.0 175.1 Z" fill="none"/>
  <path d="M110 110 L175.1 175.1 A92 92 0 0 1 110 202 Z" fill="#d8e7fb"/>
  <path d="M110 110 L45.0 175.1 A92 92 0 0 1 18 110 Z" fill="#dbead2"/>
  <path d="M110 110 L18 110 A92 92 0 0 1 45.0 45.0 Z" fill="#dbead2"/>
  <line x1="110" y1="110" x2="175.1" y2="45.0" stroke="#555" stroke-dasharray="4 4"/>
  <line x1="110" y1="110" x2="175.1" y2="175.1" stroke="#555" stroke-dasharray="4 4"/>
  <line x1="110" y1="110" x2="45.0" y2="175.1" stroke="#555" stroke-dasharray="4 4"/>
  <line x1="110" y1="110" x2="45.0" y2="45.0" stroke="#555" stroke-dasharray="4 4"/>
  <g filter="url(#shadow)">
    <rect x="100" y="86" width="20" height="48" rx="5" fill="#f5f5f5" stroke="#333"/>
    <rect x="104" y="80" width="12" height="10" rx="2" fill="#f5f5f5" stroke="#333"/>
    <circle cx="110" cy="95" r="1.5" fill="#666"/>
    <circle cx="110" cy="102" r="1.5" fill="#666"/>
    <circle cx="110" cy="109" r="1.5" fill="#666"/>
    <circle cx="110" cy="116" r="1.5" fill="#666"/>
    <rect x="92" y="92" width="8" height="16" rx="2" fill="#222"/>
    <rect x="120" y="92" width="8" height="16" rx="2" fill="#222"/>
    <rect x="92" y="112" width="8" height="16" rx="2" fill="#222"/>
    <rect x="120" y="112" width="8" height="16" rx="2" fill="#222"/>
    <rect x="104" y="75" width="12" height="4" rx="1" fill="#222"/>
    <rect x="104" y="134" width="12" height="4" rx="1" fill="#222"/>
  </g>
  <text x="110" y="46" text-anchor="middle" font-size="14" font-weight="700" fill="#c91818">Q0</text>
  <text x="154" y="112" text-anchor="middle" font-size="14" font-weight="700" fill="#d97a00">Q1</text>
  <text x="110" y="180" text-anchor="middle" font-size="14" font-weight="700" fill="#1f62c4">Q2</text>
  <text x="66" y="112" text-anchor="middle" font-size="14" font-weight="700" fill="#1b7b39">Q3</text>
  <text x="110" y="11" text-anchor="middle" font-size="11" font-weight="700" fill="#222">0°</text>
  <text x="197" y="111" text-anchor="middle" font-size="11" font-weight="700" fill="#d97a00">+90°</text>
  <text x="110" y="214" text-anchor="middle" font-size="11" font-weight="700" fill="#1f62c4">±180°</text>
  <text x="22" y="111" text-anchor="middle" font-size="11" font-weight="700" fill="#1b7b39">-90°</text>
</svg>
"""

COLUMNS = [
    ("relative_time", "t", None),
    ("raw_point_count", "raw", None),
    ("front_raw_count", "front_raw<br>count", "entropy-header"),
    ("back_raw_count", "back_raw<br>count", "entropy-header"),
    ("feats_undistort_size", "undist", None),
    ("feats_down_size", "down", None),
    ("eff_feat_num_first", "eff1", None),
    ("eff_feat_num_last", "effL", None),
    ("front_effL", "front<br>effL", "entropy-header"),
    ("back_effL", "back<br>effL", "entropy-header"),
    ("front_undist", "front<br>undist", "entropy-header"),
    ("back_undist", "back<br>undist", "entropy-header"),
    ("front_down", "front<br>down", "entropy-header"),
    ("back_down", "back<br>down", "entropy-header"),
    ("t_lambda_min", "t_lambda<br>min", "entropy-header"),
    ("r_lambda_min", "r_lambda<br>min", "entropy-header"),
]

HEAD = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>scan table</title>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 14px; }}
    h1 {{ margin-bottom: 8px; }}
    h2 {{ margin: 28px 0 8px; }}
    p {{ color: #444; margin-right: 120px; }}
    .sections {{ display: grid; grid-template-columns: repeat(1, minmax(0, 1fr)); gap: 16px; align-items: start; }}
    section {{ margin-bottom: 0; position: relative; padding-top: 4px; }}
    .legend-card {{ position: absolute; top: 0; right: 0; display: flex; justify-content: flex-end; align-items: flex-start; }}
    .legend-card svg {{ width: 96px; height: 96px; }}
    .toggle-qcols {{ margin: 0 0 8px; padding: 4px 8px; font-size: 11px; border: 1px solid #bbb; background: #f7f7f7; border-radius: 4px; cursor: pointer; }}
    .table-card {{ min-width: 0; overflow-x: hidden; }}
    table {{ border-collapse: collapse; font-family: monospace; font-size: 11px; table-layout: fixed; width: 100%; }}
    th, td {{ border: 1px solid #ccc; padding: 3px 4px; text-align: right; }}
    th:first-child, td:first-child {{ text-align: right; }}
    thead th {{ background: #f3f3f3; white-space: normal; word-break: break-word; overflow-wrap: anywhere; line-height: 1.05; }}
    tbody td {{ white-space: nowrap; }}
    .entropy-header {{ padding-left: 4px; padding-right: 4px; line-height: 1.05; }}
    .entropy-header br {{ display: block; }}
    th.col-undist, td.col-undist {{ background: #fff8cc; }}
    th.col-down, td.col-down {{ background: #e6f4ff; }}
    th.col-eff-last, td.col-eff-last {{ background: #eef9e8; }}
    .quadrant-col {{ display: none; }}
    .table-card.show-qcols .quadrant-col {{ display: table-cell; }}
  </style>
  <script>
    function toggleQuadrantCols(button) {{
      const card = button.nextElementSibling;
      const expanded = card.classList.toggle('show-qcols');
      button.textContent = expanded ? 'q0~q3 columns 접기' : 'q0~q3 columns 펼치기';
    }}
  </script>
</head>
<body>
  <h1>Scan Table</h1>
  <p>time range: {t0:.1f} s to {t1:.1f} s</p>
  <div class="sections">

  <section>
    <h2>{csv_name}</h2>
    <p>time range: {t0:.1f} s to {t1:.1f} s, scans: {n}</p>
    <div class="legend-card">
      {svg}
    </div>
    <button type="button" class="toggle-qcols" onclick="toggleQuadrantCols(this)">q0~q3 columns 펼치기</button>
    <div class="table-card">
      <table class="scan-table"><thead><tr>{header_cells}</tr></thead><tbody>{rows}</tbody></table>
    </div>
  </section>

  </div>
</body>
</html>
"""


def fmt(v):
    if v is None or v == "" or v == "nan":
        return ""
    try:
        f = float(v)
        if f == int(f):
            return str(int(f))
        return f"{f:.1f}"
    except ValueError:
        return v


def main():
    csv_path = sys.argv[1]
    out_path = sys.argv[2]
    start_time = float(sys.argv[3]) if len(sys.argv) > 3 else None
    end_time = float(sys.argv[4]) if len(sys.argv) > 4 else None

    with open(csv_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    def rt(row):
        try:
            return float(row["relative_time"])
        except (KeyError, ValueError):
            return None

    filtered = []
    for row in rows:
        t = rt(row)
        if t is None:
            continue
        if start_time is not None and t < start_time:
            continue
        if end_time is not None and t > end_time:
            continue
        filtered.append(row)

    if not filtered:
        print("No rows matched the given time range.")
        return

    t0 = rt(filtered[0])
    t1 = rt(filtered[-1])

    header_cells = "".join(
        f'<th class="{cls}">{label}</th>' if cls else f"<th>{label}</th>"
        for _, label, cls in COLUMNS
    )

    row_htmls = []
    for row in filtered:
        cells = "".join(f"<td>{fmt(row.get(col))}</td>" for col, _, _ in COLUMNS)
        row_htmls.append(f"<tr>{cells}</tr>")

    import os

    html = HEAD.format(
        t0=t0,
        t1=t1,
        csv_name=os.path.basename(csv_path),
        n=len(filtered),
        svg=QUADRANT_SVG,
        header_cells=header_cells,
        rows="".join(row_htmls),
    )

    with open(out_path, "w") as f:
        f.write(html)

    print(f"Wrote {out_path}: {len(filtered)} rows, t={t0:.1f}..{t1:.1f}")


if __name__ == "__main__":
    main()
