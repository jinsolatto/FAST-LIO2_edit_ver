#!/usr/bin/env python3
"""Generate an HTML table report for scan metrics."""

from __future__ import annotations

import csv
import html
import math
import sys
from pathlib import Path


HIGHLIGHT_START_DEFAULT = 440.0


def parse_float(value: str) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(number) or math.isinf(number):
        return None
    return number


def build_html(csv_name: str, headers: list[str], rows: list[dict[str, str]], highlight_start: float) -> str:
    header_html = "".join(f"<th>{html.escape(name)}</th>" for name in headers)
    body_rows = []
    for row in rows:
        time_value = parse_float(row.get("relative_time", ""))
        css = ' class="highlight-row"' if time_value is not None and time_value >= highlight_start else ""
        cells = "".join(f"<td>{html.escape(row.get(name, ''))}</td>" for name in headers)
        body_rows.append(f"<tr{css}>{cells}</tr>")

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(csv_name)}</title>
  <style>
    :root {{
      --bg: #f7f3ed;
      --panel: #fffdf8;
      --ink: #1f2937;
      --muted: #6b7280;
      --border: #d8d0c4;
      --header: #efe7da;
      --highlight: #fff2cc;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", Arial, sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(244, 196, 128, 0.20), transparent 26%),
        linear-gradient(180deg, #faf7f1 0%, #f2ede4 100%);
    }}
    .wrap {{
      max-width: 100%;
      padding: 20px;
    }}
    .hero {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px 20px;
      margin-bottom: 16px;
      box-shadow: 0 10px 24px rgba(60, 50, 38, 0.06);
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    .sub {{
      margin: 0;
      color: var(--muted);
      font-size: 14px;
    }}
    .table-shell {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      overflow: auto;
      box-shadow: 0 10px 24px rgba(60, 50, 38, 0.06);
    }}
    table {{
      border-collapse: collapse;
      width: max-content;
      min-width: 100%;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
    }}
    th, td {{
      border-bottom: 1px solid #e9e2d7;
      border-right: 1px solid #efe7da;
      padding: 6px 8px;
      white-space: nowrap;
      text-align: right;
    }}
    th {{
      position: sticky;
      top: 0;
      background: var(--header);
      z-index: 2;
      font-weight: 700;
    }}
    th:first-child, td:first-child {{
      position: sticky;
      left: 0;
      z-index: 1;
      background: #fbf8f2;
    }}
    th:first-child {{
      z-index: 3;
      background: var(--header);
    }}
    .highlight-row td {{
      background: var(--highlight);
    }}
    .highlight-row td:first-child {{
      background: #fde8aa;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>{html.escape(csv_name)}</h1>
      <p class="sub">Per-scan table. Excludes <code>front_rejected</code> and columns containing <code>med</code>. Rows with <code>relative_time &ge; {highlight_start:.1f}</code> are highlighted.</p>
    </section>
    <section class="table-shell">
      <table>
        <thead>
          <tr>{header_html}</tr>
        </thead>
        <tbody>
          {''.join(body_rows)}
        </tbody>
      </table>
    </section>
  </div>
</body>
</html>
"""


def main() -> int:
    if len(sys.argv) < 3:
        print(
            "Usage: python3 generate_scan_metrics_table_html.py <csv_path> <out_html_path> [highlight_start]",
            file=sys.stderr,
        )
        return 1

    csv_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2])
    highlight_start = float(sys.argv[3]) if len(sys.argv) > 3 else HIGHLIGHT_START_DEFAULT

    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
        headers = [
            name
            for name in fieldnames
            if name != "front_rejected" and "med" not in name
        ]
        rows = list(reader)

    html_text = build_html(csv_path.name, headers, rows, highlight_start)
    out_path.write_text(html_text, encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
