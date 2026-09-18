#!/usr/bin/env python3
"""Generate a focused comparison HTML for front/back effL over selected time ranges."""

from __future__ import annotations

import html
import math
import re
import statistics
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


def filter_range(rows: list[dict[str, str]], start: float, end: float) -> list[dict[str, str]]:
    filtered = []
    for row in rows:
        t = parse_float(row.get("relative_time", ""))
        if t is None:
            continue
        if start <= t <= end:
            filtered.append(row)
    return filtered


def get_num(row: dict[str, str] | None, key: str) -> float | None:
    if row is None:
        return None
    return parse_float(row.get(key, ""))


def fmt(value: float | None, digits: int = 1) -> str:
    if value is None:
        return ""
    if abs(value - round(value)) < 1e-9:
        return str(int(round(value)))
    return f"{value:.{digits}f}"


def summarize(rows: list[dict[str, str]], key: str) -> dict[str, float | None]:
    values = [get_num(row, key) for row in rows]
    finite = [v for v in values if v is not None]
    if not finite:
        return {"count": len(rows), "sum": None, "mean": None, "max": None}
    return {
        "count": len(rows),
        "sum": sum(finite),
        "mean": statistics.fmean(finite),
        "max": max(finite),
    }


def delta_class(delta: float | None) -> str:
    if delta is None:
        return "delta-empty"
    if delta > 0:
        return "delta-pos"
    if delta < 0:
        return "delta-neg"
    return "delta-zero"


def section_html(
    title: str,
    start: float,
    end: float,
    custom_rows: list[dict[str, str]],
    fastlio_rows: list[dict[str, str]],
) -> str:
    custom_front = summarize(custom_rows, "front_effL")
    custom_back = summarize(custom_rows, "back_effL")
    fast_front = summarize(fastlio_rows, "front_effL")
    fast_back = summarize(fastlio_rows, "back_effL")

    max_len = max(len(custom_rows), len(fastlio_rows))
    table_rows = []
    for idx in range(max_len):
        c = custom_rows[idx] if idx < len(custom_rows) else None
        f = fastlio_rows[idx] if idx < len(fastlio_rows) else None
        c_front = get_num(c, "front_effL")
        f_front = get_num(f, "front_effL")
        c_back = get_num(c, "back_effL")
        f_back = get_num(f, "back_effL")
        d_front = None if c_front is None or f_front is None else c_front - f_front
        d_back = None if c_back is None or f_back is None else c_back - f_back
        table_rows.append(
            f"""
            <tr>
              <td>{idx + 1}</td>
              <td>{fmt(get_num(c, "relative_time"), 3)}</td>
              <td>{fmt(c_front)}</td>
              <td>{fmt(c_back)}</td>
              <td>{fmt(get_num(f, "relative_time"), 3)}</td>
              <td>{fmt(f_front)}</td>
              <td>{fmt(f_back)}</td>
              <td class="{delta_class(d_front)}">{fmt(d_front)}</td>
              <td class="{delta_class(d_back)}">{fmt(d_back)}</td>
            </tr>
            """
        )

    return f"""
    <section class="range-card">
      <div class="range-head">
        <div>
          <h2>{html.escape(title)}</h2>
          <p>{start:.1f}s to {end:.1f}s. Rows are paired by scan order inside the selected range.</p>
        </div>
      </div>
      <div class="summary-grid">
        <div class="summary-panel">
          <h3>Custom</h3>
          <table class="mini">
            <tr><th></th><th>Rows</th><th>Sum</th><th>Mean</th><th>Max</th></tr>
            <tr><th>front_effL</th><td>{fmt(custom_front["count"], 0)}</td><td>{fmt(custom_front["sum"])}</td><td>{fmt(custom_front["mean"])}</td><td>{fmt(custom_front["max"])}</td></tr>
            <tr><th>back_effL</th><td>{fmt(custom_back["count"], 0)}</td><td>{fmt(custom_back["sum"])}</td><td>{fmt(custom_back["mean"])}</td><td>{fmt(custom_back["max"])}</td></tr>
          </table>
        </div>
        <div class="summary-panel">
          <h3>FAST-LIO2</h3>
          <table class="mini">
            <tr><th></th><th>Rows</th><th>Sum</th><th>Mean</th><th>Max</th></tr>
            <tr><th>front_effL</th><td>{fmt(fast_front["count"], 0)}</td><td>{fmt(fast_front["sum"])}</td><td>{fmt(fast_front["mean"])}</td><td>{fmt(fast_front["max"])}</td></tr>
            <tr><th>back_effL</th><td>{fmt(fast_back["count"], 0)}</td><td>{fmt(fast_back["sum"])}</td><td>{fmt(fast_back["mean"])}</td><td>{fmt(fast_back["max"])}</td></tr>
          </table>
        </div>
        <div class="summary-panel emphasis">
          <h3>Delta Summary</h3>
          <table class="mini">
            <tr><th></th><th>Custom - FAST</th></tr>
            <tr><th>front_effL sum</th><td>{fmt((custom_front["sum"] or 0) - (fast_front["sum"] or 0))}</td></tr>
            <tr><th>back_effL sum</th><td>{fmt((custom_back["sum"] or 0) - (fast_back["sum"] or 0))}</td></tr>
            <tr><th>front_effL mean</th><td>{fmt((custom_front["mean"] or 0) - (fast_front["mean"] or 0))}</td></tr>
            <tr><th>back_effL mean</th><td>{fmt((custom_back["mean"] or 0) - (fast_back["mean"] or 0))}</td></tr>
          </table>
        </div>
      </div>
      <div class="table-shell">
        <table class="compare">
          <thead>
            <tr>
              <th rowspan="2">#</th>
              <th colspan="3">Custom</th>
              <th colspan="3">FAST-LIO2</th>
              <th colspan="2">Delta</th>
            </tr>
            <tr>
              <th>t</th>
              <th>front_effL</th>
              <th>back_effL</th>
              <th>t</th>
              <th>front_effL</th>
              <th>back_effL</th>
              <th>front</th>
              <th>back</th>
            </tr>
          </thead>
          <tbody>
            {''.join(table_rows)}
          </tbody>
        </table>
      </div>
    </section>
    """


def build_html(custom_path: Path, fastlio_path: Path) -> str:
    custom_all = parse_table_html(custom_path)
    fastlio_all = parse_table_html(fastlio_path)
    sections = []
    for start, end in RANGES:
        sections.append(
            section_html(
                f"Range {start:.0f}-{end:.0f}s",
                start,
                end,
                filter_range(custom_all, start, end),
                filter_range(fastlio_all, start, end),
            )
        )

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>front/back effL comparison</title>
  <style>
    :root {{
      --bg: #f4efe7;
      --panel: rgba(255, 252, 246, 0.96);
      --ink: #1f2937;
      --muted: #667085;
      --border: #dfd5c8;
      --accent: #7c3aed;
      --warm: #e9b949;
      --cool: #5b8def;
      --neg: #ffe1dc;
      --pos: #dff5e7;
      --zero: #f4efe7;
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
    .range-card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 22px;
      padding: 18px;
      box-shadow: 0 14px 32px rgba(44, 38, 29, 0.06);
      margin-bottom: 18px;
    }}
    .range-head h2 {{
      margin: 0 0 6px;
      font-size: 24px;
    }}
    .range-head p {{
      margin: 0 0 14px;
      color: var(--muted);
    }}
    .summary-grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 14px;
    }}
    .summary-panel {{
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 12px 14px;
      background: rgba(255,255,255,0.7);
    }}
    .summary-panel.emphasis {{
      background: linear-gradient(135deg, rgba(124,58,237,0.07), rgba(91,141,239,0.08));
    }}
    .summary-panel h3 {{
      margin: 0 0 8px;
      font-size: 16px;
    }}
    .mini {{
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
    }}
    .mini th, .mini td {{
      border-bottom: 1px solid #ece4d8;
      padding: 6px 4px;
      text-align: right;
    }}
    .mini th:first-child, .mini td:first-child {{
      text-align: left;
    }}
    .table-shell {{
      overflow: auto;
      border: 1px solid var(--border);
      border-radius: 16px;
    }}
    .compare {{
      border-collapse: collapse;
      width: max-content;
      min-width: 100%;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
    }}
    .compare th, .compare td {{
      padding: 6px 8px;
      border-right: 1px solid #ece4d8;
      border-bottom: 1px solid #ece4d8;
      text-align: right;
      white-space: nowrap;
    }}
    .compare thead th {{
      position: sticky;
      top: 0;
      background: #f0e8dc;
      z-index: 2;
    }}
    .compare tbody tr:nth-child(odd) td {{
      background: rgba(255,255,255,0.45);
    }}
    .compare tbody tr:hover td {{
      background: rgba(91,141,239,0.10);
    }}
    .delta-pos {{ background: var(--pos) !important; font-weight: 700; }}
    .delta-neg {{ background: var(--neg) !important; font-weight: 700; }}
    .delta-zero {{ background: var(--zero) !important; color: #6b7280; }}
    .delta-empty {{ color: #9ca3af; }}
    @media (max-width: 980px) {{
      .summary-grid {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>front_effL / back_effL Comparison</h1>
      <p>
        Source tables:
        <code>{html.escape(custom_path.name)}</code> and
        <code>{html.escape(fastlio_path.name)}</code>.
        Focus ranges are <code>280-375 s</code> and <code>440-470 s</code>.
        The tables below are optimized for scan-by-scan effL count comparison, with delta columns highlighted.
      </p>
    </section>
    {''.join(sections)}
  </div>
</body>
</html>
"""


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "Usage: python3 generate_effl_range_comparison_html.py <custom_table_html> <fastlio_table_html> <out_html>",
            file=sys.stderr,
        )
        return 1

    custom_path = Path(sys.argv[1])
    fastlio_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3])
    out_path.write_text(build_html(custom_path, fastlio_path), encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
