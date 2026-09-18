#!/usr/bin/env python3
import csv
import html
import math
import sys
from pathlib import Path


def parse_float(value: str):
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    if math.isnan(number):
        return None
    return number


def format_time(value):
    if value is None:
        return "N/A"
    return f"{value:.9f}"


def format_count(value):
    if value is None:
        return "N/A"
    return str(int(value))


def main():
    if len(sys.argv) != 2:
        print("usage: generate_front_back_valid_points_html.py <FAST-LIO2.csv>", file=sys.stderr)
        sys.exit(1)

    csv_path = Path(sys.argv[1]).resolve()
    if not csv_path.exists():
        print(f"missing file: {csv_path}", file=sys.stderr)
        sys.exit(1)

    rows_html = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rel_time = parse_float(row.get("relative_time"))
            front_valid = parse_float(row.get("front_effL"))
            back_valid = parse_float(row.get("back_effL"))
            rows_html.append(
                "<tr>"
                f"<td>{format_time(rel_time)}</td>"
                f"<td>{format_count(front_valid)}</td>"
                f"<td>{format_count(back_valid)}</td>"
                "</tr>"
            )

    output_path = csv_path.with_name("front_back_valid_points.html")
    title = f"{csv_path.parent.name} 최종 사용 앞/뒤 유효 포인트 표"

    html_text = f"""<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>{html.escape(title)}</title>
  <style>
    body {{
      margin: 0;
      font-family: "Noto Sans KR", "Malgun Gothic", sans-serif;
      background: #f7f4ee;
      color: #1f2933;
    }}
    .wrap {{
      max-width: 920px;
      margin: 0 auto;
      padding: 24px 16px 40px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    p {{
      margin: 0 0 16px;
      color: #52606d;
    }}
    .table-wrap {{
      background: #ffffff;
      border: 1px solid #d9d9d9;
      border-radius: 14px;
      overflow: auto;
      max-height: 82vh;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
    }}
    thead th {{
      position: sticky;
      top: 0;
      background: #efe7d8;
      text-align: left;
      padding: 12px 14px;
      border-bottom: 1px solid #d9d9d9;
      white-space: nowrap;
    }}
    tbody td {{
      padding: 10px 14px;
      border-bottom: 1px solid #ece7de;
      white-space: nowrap;
      font-variant-numeric: tabular-nums;
    }}
    tbody tr:nth-child(even) {{
      background: #fcfaf6;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <h1>{html.escape(title)}</h1>
    <p>소스: {html.escape(str(csv_path))}<br />기준 컬럼: <code>front_effL</code>, <code>back_effL</code></p>
    <div class="table-wrap">
      <table>
        <thead>
          <tr>
            <th>relative_time</th>
            <th>front_effL</th>
            <th>back_effL</th>
          </tr>
        </thead>
        <tbody>
          {''.join(rows_html)}
        </tbody>
      </table>
    </div>
  </div>
</body>
</html>
"""

    output_path.write_text(html_text, encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
