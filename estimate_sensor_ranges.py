"""estimate_sensor_ranges.py
===========================
Estimate sensor ranges from a structured calibration JSON.

Outputs:
1) Per-sensor complete range estimates (all available real data points)
2) Joint-grouped view (right-hand reference mapping)

The script uses:
- fully_open_sensors
- joints.*.sensor_readings_when_joint_closed
- sensor_live_limits_deg (if present)

It writes both CSV and Markdown reports.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


RIGHT_HAND_JOINT_GROUPS = [
    {"joint": "Index abduction", "sensor": 0, "motor": 8},
    {"joint": "Index MCP", "sensor": 3, "motor": 9},
    {"joint": "Index PIP", "sensor": 2, "motor": 7},
    {"joint": "Index DIP", "sensor": 1, "motor": 7},
    {"joint": "Thumb CMC", "sensor": 6, "motor": 12},
    {"joint": "Thumb MCP", "sensor": 5, "motor": 14},
    {"joint": "Thumb DIP", "sensor": 4, "motor": 13},
]


def _finite(v: float) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def _safe_float(v, default=float("nan")) -> float:
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default


def _collect_sensor_values(cal_map: dict) -> dict[int, dict[str, list[float]]]:
    """Collect all available sensor degree samples from map sections."""
    values: dict[int, dict[str, list[float]]] = {}

    def ensure_sensor(s: int):
        if s not in values:
            values[s] = {"open": [], "closed": [], "live_min": [], "live_max": []}

    fully_open = cal_map.get("fully_open_sensors", {})
    for s_key, rec in fully_open.items():
        s = int(s_key)
        ensure_sensor(s)
        d = _safe_float(rec.get("deg"))
        if _finite(d):
            values[s]["open"].append(d)

    joints = cal_map.get("joints", {})
    for _j_key, j_data in joints.items():
        closed = j_data.get("sensor_readings_when_joint_closed", {})
        for s_key, rec in closed.items():
            s = int(s_key)
            ensure_sensor(s)
            d = _safe_float(rec.get("deg"))
            if _finite(d):
                values[s]["closed"].append(d)

    live_limits = cal_map.get("sensor_live_limits_deg", {})
    for s_key, rec in live_limits.items():
        s = int(s_key)
        ensure_sensor(s)
        lo = _safe_float(rec.get("min_deg"))
        hi = _safe_float(rec.get("max_deg"))
        if _finite(lo):
            values[s]["live_min"].append(lo)
        if _finite(hi):
            values[s]["live_max"].append(hi)

    return values


def _sensor_summary_rows(cal_map: dict) -> list[dict]:
    values = _collect_sensor_values(cal_map)
    rows = []

    for s in sorted(values.keys()):
        v = values[s]
        observed = v["open"] + v["closed"]
        live = v["live_min"] + v["live_max"]
        complete = observed + live

        obs_min = min(observed) if observed else float("nan")
        obs_max = max(observed) if observed else float("nan")
        obs_span = (obs_max - obs_min) if _finite(obs_min) and _finite(obs_max) else float("nan")

        live_min = min(v["live_min"]) if v["live_min"] else float("nan")
        live_max = max(v["live_max"]) if v["live_max"] else float("nan")
        live_span = (live_max - live_min) if _finite(live_min) and _finite(live_max) else float("nan")

        comp_min = min(complete) if complete else float("nan")
        comp_max = max(complete) if complete else float("nan")
        comp_span = (comp_max - comp_min) if _finite(comp_min) and _finite(comp_max) else float("nan")

        rows.append(
            {
                "sensor": s,
                "observed_min_deg": obs_min,
                "observed_max_deg": obs_max,
                "observed_span_deg": obs_span,
                "live_min_deg": live_min,
                "live_max_deg": live_max,
                "live_span_deg": live_span,
                "calculated_min_deg": comp_min,
                "calculated_max_deg": comp_max,
                "calculated_span_deg": comp_span,
                "n_open_samples": len(v["open"]),
                "n_closed_samples": len(v["closed"]),
            }
        )

    return rows


def _grouped_rows(sensor_rows: list[dict]) -> list[dict]:
    by_sensor = {r["sensor"]: r for r in sensor_rows}
    out = []
    for g in RIGHT_HAND_JOINT_GROUPS:
        s = g["sensor"]
        sr = by_sensor.get(s, {})
        out.append(
            {
                "joint": g["joint"],
                "sensor": s,
                "motor": g["motor"],
                "calculated_min_deg": sr.get("calculated_min_deg", float("nan")),
                "calculated_max_deg": sr.get("calculated_max_deg", float("nan")),
                "calculated_span_deg": sr.get("calculated_span_deg", float("nan")),
                "observed_span_deg": sr.get("observed_span_deg", float("nan")),
                "live_span_deg": sr.get("live_span_deg", float("nan")),
            }
        )
    return out


def _fmt(v: float) -> str:
    return f"{v:.4f}" if _finite(v) else "nan"


def _write_sensor_csv(path: Path, rows: list[dict]):
    header = [
        "sensor",
        "observed_min_deg",
        "observed_max_deg",
        "observed_span_deg",
        "live_min_deg",
        "live_max_deg",
        "live_span_deg",
        "calculated_min_deg",
        "calculated_max_deg",
        "calculated_span_deg",
        "n_open_samples",
        "n_closed_samples",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r[k] for k in header])


def _write_grouped_csv(path: Path, rows: list[dict]):
    header = [
        "joint",
        "sensor",
        "motor",
        "calculated_min_deg",
        "calculated_max_deg",
        "calculated_span_deg",
        "observed_span_deg",
        "live_span_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([r[k] for k in header])


def _write_markdown(path: Path, sensor_rows: list[dict], grouped_rows: list[dict], json_name: str):
    lines = []
    lines.append(f"# Sensor Range Estimation from {json_name}")
    lines.append("")
    lines.append("## 1) Per-Sensor Complete Range")
    lines.append("")
    lines.append(
        "| Sensor | Observed Min (deg) | Observed Max (deg) | Observed Span (deg) | "
        "Live Min (deg) | Live Max (deg) | Live Span (deg) | "
        "Calculated Min (deg) | Calculated Max (deg) | Calculated Span (deg) |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in sensor_rows:
        lines.append(
            "| "
            + f"{r['sensor']} | {_fmt(r['observed_min_deg'])} | {_fmt(r['observed_max_deg'])} | {_fmt(r['observed_span_deg'])} | "
            + f"{_fmt(r['live_min_deg'])} | {_fmt(r['live_max_deg'])} | {_fmt(r['live_span_deg'])} | "
            + f"{_fmt(r['calculated_min_deg'])} | {_fmt(r['calculated_max_deg'])} | {_fmt(r['calculated_span_deg'])} |"
        )

    lines.append("")
    lines.append("## 2) Grouped by Joint Mapping")
    lines.append("")
    lines.append(
        "| Joint Group | Sensor | Motor | Calculated Min (deg) | Calculated Max (deg) | Calculated Span (deg) | "
        "Observed Span (deg) | Live Span (deg) |"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for r in grouped_rows:
        lines.append(
            "| "
            + f"{r['joint']} | {r['sensor']} | {r['motor']} | "
            + f"{_fmt(r['calculated_min_deg'])} | {_fmt(r['calculated_max_deg'])} | {_fmt(r['calculated_span_deg'])} | "
            + f"{_fmt(r['observed_span_deg'])} | {_fmt(r['live_span_deg'])} |"
        )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Estimate sensor complete ranges from structured calibration JSON")
    p.add_argument(
        "--json-path",
        type=str,
        default="calibration/right_structured_calibration_map.json",
        help="Path to structured calibration JSON",
    )
    p.add_argument(
        "--out-dir",
        type=str,
        default="calibration_visualizations",
        help="Directory to write reports",
    )
    return p.parse_args()


def main():
    args = parse_args()
    json_path = Path(args.json_path)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not json_path.exists():
        raise SystemExit(f"JSON file not found: {json_path}")

    cal_map = json.loads(json_path.read_text(encoding="utf-8"))

    sensor_rows = _sensor_summary_rows(cal_map)
    grouped = _grouped_rows(sensor_rows)

    stem = json_path.stem
    sensor_csv = out_dir / f"{stem}_sensor_complete_ranges.csv"
    grouped_csv = out_dir / f"{stem}_sensor_ranges_grouped_by_joint.csv"
    md_path = out_dir / f"{stem}_sensor_range_estimation.md"

    _write_sensor_csv(sensor_csv, sensor_rows)
    _write_grouped_csv(grouped_csv, grouped)
    _write_markdown(md_path, sensor_rows, grouped, json_path.name)

    print("Saved:")
    print(f"- {sensor_csv}")
    print(f"- {grouped_csv}")
    print(f"- {md_path}")

    print("\nPer-sensor calculated complete spans:")
    for r in sensor_rows:
        print(
            f"  S{r['sensor']}: min={_fmt(r['calculated_min_deg'])} "
            f"max={_fmt(r['calculated_max_deg'])} span={_fmt(r['calculated_span_deg'])}"
        )


if __name__ == "__main__":
    main()
