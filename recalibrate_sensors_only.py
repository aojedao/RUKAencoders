"""recalibrate_sensors_only.py
================================
Interactive sensor-only recalibration for structured calibration JSON files.

This script updates ONLY sensor-related fields in the target JSON:
- fully_open_sensors
- joints.<motor_id>.sensor_readings_when_joint_closed
- sensor_live_limits_deg

Motor tick fields are preserved untouched.

Typical use:
  python recalibrate_sensors_only.py \
      --json-path calibration/right_structured_calibration_map.json \
      --serial-port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import select
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

try:
    from ruka_hand.control.hand import Hand
except ImportError:
    Hand = None


RIGHT_HAND_JOINT_GROUPS = [
    {"joint": "Index abduction", "sensor": 0, "motor": 8},
    {"joint": "Index MCP", "sensor": 3, "motor": 9},
    {"joint": "Index PIP", "sensor": 2, "motor": 7},
    {"joint": "Index DIP", "sensor": 1, "motor": 7},
    {"joint": "Thumb CMC", "sensor": 6, "motor": 12},
    {"joint": "Thumb MCP", "sensor": 5, "motor": 14},
    {"joint": "Thumb DIP", "sensor": 4, "motor": 13},
]


def _quantile(sorted_vals: list[float], q: float) -> float:
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return float(sorted_vals[0])
    q = min(max(q, 0.0), 1.0)
    idx = (len(sorted_vals) - 1) * q
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    t = idx - lo
    return float(sorted_vals[lo] * (1.0 - t) + sorted_vals[hi] * t)


def _filter_by_iqr(samples: list[tuple[float, int]], iqr_k: float, min_keep: int) -> list[tuple[float, int]]:
    """Reject outliers using IQR bounds on degree values.

    Falls back to original samples if filtering would leave too few points.
    """
    if len(samples) < max(min_keep, 5):
        return samples

    degs_sorted = sorted(v[0] for v in samples)
    q1 = _quantile(degs_sorted, 0.25)
    q3 = _quantile(degs_sorted, 0.75)
    iqr = q3 - q1
    if not math.isfinite(iqr) or iqr <= 0.0:
        return samples

    lo = q1 - iqr_k * iqr
    hi = q3 + iqr_k * iqr
    filtered = [p for p in samples if lo <= p[0] <= hi]
    return filtered if len(filtered) >= min_keep else samples


def _parse_sensor_line(line: str) -> dict[int, dict[str, float | int]] | None:
    """Parse one ESP32 line in either format:
    - S0:123.45:1405\tS1:...
    - Sensor0:79.54\tSensor1:...
    """
    line = line.strip()
    if not line or line.startswith("["):
        return None

    out: dict[int, dict[str, float | int]] = {}

    # Format A: S#:deg:raw
    matches_with_raw = re.findall(r"S(\d+)\s*:\s*([-+]?\d*\.?\d+)\s*:\s*(-?\d+)", line)
    for s_str, deg_str, raw_str in matches_with_raw:
        s_idx = int(s_str)
        out[s_idx] = {"deg": float(deg_str), "raw": int(raw_str)}

    if out:
        return out

    # Format B: Sensor#:deg
    matches_deg_only = re.findall(r"Sensor(\d+)\s*:\s*([-+]?\d*\.?\d+)", line)
    for s_str, deg_str in matches_deg_only:
        s_idx = int(s_str)
        out[s_idx] = {"deg": float(deg_str), "raw": -1}

    return out if out else None


class SensorReader:
    def __init__(self, port: str, baud: int):
        self.conn = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.0)
        self.conn.reset_input_buffer()

        # Probe firmware and keep request mode deterministic.
        self.conn.write(b"h")
        time.sleep(0.05)
        self.conn.reset_input_buffer()

    def close(self):
        try:
            if self.conn and self.conn.is_open:
                self.conn.close()
        except Exception:
            pass

    def read_packet(self) -> dict[int, dict[str, float | int]] | None:
        """Request one packet using 'r' reads with retries."""
        for _ in range(5):
            try:
                self.conn.write(b"r")
                for _ in range(4):
                    line = self.conn.readline().decode(errors="ignore")
                    parsed = _parse_sensor_line(line)
                    if parsed is not None:
                        return parsed
            except Exception:
                continue
        return None


def _stdin_enter_pressed() -> bool:
    """Return True if user pressed Enter in terminal."""
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not rlist:
        return False
    _ = sys.stdin.readline()
    return True


def _collect_snapshot(
    reader: SensorReader,
    sensor_ids: list[int],
    title: str,
    iqr_k: float,
    min_keep: int,
) -> dict[str, dict[str, float | int]]:
    print("\n" + "=" * 72)
    print(title)
    print("Move to target pose, then press Enter to capture median snapshot.")
    print("Sampling now...")

    samples: dict[int, list[tuple[float, int]]] = {s: [] for s in sensor_ids}
    total_packets = 0

    while True:
        packet = reader.read_packet()
        if packet:
            total_packets += 1
            for s in sensor_ids:
                if s in packet:
                    deg = float(packet[s]["deg"])
                    raw = int(packet[s].get("raw", -1))
                    samples[s].append((deg, raw))

            if total_packets % 20 == 0:
                preview = []
                for s in sensor_ids[:3]:
                    if samples[s]:
                        preview.append(f"S{s}={samples[s][-1][0]:.2f}")
                if preview:
                    print("  latest:", " ".join(preview))

        if _stdin_enter_pressed() and total_packets > 0:
            break

    out: dict[str, dict[str, float | int]] = {}
    for s in sensor_ids:
        vals = samples[s]
        if not vals:
            continue

        filt = _filter_by_iqr(vals, iqr_k=iqr_k, min_keep=min_keep)
        degs = sorted(v[0] for v in filt)
        raws = sorted(v[1] for v in filt)
        mid = len(filt) // 2
        deg_med = float(degs[mid])
        raw_med = int(raws[mid])
        out[str(s)] = {"deg": round(deg_med, 4), "raw": raw_med}

    print(f"Captured {total_packets} packets.")
    return out


def _collect_global_limits(
    reader: SensorReader,
    sensor_ids: list[int],
    low_pct: float,
    high_pct: float,
    iqr_k: float,
    min_keep: int,
    hand=None,
    out_dir="encoder_logs",
) -> dict[str, dict[str, float]]:
    print("\n" + "=" * 72)
    print("Global sensor limits capture")
    print("Now move each joint/sensor through its full range repeatedly.")
    print("Press Enter when done to save min/max degrees.")

    deg_samples: dict[int, list[float]] = {s: [] for s in sensor_ids}
    total_packets = 0
    t0 = time.time()
    trace_log = []

    while True:
        packet = reader.read_packet()
        if packet:
            t_rel = time.time() - t0
            motor_ticks = [-1] * 16
            if hand is not None:
                try:
                    pos = hand.read_pos()
                    if pos:
                        motor_ticks = [int(p) if p is not None else -1 for p in pos[:16]]
                except Exception:
                    pass

            total_packets += 1
            raw_vals = [-1] * 7
            deg_vals = [float("nan")] * 7

            for s in sensor_ids:
                if s in packet:
                    d = float(packet[s]["deg"])
                    r = int(packet[s].get("raw", -1))
                    if s < 7:
                        raw_vals[s] = r
                        deg_vals[s] = d
                    if math.isfinite(d):
                        deg_samples[s].append(d)

            trace_log.append({
                "t_rel": t_rel,
                "raw": raw_vals,
                "deg": deg_vals,
                "actual": motor_ticks
            })

            if total_packets % 30 == 0:
                preview = []
                for s in sensor_ids[:3]:
                    vals = deg_samples[s]
                    if vals:
                        lo = min(vals)
                        hi = max(vals)
                        preview.append(f"S{s}[{lo:.1f},{hi:.1f}]")
                if preview:
                    print("  span:", " ".join(preview))

        if _stdin_enter_pressed() and total_packets > 0:
            break

    out: dict[str, dict[str, float]] = {}
    for s in sensor_ids:
        vals = deg_samples[s]
        if not vals:
            continue

        paired = [(v, -1) for v in vals]
        filt = _filter_by_iqr(paired, iqr_k=iqr_k, min_keep=min_keep)
        vals_sorted = sorted(v[0] for v in filt)

        if not vals_sorted:
            continue

        lo = _quantile(vals_sorted, low_pct / 100.0)
        hi = _quantile(vals_sorted, high_pct / 100.0)
        out[str(s)] = {
            "min_deg": round(float(lo), 4),
            "max_deg": round(float(hi), 4),
        }

    print(f"Captured {total_packets} packets for global limits.")

    if trace_log:
        Path(out_dir).mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = Path(out_dir) / f"{stamp}_calibration_limits_trace.csv"
        with csv_path.open("w", newline="") as f:
            w = csv.writer(f)
            raw_cols = [f"raw_{i}" for i in range(7)]
            deg_cols = [f"deg_{i}" for i in range(7)]
            act_cols = [f"act_{i}" for i in range(16)]
            w.writerow(["t_rel"] + raw_cols + deg_cols + act_cols)
            for row in trace_log:
                w.writerow([row["t_rel"]] + row["raw"] + [f"{d:.2f}" for d in row["deg"]] + row["actual"])
        print(f"Saved live calibration trace log to: {csv_path}")

    return out


def _collect_sensor_closed_targets(
    reader: SensorReader,
    sensor_ids: list[int],
    iqr_k: float,
    min_keep: int,
) -> dict[str, dict[str, float | int]]:
    """Capture one closed target reading per sensor (sensor-by-sensor workflow)."""
    out: dict[str, dict[str, float | int]] = {}

    for s in sensor_ids:
        print("\n" + "=" * 72)
        print(f"Sensor S{s} closed target")
        print(f"Move ONLY sensor S{s} to its fully closed physical limit, then press Enter.")

        snap = _collect_snapshot(
            reader,
            sensor_ids,
            f"Capturing closed target for S{s}",
            iqr_k=iqr_k,
            min_keep=min_keep,
        )

        if str(s) in snap:
            out[str(s)] = snap[str(s)]

    return out


def _apply_sensor_closed_targets_to_joints(
    joints: dict,
    sensor_targets: dict[str, dict[str, float | int]],
) -> dict[str, dict]:
    """Update per-joint closed snapshots for captured sensors, preserve others."""
    updated_joints = json.loads(json.dumps(joints))

    for j_key, j_val in updated_joints.items():
        closed = j_val.get("sensor_readings_when_joint_closed", {})
        for s_key, target in sensor_targets.items():
            if s_key in closed:
                closed[s_key] = {
                    "deg": float(target.get("deg", float("nan"))),
                    "raw": int(target.get("raw", -1)),
                }
        j_val["sensor_readings_when_joint_closed"] = closed

    return updated_joints


def _collect_sensor_values_for_estimation(cal_map: dict) -> dict[int, dict[str, list[float]]]:
    vals: dict[int, dict[str, list[float]]] = {}

    def ensure(s: int):
        if s not in vals:
            vals[s] = {"open": [], "closed": [], "live_min": [], "live_max": []}

    for s_key, rec in cal_map.get("fully_open_sensors", {}).items():
        s = int(s_key)
        ensure(s)
        d = rec.get("deg", float("nan"))
        if isinstance(d, (int, float)) and math.isfinite(float(d)):
            vals[s]["open"].append(float(d))

    for _, j in cal_map.get("joints", {}).items():
        closed = j.get("sensor_readings_when_joint_closed", {})
        for s_key, rec in closed.items():
            s = int(s_key)
            ensure(s)
            d = rec.get("deg", float("nan"))
            if isinstance(d, (int, float)) and math.isfinite(float(d)):
                vals[s]["closed"].append(float(d))

    for s_key, rec in cal_map.get("sensor_live_limits_deg", {}).items():
        s = int(s_key)
        ensure(s)
        lo = rec.get("min_deg", float("nan"))
        hi = rec.get("max_deg", float("nan"))
        if isinstance(lo, (int, float)) and math.isfinite(float(lo)):
            vals[s]["live_min"].append(float(lo))
        if isinstance(hi, (int, float)) and math.isfinite(float(hi)):
            vals[s]["live_max"].append(float(hi))

    return vals


def _estimate_sensor_rows(cal_map: dict) -> list[dict]:
    vals = _collect_sensor_values_for_estimation(cal_map)
    rows: list[dict] = []

    for s in sorted(vals.keys()):
        v = vals[s]
        observed = v["open"] + v["closed"]
        live = v["live_min"] + v["live_max"]
        complete = observed + live

        obs_min = min(observed) if observed else float("nan")
        obs_max = max(observed) if observed else float("nan")
        obs_span = (obs_max - obs_min) if (math.isfinite(obs_min) and math.isfinite(obs_max)) else float("nan")

        live_min = min(v["live_min"]) if v["live_min"] else float("nan")
        live_max = max(v["live_max"]) if v["live_max"] else float("nan")
        live_span = (live_max - live_min) if (math.isfinite(live_min) and math.isfinite(live_max)) else float("nan")

        comp_min = min(complete) if complete else float("nan")
        comp_max = max(complete) if complete else float("nan")
        comp_span = (comp_max - comp_min) if (math.isfinite(comp_min) and math.isfinite(comp_max)) else float("nan")

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
            }
        )

    return rows


def _group_rows_by_joint(sensor_rows: list[dict]) -> list[dict]:
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
    return f"{v:.4f}" if isinstance(v, (int, float)) and math.isfinite(float(v)) else "nan"


def _write_estimation_reports(cal_map: dict, json_path: Path, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = json_path.stem

    sensor_rows = _estimate_sensor_rows(cal_map)
    grouped_rows = _group_rows_by_joint(sensor_rows)

    sensor_csv = out_dir / f"{stem}_sensor_complete_ranges.csv"
    grouped_csv = out_dir / f"{stem}_sensor_ranges_grouped_by_joint.csv"
    md_path = out_dir / f"{stem}_sensor_range_estimation.md"

    with sensor_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
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
        ])
        for r in sensor_rows:
            w.writerow([
                r["sensor"],
                r["observed_min_deg"],
                r["observed_max_deg"],
                r["observed_span_deg"],
                r["live_min_deg"],
                r["live_max_deg"],
                r["live_span_deg"],
                r["calculated_min_deg"],
                r["calculated_max_deg"],
                r["calculated_span_deg"],
            ])

    with grouped_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "joint",
            "sensor",
            "motor",
            "calculated_min_deg",
            "calculated_max_deg",
            "calculated_span_deg",
            "observed_span_deg",
            "live_span_deg",
        ])
        for r in grouped_rows:
            w.writerow([
                r["joint"],
                r["sensor"],
                r["motor"],
                r["calculated_min_deg"],
                r["calculated_max_deg"],
                r["calculated_span_deg"],
                r["observed_span_deg"],
                r["live_span_deg"],
            ])

    lines = []
    lines.append(f"# Sensor Range Estimation from {json_path.name}")
    lines.append("")
    lines.append("## 1) Per-Sensor Complete Range")
    lines.append("")
    lines.append(
        "| Sensor | Observed Min (deg) | Observed Max (deg) | Observed Span (deg) | Live Min (deg) | Live Max (deg) | Live Span (deg) | Calculated Min (deg) | Calculated Max (deg) | Calculated Span (deg) |"
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
        "| Joint Group | Sensor | Motor | Calculated Min (deg) | Calculated Max (deg) | Calculated Span (deg) | Observed Span (deg) | Live Span (deg) |"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|---:|")
    for r in grouped_rows:
        lines.append(
            "| "
            + f"{r['joint']} | {r['sensor']} | {r['motor']} | {_fmt(r['calculated_min_deg'])} | {_fmt(r['calculated_max_deg'])} | {_fmt(r['calculated_span_deg'])} | {_fmt(r['observed_span_deg'])} | {_fmt(r['live_span_deg'])} |"
        )

    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("\nSaved range estimation reports:")
    print(f"- {sensor_csv}")
    print(f"- {grouped_csv}")
    print(f"- {md_path}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive sensor-only recalibration for structured JSON")
    p.add_argument("--json-path", type=str, default="calibration/right_structured_calibration_map.json")
    p.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--skip-open", action="store_true", help="Do not recapture fully_open_sensors")
    p.add_argument("--skip-closed", action="store_true", help="Do not recapture per-joint closed snapshots")
    p.add_argument("--skip-limits", action="store_true", help="Do not recapture sensor_live_limits_deg")
    p.add_argument("--no-backup", action="store_true", help="Disable timestamped backup before overwrite")
    p.add_argument(
        "--outlier-iqr-k",
        type=float,
        default=1.5,
        help="IQR multiplier for outlier rejection (default: 1.5).",
    )
    p.add_argument(
        "--min-filtered-samples",
        type=int,
        default=8,
        help="Minimum kept samples after filtering before fallback to original data.",
    )
    p.add_argument(
        "--limit-low-percentile",
        type=float,
        default=2.0,
        help="Lower percentile for global min_deg estimate (default: 2).",
    )
    p.add_argument(
        "--limit-high-percentile",
        type=float,
        default=98.0,
        help="Upper percentile for global max_deg estimate (default: 98).",
    )
    p.add_argument(
        "--no-estimate-report",
        action="store_true",
        help="Skip generating complete/grouped sensor range estimation reports.",
    )
    p.add_argument(
        "--estimate-out-dir",
        type=str,
        default="calibration_visualizations",
        help="Output directory for generated sensor range estimation reports.",
    )
    p.add_argument(
        "--hand",
        type=str,
        default="none",
        choices=["left", "right", "none"],
        help="Connect to Hand to also log dynamixel motor positions during limit capture.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    json_path = Path(args.json_path)
    if not json_path.exists():
        raise SystemExit(f"JSON file not found: {json_path}")

    data = json.loads(json_path.read_text(encoding="utf-8"))

    # Infer sensor IDs from current JSON to avoid hard-coding 0..6.
    sensor_ids = set()
    sensor_ids.update(int(k) for k in data.get("fully_open_sensors", {}).keys())
    for _, j in data.get("joints", {}).items():
        sensor_ids.update(int(k) for k in j.get("sensor_readings_when_joint_closed", {}).keys())
    if not sensor_ids:
        sensor_ids = set(range(7))
    sensor_ids = sorted(sensor_ids)

    print(f"Using sensor IDs: {sensor_ids}")

    hand_conn = None
    if args.hand in ["left", "right"]:
        if Hand is None:
            print("WARNING: 'ruka_hand.control.hand' not installed, motor logging disabled.")
        else:
            try:
                print(f"Connecting to {args.hand} hand for motor logging...")
                hand_conn = Hand(args.hand)
            except Exception as e:
                print(f"Failed to connect to hand: {e}")

    reader = SensorReader(args.serial_port, args.baud)
    try:
        if not args.skip_open:
            data["fully_open_sensors"] = _collect_snapshot(
                reader,
                sensor_ids,
                "Fully open snapshot\nMove hand to fully open pose.",
                iqr_k=args.outlier_iqr_k,
                min_keep=max(3, args.min_filtered_samples),
            )

        if not args.skip_closed:
            sensor_targets = _collect_sensor_closed_targets(
                reader,
                sensor_ids,
                iqr_k=args.outlier_iqr_k,
                min_keep=max(3, args.min_filtered_samples),
            )
            data["joints"] = _apply_sensor_closed_targets_to_joints(data.get("joints", {}), sensor_targets)

        if not args.skip_limits:
            data["sensor_live_limits_deg"] = _collect_global_limits(
                reader,
                sensor_ids,
                low_pct=args.limit_low_percentile,
                high_pct=args.limit_high_percentile,
                iqr_k=args.outlier_iqr_k,
                min_keep=max(3, args.min_filtered_samples),
                hand=hand_conn,
            )

    finally:
        reader.close()
        if hand_conn is not None:
            try:
                hand_conn.close()
            except Exception:
                pass

    if not args.no_backup:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = json_path.with_name(f"{json_path.stem}.{stamp}.bak{json_path.suffix}")
        backup.write_text(json.dumps(json.loads(json_path.read_text(encoding="utf-8")), indent=2), encoding="utf-8")
        print(f"Backup saved: {backup}")

    json_path.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"Updated sensor fields written to: {json_path}")

    if not args.no_estimate_report:
        _write_estimation_reports(data, json_path, Path(args.estimate_out_dir))


if __name__ == "__main__":
    main()
