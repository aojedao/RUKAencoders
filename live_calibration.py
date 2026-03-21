"""live_calibration.py
=====================
Live visualization for RUKA motor/sensor calibration.

What it shows live:
1) Motor movement (ticks and estimated angle for key mapped motors)
2) Sensor readings (degrees) for all streamed sensors
3) Joint-level comparison (motor delta vs sensor delta) for right-hand mapping

This script is intended to be used while physically moving the hand so you can
see, in real time, whether motor movement and sensor movement have similar
ranges.

Example:
  python live_calibration.py --serial-port /dev/ttyACM0 --hand right

Optional:
  python live_calibration.py --serial-port /dev/ttyACM0 --hand right \
      --calibration-file calibration/right_structured_calibration_map.json
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import serial

from ruka_hand.control.hand import Hand

N_MOTORS = 16
N_SENSORS = 7

INITIAL_MOTOR_TICKS = np.array(
    [3559, 3064, 2247, 1791, 3407, 849, 2300, 1641, 600, 853, 230, 800, 2576, 2150, 2000, 1850],
    dtype=float,
)

OPEN_POS = np.array(
    [3559, 2864, 2247, 1891, 3407, 849, 3098, 1741, 1490, 853, 230, 1500, 3455, 2860, 2000, 1850],
    dtype=float,
)
INDEX_THUMB_PINCH_POS = np.array(
    [3559, 3064, 2247, 1791, 3407, 849, 2300, 1541, 600, 853, 230, 800, 2576, 2150, 2000, 1850],
    dtype=float,
)

# Joint-angle domains used by the RUKA retargeting motor mapping.
MIN_DEG = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -25, 0], dtype=float)
MAX_DEG = np.array([90, 40, 85, 15, 90, 85, 70, 20, 90, 80, 90, 90, 145, 90, 25, 60], dtype=float)

# Right-hand grouped map and expected spans (deg) from your reference table.
RIGHT_HAND_JOINT_GROUPS = [
    {"joint": "Index abduction", "sensor": 0, "motor": 8, "expected_abs_span_deg": 23.0},
    {"joint": "Index MCP", "sensor": 2, "motor": 9, "expected_abs_span_deg": 100.0},
    {"joint": "Index PIP", "sensor": 3, "motor": 7, "expected_abs_span_deg": 100.0},
    {"joint": "Index DIP", "sensor": 1, "motor": 7, "expected_abs_span_deg": 90.0},
    {"joint": "Thumb CMC", "sensor": 4, "motor": 12, "expected_abs_span_deg": 173.0},
    {"joint": "Thumb MCP", "sensor": 5, "motor": 14, "expected_abs_span_deg": 100.0},
    {"joint": "Thumb DIP", "sensor": 6, "motor": 13, "expected_abs_span_deg": 90.0},
]

# Right-hand sensor ranges from structured calibration map (deg).
SENSOR_RANGE_SEED_DEG = {
    0: (80.86, 87.28),
    1: (118.56, 229.48),
    2: (34.54, 135.70),
    3: (68.82, 90.53),
    4: (98.26, 212.78),
    5: (302.08, 313.77),
    6: (40.17, 63.63),
}


def _parse_sensor_line(line: str) -> dict[int, dict[str, float | int]] | None:
    """Parse one ESP32 line in either legacy or simplified firmware format.

    Supported examples:
      S0:123.45:1405\tS1:200.1:2275
      Sensor0:79.54\tSensor1:228.87\t...
    """
    line = line.strip()
    if not line or line.startswith("["):
        return None

    out: dict[int, dict[str, float | int]] = {}
    try:
        # Format A: S0:deg:raw
        matches_with_raw = re.findall(r"S(\d+)\s*:\s*([-+]?\d*\.?\d+)\s*:\s*(-?\d+)", line)
        for s_str, deg_str, raw_str in matches_with_raw:
            s_idx = int(s_str)
            out[s_idx] = {"deg": float(deg_str), "raw": int(raw_str)}

        # Format B: Sensor0:deg (no raw provided by firmware)
        if not out:
            matches_deg_only = re.findall(r"Sensor(\d+)\s*:\s*([-+]?\d*\.?\d+)", line)
            for s_str, deg_str in matches_deg_only:
                s_idx = int(s_str)
                out[s_idx] = {"deg": float(deg_str), "raw": -1}
    except Exception:
        return None

    return out if out else None


def _load_motor_bounds(calibration_file: str | None) -> dict[int, tuple[float, float]]:
    """Load open/closed motor ticks from structured calibration JSON."""
    if not calibration_file:
        return {}

    p = Path(calibration_file)
    if not p.exists():
        print(f"[Live] WARNING: calibration file not found: {calibration_file}")
        return {}

    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)

    out: dict[int, tuple[float, float]] = {}
    for m_str, m_data in data.get("joints", {}).items():
        try:
            m = int(m_str)
            open_tick = float(m_data["motor_max_open_ticks"])
            closed_tick = float(m_data["motor_max_closed_ticks"])
            out[m] = (open_tick, closed_tick)
        except Exception:
            continue

    return out


def motor_tick_to_angle_with_span(
    tick: float,
    open_tick: float,
    closed_tick: float,
    expected_abs_span_deg: float,
) -> float:
    """Map tick->angle using open at 0 deg and closed at expected_abs_span_deg."""
    span = closed_tick - open_tick
    if abs(span) < 1e-9:
        return float("nan")
    return (tick - open_tick) / span * expected_abs_span_deg


def normalize_to_pct(value: float, lo: float, hi: float) -> float:
    """Normalize value to [0,100] given range bounds."""
    span = hi - lo
    if not np.isfinite(value) or not np.isfinite(lo) or not np.isfinite(hi) or abs(span) < 1e-9:
        return float("nan")
    pct = (value - lo) / span * 100.0
    return float(np.clip(pct, 0.0, 100.0))


def _solve_quad_for_motor8(clamped_val: float) -> float:
    """Invert y = x*(1+0.02x) for x>=0 used by motor 8 preprocessing."""
    if not np.isfinite(clamped_val):
        return float("nan")
    disc = 1.0 + 0.08 * max(clamped_val, 0.0)
    return float((-1.0 + np.sqrt(disc)) / 0.04)


class LiveCalibrator:
    def __init__(
        self,
        serial_port: str,
        baud: int,
        hand_type: str,
        history_sec: float,
        sample_hz: float,
        plot_interval_ms: int,
        calibration_file: str | None,
        request_sensor_reads: bool,
        enable_controls: bool,
        sensor_debug: bool,
        enable_logging: bool,
        log_label: str,
        log_out_dir: str,
    ):
        self.serial_port = serial_port
        self.baud = baud
        self.hand_type = hand_type
        self.history_sec = history_sec
        self.sample_dt = 1.0 / max(sample_hz, 1.0)
        self.plot_interval_ms = max(int(plot_interval_ms), 10)
        self.bounds = _load_motor_bounds(calibration_file)
        self.request_sensor_reads = request_sensor_reads
        self.enable_controls = enable_controls
        self.sensor_debug = sensor_debug
        self.enable_logging = enable_logging
        self.log_label = log_label
        self.log_out_dir = log_out_dir

        self.conn: serial.Serial | None = None
        self.hand: Hand | None = None

        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.hand_io_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.pending_lock = threading.Lock()
        self.pending_targets: dict[int, float] = {}
        self.last_motor_error = ""

        max_samples = int(history_sec / self.sample_dt) + 10
        self.t_hist = deque(maxlen=max_samples)

        self.sensor_deg_hist = [deque(maxlen=max_samples) for _ in range(N_SENSORS)]
        self.motor_tick_hist = [deque(maxlen=max_samples) for _ in range(N_MOTORS)]

        self.latest_sensor_deg = np.full(N_SENSORS, np.nan, dtype=float)
        self.latest_sensor_raw = np.full(N_SENSORS, -1, dtype=int)
        self.latest_motor_ticks = np.full(N_MOTORS, np.nan, dtype=float)
        self.last_valid_sensor_deg = np.full(N_SENSORS, np.nan, dtype=float)
        self.last_valid_motor_ticks = np.full(N_MOTORS, np.nan, dtype=float)

        self.baseline_sensor_deg = np.full(N_SENSORS, np.nan, dtype=float)
        self.baseline_motor_ticks = np.full(N_MOTORS, np.nan, dtype=float)

        self.start_t = time.time()
        self.sensor_good_count = 0
        self.sensor_total_count = 0
        self.sensor_fail_streak = 0
        self.last_sensor_raw_line = ""
        self.last_sensor_mode = "request" if request_sensor_reads else "stream"
        self._last_debug_print_t = 0.0
        self._last_no_sensor_warn_t = 0.0
        self.controls_fig = None
        self.sliders = []
        self.slider_axes = []
        self.baseline_button = None

        self.commanded_pos = np.full(N_MOTORS, np.nan, dtype=float)
        self.t0 = 0.0
        self.run_stamp = ""
        self.unified_log: list[dict[str, object]] = []

        # Adaptive sensor normalization ranges (seeded from calibration map values).
        self.sensor_live_min = np.full(N_SENSORS, np.nan, dtype=float)
        self.sensor_live_max = np.full(N_SENSORS, np.nan, dtype=float)
        for s, (lo, hi) in SENSOR_RANGE_SEED_DEG.items():
            if 0 <= s < N_SENSORS:
                self.sensor_live_min[s] = float(lo)
                self.sensor_live_max[s] = float(hi)

    def _sanitize_command_array(self, arr: np.ndarray) -> np.ndarray:
        """Replace non-finite commands so set_pos always receives valid ticks."""
        out = np.array(arr, dtype=float)
        for i in range(N_MOTORS):
            if np.isfinite(out[i]):
                continue
            if np.isfinite(self.last_valid_motor_ticks[i]):
                out[i] = float(self.last_valid_motor_ticks[i])
            elif np.isfinite(self.latest_motor_ticks[i]):
                out[i] = float(self.latest_motor_ticks[i])
            else:
                out[i] = float(INITIAL_MOTOR_TICKS[i])
        return out

    def _estimate_joint_angle_from_tick(self, motor_idx: int, tick: float) -> float:
        """Approximate inverse tick->angle for plotted mapped motors."""
        if self.hand is None or not np.isfinite(tick):
            return float("nan")

        tensioned = np.array(self.hand.tensioned_pos, dtype=float)
        curled = np.array(self.hand.curled_bound, dtype=float)
        span_abs = abs(curled[motor_idx] - tensioned[motor_idx])

        if motor_idx == 1:
            normed = (tick - 2285.0) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 3:
            normed = (2070.0 - tick) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 7:
            normed = (tick - 2125.0) / span_abs if span_abs > 1e-9 else np.nan
        elif motor_idx == 14:
            normed = (tick - 1990.0) / span_abs if span_abs > 1e-9 else np.nan
        else:
            span = curled[motor_idx] - tensioned[motor_idx]
            normed = (tick - tensioned[motor_idx]) / span if abs(span) > 1e-9 else np.nan

        if not np.isfinite(normed):
            return float("nan")

        clamped_val = normed * (MAX_DEG[motor_idx] - MIN_DEG[motor_idx])

        # Undo motor-specific preprocessing.
        if motor_idx == 8:
            return _solve_quad_for_motor8(clamped_val)
        if motor_idx == 12:
            return clamped_val / 1.8
        if motor_idx == 13:
            return clamped_val / 1.5 + 40.0
        if motor_idx == 7:
            return clamped_val / 1.7
        if motor_idx == 3:
            return clamped_val / 1.7
        if motor_idx == 1:
            return clamped_val / 1.7

        return clamped_val

    def connect(self):
        print(f"[Live] Connecting to hand ({self.hand_type})...")
        self.hand = Hand(self.hand_type)

        print(f"[Live] Connecting serial {self.serial_port} @ {self.baud}...")
        self.conn = serial.Serial(self.serial_port, self.baud, timeout=0.2)
        time.sleep(1.5)
        self.conn.reset_input_buffer()

        # Probe firmware.
        self.conn.write(b"h")
        time.sleep(0.05)
        self.conn.reset_input_buffer()

        # Keep firmware state simple: in request mode we do not toggle stream here.
        # In streaming mode we toggle ON once.
        if not self.request_sensor_reads:
            self.conn.write(b"s")

        # Move to fixed initial pose requested for live calibration consistency.
        try:
            with self.hand_io_lock:
                self.hand.set_pos(INITIAL_MOTOR_TICKS)
            time.sleep(0.2)
        except Exception:
            pass

        # Seed commanded positions from current hand state after initial move.
        now = self._read_motor_ticks_once()
        for i in range(N_MOTORS):
            if np.isfinite(now[i]):
                self.commanded_pos[i] = now[i]
            else:
                self.commanded_pos[i] = float(INITIAL_MOTOR_TICKS[i])

        self.t0 = time.time()
        self.run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        print("[Live] Connected. Close plot window or press Ctrl+C to stop.")

    def close(self):
        try:
            if self.conn is not None and self.conn.is_open:
                # Toggle stream off on exit.
                self.conn.write(b"s")
                self.conn.close()
        except Exception:
            pass

        try:
            if self.controls_fig is not None:
                plt.close(self.controls_fig)
        except Exception:
            pass

        try:
            if self.hand is not None:
                with self.hand_io_lock:
                    self.hand.close()
        except Exception:
            pass

    def _read_sensor_once(self) -> tuple[np.ndarray, np.ndarray]:
        vals = np.full(N_SENSORS, np.nan, dtype=float)
        raws = np.full(N_SENSORS, -1, dtype=int)
        if self.conn is None:
            return vals, raws

        try:
            parsed = self._read_sensor_parsed()

            self.sensor_total_count += 1
            if parsed is None:
                self.sensor_fail_streak += 1
                # If request mode is failing repeatedly, auto-fallback to streaming mode once.
                if self.request_sensor_reads and self.sensor_fail_streak >= 20:
                    if self.sensor_debug:
                        print("[Live][Sensor] request mode failed repeatedly; switching to streaming mode", flush=True)
                    self.request_sensor_reads = False
                    self.last_sensor_mode = "stream"
                    self.sensor_fail_streak = 0
                    try:
                        self.conn.write(b"s")
                    except Exception:
                        pass

                # Always emit occasional warning if we are not receiving packets.
                now = time.time()
                if now - self._last_no_sensor_warn_t >= 1.5:
                    self._last_no_sensor_warn_t = now
                    print(
                        "[Live][SensorWARN] no parsed packet "
                        + f"(mode={self.last_sensor_mode}, fail_streak={self.sensor_fail_streak}) "
                        + f"last_raw='{self.last_sensor_raw_line[:120]}'",
                        flush=True,
                    )
                return vals, raws

            self.sensor_fail_streak = 0
            self.sensor_good_count += 1
            for s in range(N_SENSORS):
                if s in parsed:
                    vals[s] = float(parsed[s]["deg"])
                    raws[s] = int(parsed[s].get("raw", -1))

            self._maybe_print_sensor_debug(vals)
        except Exception:
            pass

        return vals, raws

    def _read_sensor_parsed(self) -> dict[int, dict[str, float | int]] | None:
        """Read one parsed sensor packet using request mode or streaming mode."""
        if self.conn is None:
            return None

        if self.request_sensor_reads:
            self.last_sensor_mode = "request"
            return self._read_sensor_request_mode()
        self.last_sensor_mode = "stream"
        return self._read_sensor_streaming_mode()

    def _maybe_print_sensor_debug(self, sensor_vals: np.ndarray):
        """Print rate-limited sensor diagnostics to terminal when debug is enabled."""
        if not self.sensor_debug:
            return

        now = time.time()
        if now - self._last_debug_print_t < 1.0:
            return
        self._last_debug_print_t = now

        shown = []
        for i in range(min(3, N_SENSORS)):
            if np.isfinite(sensor_vals[i]):
                shown.append(f"S{i}={sensor_vals[i]:.2f}")
            else:
                shown.append(f"S{i}=nan")

        ratio = 0.0
        if self.sensor_total_count > 0:
            ratio = self.sensor_good_count / self.sensor_total_count

        print(
            "[Live][SensorDBG] "
            + f"mode={self.last_sensor_mode} "
            + f"good/total={self.sensor_good_count}/{self.sensor_total_count} ({ratio:.2%}) "
            + f"raw='{self.last_sensor_raw_line[:120]}' "
            + " ".join(shown),
            flush=True,
        )

    def _read_sensor_request_mode(self) -> dict[int, dict[str, float | int]] | None:
        """Firmware-compatible on-demand read path: send 'r', then parse with retries."""
        assert self.conn is not None
        for _ in range(6):
            try:
                self.conn.write(b"r")
                # Read a few lines because firmware may emit status + sample lines.
                for _ in range(4):
                    raw_line = self.conn.readline().decode(errors="ignore")
                    self.last_sensor_raw_line = raw_line.strip()
                    parsed = _parse_sensor_line(raw_line)
                    if parsed is not None:
                        return parsed
            except Exception:
                continue
        return None

    def _read_sensor_streaming_mode(self) -> dict[int, dict[str, float | int]] | None:
        """Parse one packet from passive serial streaming."""
        assert self.conn is not None
        for _ in range(4):
            try:
                line = self.conn.readline().decode(errors="ignore")
                self.last_sensor_raw_line = line.strip()
                parsed = _parse_sensor_line(line)
                if parsed is not None:
                    return parsed
            except Exception:
                continue
        return None

    def _read_motor_ticks_once(self) -> np.ndarray:
        vals = np.full(N_MOTORS, np.nan, dtype=float)
        if self.hand is None:
            return vals

        try:
            with self.hand_io_lock:
                pos = self.hand.read_pos()
            for i in range(min(len(pos), N_MOTORS)):
                if pos[i] is not None:
                    vals[i] = float(pos[i])
        except Exception:
            pass

        return vals

    def _motor_tick_limits(self, motor_idx: int) -> tuple[float, float]:
        """Return safe command limits for one motor from calibrated hand bounds."""
        # Prefer structured-map limits for mapped joints when available.
        if motor_idx in self.bounds:
            lo = min(self.bounds[motor_idx][0], self.bounds[motor_idx][1])
            hi = max(self.bounds[motor_idx][0], self.bounds[motor_idx][1])
            # Guard against degenerate calibration ranges that produce unusable sliders.
            if hi - lo >= 50.0:
                return (float(np.clip(lo, 0.0, 4095.0)), float(np.clip(hi, 0.0, 4095.0)))

        if self.hand is not None:
            try:
                t = float(self.hand.tensioned_pos[motor_idx])
                c = float(self.hand.curled_bound[motor_idx])
                lo, hi = min(t, c), max(t, c)
                return (float(np.clip(lo, 0.0, 4095.0)), float(np.clip(hi, 0.0, 4095.0)))
            except Exception:
                pass

        return (0.0, 4095.0)

    def capture_loop(self):
        while not self.stop_event.is_set():
            t_rel = time.time() - (self.t0 if self.t0 > 0 else self.start_t)
            sensor_deg, sensor_raw = self._read_sensor_once()
            motor_ticks = self._read_motor_ticks_once()

            # Forward-fill missing values so plots stay continuous even with occasional dropouts.
            valid_s = np.isfinite(sensor_deg)
            self.last_valid_sensor_deg[valid_s] = sensor_deg[valid_s]
            sensor_deg = np.where(valid_s, sensor_deg, self.last_valid_sensor_deg)

            valid_m = np.isfinite(motor_ticks)
            self.last_valid_motor_ticks[valid_m] = motor_ticks[valid_m]
            motor_ticks = np.where(valid_m, motor_ticks, self.last_valid_motor_ticks)

            # Update adaptive sensor ranges for normalization.
            for s in range(N_SENSORS):
                v = sensor_deg[s]
                if not np.isfinite(v):
                    continue
                if not np.isfinite(self.sensor_live_min[s]) or v < self.sensor_live_min[s]:
                    self.sensor_live_min[s] = float(v)
                if not np.isfinite(self.sensor_live_max[s]) or v > self.sensor_live_max[s]:
                    self.sensor_live_max[s] = float(v)

            with self.lock:
                # Initialize baselines on first valid values.
                for s in range(N_SENSORS):
                    if np.isnan(self.baseline_sensor_deg[s]) and np.isfinite(sensor_deg[s]):
                        self.baseline_sensor_deg[s] = sensor_deg[s]
                for m in range(N_MOTORS):
                    if np.isnan(self.baseline_motor_ticks[m]) and np.isfinite(motor_ticks[m]):
                        self.baseline_motor_ticks[m] = motor_ticks[m]

                self.latest_sensor_deg = sensor_deg
                self.latest_sensor_raw = sensor_raw
                self.latest_motor_ticks = motor_ticks

                self.t_hist.append(t_rel)
                for s in range(N_SENSORS):
                    self.sensor_deg_hist[s].append(sensor_deg[s])
                for m in range(N_MOTORS):
                    self.motor_tick_hist[m].append(motor_ticks[m])

                if self.enable_logging:
                    cmd_now = self._sanitize_command_array(self.commanded_pos)
                    motor_actual = [int(v) if np.isfinite(v) else -1 for v in motor_ticks]
                    motor_cmd = [int(v) if np.isfinite(v) else -1 for v in cmd_now]
                    cmd_angles = [
                        float(self._estimate_joint_angle_from_tick(i, float(cmd_now[i])))
                        if np.isfinite(cmd_now[i])
                        else float("nan")
                        for i in range(N_MOTORS)
                    ]
                    act_angles = [
                        float(self._estimate_joint_angle_from_tick(i, float(motor_ticks[i])))
                        if np.isfinite(motor_ticks[i])
                        else float("nan")
                        for i in range(N_MOTORS)
                    ]
                    sensor_deg_row = [float(v) if np.isfinite(v) else float("nan") for v in sensor_deg]
                    sensor_raw_row = [int(v) for v in sensor_raw]
                    t_unix = float(self.t0 + t_rel) if self.t0 > 0 else float(time.time())

                    self.unified_log.append({
                        "timestamp": datetime.fromtimestamp(t_unix).isoformat(timespec="milliseconds"),
                        "t_unix": t_unix,
                        "t_rel": t_rel,
                        "commanded": motor_cmd,
                        "actual": motor_actual,
                        "cmd_angles": cmd_angles,
                        "act_angles": act_angles,
                        "raw": sensor_raw_row,
                        "deg": sensor_deg_row,
                    })

            time.sleep(self.sample_dt)

    def set_motor_target(self, motor_idx: int, target_tick: float):
        """Queue one motor target; worker executes commands off the GUI thread."""
        if self.hand is None:
            return
        if motor_idx < 0 or motor_idx >= N_MOTORS:
            return

        lo_lim, hi_lim = self._motor_tick_limits(motor_idx)
        clamped = float(np.clip(target_tick, lo_lim, hi_lim))
        with self.pending_lock:
            self.pending_targets[motor_idx] = clamped

    def motor_command_loop(self):
        """Consume pending slider commands and apply them sequentially."""
        while not self.stop_event.is_set():
            task: tuple[int, float] | None = None
            with self.pending_lock:
                if self.pending_targets:
                    # Last update wins to avoid stale intermediate slider events.
                    motor_idx = sorted(self.pending_targets.keys())[0]
                    target_tick = self.pending_targets.pop(motor_idx)
                    task = (motor_idx, target_tick)

            if task is None:
                time.sleep(0.01)
                continue

            m, t = task
            try:
                with self.motion_lock:
                    self._set_motor_target_locked(m, t)
                self.last_motor_error = ""
            except Exception as exc:
                self.last_motor_error = f"M{m}: {exc}"
                print(f"[Live][MotorERR] {self.last_motor_error}", flush=True)

    def _set_motor_target_locked(self, motor_idx: int, target_tick: float):
        if self.hand is None:
            return
        if motor_idx < 0 or motor_idx >= N_MOTORS:
            return

        # Refresh missing command entries without inverting lock order.
        if np.any(np.isnan(self.commanded_pos)):
            current = self._read_motor_ticks_once()
            with self.lock:
                for i in range(N_MOTORS):
                    if np.isfinite(current[i]):
                        self.commanded_pos[i] = current[i]

        # Snapshot current command array once; only one index is changed below.
        with self.lock:
            safe_cmd = self._sanitize_command_array(self.commanded_pos)
            start_tick = float(safe_cmd[motor_idx]) if np.isfinite(safe_cmd[motor_idx]) else float(target_tick)
            lo_lim, hi_lim = self._motor_tick_limits(motor_idx)
            target_tick = float(np.clip(target_tick, lo_lim, hi_lim))
            base_cmd = np.array(safe_cmd, dtype=float)

        n_steps = 50
        ticks = np.linspace(start_tick, target_tick, n_steps)
        for tick in ticks:
            cmd = np.array(base_cmd, dtype=float)
            cmd[motor_idx] = float(tick)
            try:
                with self.hand_io_lock:
                    self.hand.set_pos(cmd)
            except Exception as exc:
                self.last_motor_error = f"M{motor_idx}: {exc}"
                print(f"[Live][MotorERR] {self.last_motor_error}", flush=True)
                break

            with self.lock:
                self.commanded_pos[motor_idx] = float(tick)
            time.sleep(0.004)

    def reset_baseline(self):
        with self.lock:
            self.baseline_sensor_deg = self.latest_sensor_deg.copy()
            self.baseline_motor_ticks = self.latest_motor_ticks.copy()

    def run_reference_sequence(
        self,
        positions: list[np.ndarray],
        traj_len: int = 50,
        hold_sec: float = 1.0,
    ):
        """Run a replay-style position sequence repeatedly until stopped.

        The sequence positions are explicit function parameters so we can extend
        this routine later with additional waypoints and timings.
        """
        if self.hand is None:
            return

        safe_positions: list[np.ndarray] = []
        for pos in positions:
            arr = np.array(pos, dtype=float)
            if arr.shape[0] != N_MOTORS:
                continue
            safe_positions.append(np.clip(arr, 0.0, 4095.0))

        if not safe_positions:
            return

        while not self.stop_event.is_set():
            for des_pos in safe_positions:
                if self.stop_event.is_set():
                    break

                curr_pos = self._read_motor_ticks_once()
                curr_pos = self._sanitize_command_array(curr_pos)
                n_steps = max(int(traj_len), 2)
                alphas = np.linspace(0.0, 1.0, n_steps)

                for a in alphas:
                    if self.stop_event.is_set():
                        break
                    cmd = (1.0 - a) * curr_pos + a * des_pos
                    try:
                        with self.hand_io_lock:
                            self.hand.set_pos(cmd)
                    except Exception as exc:
                        self.last_motor_error = f"sequence: {exc}"
                        print(f"[Live][MotorERR] {self.last_motor_error}", flush=True)
                        break

                    with self.lock:
                        self.commanded_pos = np.array(cmd, dtype=float)
                    time.sleep(0.01)

                if hold_sec > 0:
                    time.sleep(hold_sec)

    def run_plot(self):
        fig = plt.figure(figsize=(16, 11))
        gs = fig.add_gridspec(4, 2)
        plt.subplots_adjust(bottom=0.08, top=0.93, hspace=0.35, wspace=0.20)

        # 7 joint-group panels: sensor angle and mapped motor angle on same axis.
        joint_axes = []
        joint_sensor_lines = []
        joint_motor_lines = []
        for i, g in enumerate(RIGHT_HAND_JOINT_GROUPS):
            ax = fig.add_subplot(gs[i // 2, i % 2])
            ls, = ax.plot([], [], label=f"S{g['sensor']} sensor", linewidth=1.8)
            lm, = ax.plot([], [], label=f"M{g['motor']} mapped", linewidth=1.8)
            ax.set_title(f"{g['joint']} (S{g['sensor']} / M{g['motor']})")
            ax.set_ylim(0.0, 300.0)
            ax.grid(alpha=0.3)
            ax.legend(loc="upper left", fontsize=8)
            joint_axes.append(ax)
            joint_sensor_lines.append(ls)
            joint_motor_lines.append(lm)

        # Delta comparison bars in the 8th slot.
        ax_delta = fig.add_subplot(gs[3, 1])
        joint_labels = [g["joint"] for g in RIGHT_HAND_JOINT_GROUPS]
        x = np.arange(len(joint_labels))
        match_bars = ax_delta.bar(x, np.zeros(len(x)), width=0.6, label="Delta match (%)")
        ax_delta.set_xticks(x)
        ax_delta.set_xticklabels(joint_labels, rotation=18, ha="right")
        ax_delta.set_title("Normalized Delta Match (0-100%)")
        ax_delta.set_ylabel("match %")
        ax_delta.set_ylim(0.0, 100.0)
        ax_delta.grid(axis="y", alpha=0.3)
        ax_delta.legend(loc="upper right")

        status_text = fig.text(0.01, 0.99, "", va="top", ha="left", fontsize=10)

        # Manual slider controls were intentionally removed; sequence playback
        # now provides the reference motion source.

        # Keyboard quit and window-close handlers for main figure.
        def _on_key(event):
            if event.key in ("q", "escape"):
                self.stop_event.set()
                plt.close(fig)

        def _on_close(_event):
            self.stop_event.set()

        fig.canvas.mpl_connect("key_press_event", _on_key)
        fig.canvas.mpl_connect("close_event", _on_close)

        def update(_frame):
            with self.lock:
                t = np.array(self.t_hist, dtype=float)
                sensor_hist = [np.array(h, dtype=float) for h in self.sensor_deg_hist]
                motor_hist = [np.array(h, dtype=float) for h in self.motor_tick_hist]
                latest_sensor = self.latest_sensor_deg.copy()
                latest_motor = self.latest_motor_ticks.copy()
                base_sensor = self.baseline_sensor_deg.copy()
                base_motor = self.baseline_motor_ticks.copy()

            if t.size == 0:
                return []

            # Joint-group traces (sensor deg and mapped motor deg on same axis).
            finite_counts = []
            for i, g in enumerate(RIGHT_HAND_JOINT_GROUPS):
                s = g["sensor"]
                motor_id = g["motor"]
                m = motor_id - 1
                expected_span = g["expected_abs_span_deg"]

                y_sensor = sensor_hist[s]
                y_motor = np.full_like(t, np.nan, dtype=float)

                if 0 <= m < N_MOTORS:
                    mt = motor_hist[m]
                    valid = np.isfinite(mt)
                    if np.any(valid):
                        y_motor[valid] = np.array(
                            [self._estimate_joint_angle_from_tick(m, vv) for vv in mt[valid]],
                            dtype=float,
                        )

                mask_s = np.isfinite(y_sensor)
                mask_m = np.isfinite(y_motor)
                joint_sensor_lines[i].set_data(t[mask_s], y_sensor[mask_s])
                joint_motor_lines[i].set_data(t[mask_m], y_motor[mask_m])
                finite_counts.append(int(np.count_nonzero(mask_s)))
                joint_axes[i].set_xlim(max(0.0, t[-1] - self.history_sec), t[-1] + 0.05)

                # Dynamic per-joint y-range so small sensor movement is visible.
                local_vals = np.concatenate([
                    y_sensor[mask_s] if np.any(mask_s) else np.array([], dtype=float),
                    y_motor[mask_m] if np.any(mask_m) else np.array([], dtype=float),
                ])
                if local_vals.size > 0:
                    lo = float(np.min(local_vals))
                    hi = float(np.max(local_vals))
                    span = max(hi - lo, 20.0)
                    pad = 0.10 * span
                    y_lo = lo - pad
                    y_hi = hi + pad
                    if y_hi - y_lo < 10.0:
                        mid = 0.5 * (y_hi + y_lo)
                        y_lo = mid - 5.0
                        y_hi = mid + 5.0
                    joint_axes[i].set_ylim(y_lo, y_hi)

            # Normalized position bars (0-100) based on real ranges.
            match_scores = []
            sensor_pcts = []
            motor_pcts = []
            for g in RIGHT_HAND_JOINT_GROUPS:
                s = g["sensor"]
                motor_id = g["motor"]
                m = motor_id - 1
                sensor_pct = np.nan
                motor_pct = np.nan

                # Sensor position % from adaptive live min/max range.
                smin = self.sensor_live_min[s] if 0 <= s < N_SENSORS else np.nan
                smax = self.sensor_live_max[s] if 0 <= s < N_SENSORS else np.nan
                sensor_pct = normalize_to_pct(latest_sensor[s], smin, smax)

                # Motor position % from open/closed tick bounds.
                if (motor_id in self.bounds) and (0 <= m < N_MOTORS):
                    open_tick, closed_tick = self.bounds[motor_id]
                    motor_pct = normalize_to_pct(latest_motor[m], open_tick, closed_tick)

                if np.isfinite(sensor_pct) and np.isfinite(motor_pct):
                    # 100% means sensor and motor normalized positions match perfectly.
                    score = float(np.clip(100.0 - abs(sensor_pct - motor_pct), 0.0, 100.0))
                else:
                    score = 0.0

                sensor_pcts.append(sensor_pct)
                motor_pcts.append(motor_pct)
                match_scores.append(score)

            for i, b in enumerate(match_bars):
                b.set_height(match_scores[i])

            status_text.set_text(
                "Live status | "
                f"t={t[-1]:.1f}s | "
                + f"mode={self.last_sensor_mode} | "
                + f"sensor packets={self.sensor_good_count}/{self.sensor_total_count} | "
                + f"finite_pts={finite_counts[:3]}... | "
                + f"match_avg={np.nanmean(match_scores) if len(match_scores) else 0.0:.1f}% (range-norm) | "
                + (f"motor_err='{self.last_motor_error[:48]}' | " if self.last_motor_error else "")
                + (
                    f"raw='{self.last_sensor_raw_line[:40].replace(chr(9), ' ')}' | "
                    if self.sensor_debug
                    else ""
                )
                + "press q/esc to quit"
            )

            artists = list(joint_sensor_lines) + list(joint_motor_lines) + list(match_bars) + [status_text]
            return artists

        anim = FuncAnimation(
            fig,
            update,
            interval=self.plot_interval_ms,
            blit=False,
            cache_frame_data=False,
        )
        _ = anim

        plt.show()

        self.stop_event.set()

    def save_logs(self):
        """Save one unified replay-style log with motor and sensor fields."""
        if not self.enable_logging:
            return

        os.makedirs(self.log_out_dir, exist_ok=True)
        self._save_unified_log()

    def _save_unified_log(self):
        n = len(self.unified_log)
        if n == 0:
            print("[Save] Unified log is empty - nothing saved.", flush=True)
            return

        stamp = self.run_stamp if self.run_stamp else datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.log_out_dir, f"{self.log_label}_{stamp}_unified_log.csv")
        cmd_cols = [f"cmd_{i}" for i in range(N_MOTORS)]
        act_cols = [f"act_{i}" for i in range(N_MOTORS)]
        cmd_ang_cols = [f"cmd_angle_{i}" for i in range(N_MOTORS)]
        act_ang_cols = [f"act_angle_{i}" for i in range(N_MOTORS)]
        raw_cols = [f"raw_{i}" for i in range(N_SENSORS)]
        deg_cols = [f"deg_{i}" for i in range(N_SENSORS)]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["timestamp", "t_unix", "t_rel"]
                + cmd_cols
                + act_cols
                + cmd_ang_cols
                + act_ang_cols
                + raw_cols
                + deg_cols
            )
            for row in self.unified_log:
                writer.writerow(
                    [row["timestamp"], row["t_unix"], row["t_rel"]]
                    + row["commanded"]
                    + row["actual"]
                    + row["cmd_angles"]
                    + row["act_angles"]
                    + row["raw"]
                    + row["deg"]
                )

        print(f"[Save] Unified log -> {csv_path} ({n} rows)", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live motor/sensor calibration viewer")
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0", help="ESP32 serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud")
    parser.add_argument("--hand", type=str, default="right", choices=["left", "right"], help="Hand type")
    parser.add_argument("--history-sec", type=float, default=15.0, help="Visible history window in seconds")
    parser.add_argument("--sample-hz", type=float, default=60.0, help="Sampling frequency")
    parser.add_argument(
        "--plot-interval-ms",
        type=int,
        default=35,
        help="Matplotlib refresh interval in milliseconds (lower is faster).",
    )
    parser.add_argument(
        "--calibration-file",
        type=str,
        default="calibration/right_structured_calibration_map.json",
        help="Structured calibration map (used for motor tick->angle span mapping)",
    )
    parser.add_argument(
        "--streaming-read",
        action="store_true",
        help="Use passive streaming reads instead of explicit 'r' request reads.",
    )
    parser.add_argument(
        "--no-sequence",
        action="store_true",
        help="Disable automatic replay-style reference sequence.",
    )
    parser.add_argument(
        "--sequence-traj-len",
        type=int,
        default=50,
        help="Interpolation steps per sequence move.",
    )
    parser.add_argument(
        "--sequence-hold-sec",
        type=float,
        default=1.0,
        help="Hold time at each sequence waypoint.",
    )
    parser.add_argument(
        "--sensor-debug",
        action="store_true",
        help="Enable sensor terminal debug prints and raw status snippet.",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Disable replay-style motor/sensor logging.",
    )
    parser.add_argument(
        "--label",
        type=str,
        default="live_seq",
        help="Log file prefix label.",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        default="encoder_logs",
        help="Directory for output logs.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    viewer = LiveCalibrator(
        serial_port=args.serial_port,
        baud=args.baud,
        hand_type=args.hand,
        history_sec=args.history_sec,
        sample_hz=args.sample_hz,
        plot_interval_ms=args.plot_interval_ms,
        calibration_file=args.calibration_file,
        request_sensor_reads=not args.streaming_read,
        enable_controls=False,
        sensor_debug=args.sensor_debug,
        enable_logging=not args.no_log,
        log_label=args.label,
        log_out_dir=args.out_dir,
    )

    try:
        viewer.connect()
        thread = threading.Thread(target=viewer.capture_loop, daemon=True)
        thread.start()

        seq_thread = None
        if not args.no_sequence:
            seq_thread = threading.Thread(
                target=viewer.run_reference_sequence,
                kwargs={
                    "positions": [OPEN_POS, INDEX_THUMB_PINCH_POS],
                    "traj_len": args.sequence_traj_len,
                    "hold_sec": args.sequence_hold_sec,
                },
                daemon=True,
            )
            seq_thread.start()

        viewer.run_plot()
        thread.join(timeout=1.0)
        if seq_thread is not None:
            seq_thread.join(timeout=1.0)
    except KeyboardInterrupt:
        print("\n[Live] Interrupted by user")
        viewer.stop_event.set()
    finally:
        viewer.close()
        viewer.save_logs()
        print("[Live] Closed.")


if __name__ == "__main__":
    main()
