"""calibrate_sensors.py
=======================
Calibrates both the RUKA Dynamixel motor limits (curled / tensioned) AND the
AS5600 magnetic sensor limits (raw 0-4095 values at those same extremes, plus
the absolute min/max the sensors reach across the full sweep).

Mirrors the structure of calibrate_motors.py but opens an additional serial
connection to the ESP32 running AS5600_Mux_Calibration.ino to record sensor
readings at every calibration point.

Saved files (in <repo_root>/curl_limits/):
  {hand_type}_curl_limits.npy         — motor encoder ticks at curled bound
  {hand_type}_tension_limits.npy      — motor encoder ticks at tensioned pos
  {hand_type}_sensor_curl_raw.npy     — AS5600 raw (0-4095) at curled bound, per sensor
  {hand_type}_sensor_tension_raw.npy  — AS5600 raw (0-4095) at tensioned pos, per sensor
  {hand_type}_sensor_sweep_min.npy    — absolute min AS5600 raw over the full motor sweep
  {hand_type}_sensor_sweep_max.npy    — absolute max AS5600 raw over the full motor sweep

Sensor-to-motor mapping
------------------------
Pass --sensor-map as a JSON string that maps sensor index (str) → motor_id (int):
  --sensor-map '{"0": 1, "1": 2, "2": 3}'
If a motor has no sensor mapped to it, motor calibration still runs but no
sensor value is recorded for that motor (None / NaN in the output).

Usage examples
--------------
  # Full calibration, both curl and tension passes
  python calibrate_sensors.py --hand-type right --serial-port /dev/ttyUSB2 \
      --sensor-map '{"0": 1, "1": 2, "2": 3}'

  # Only the curl pass
  python calibrate_sensors.py --hand-type right --serial-port /dev/ttyUSB2 --mode curl

  # Only the tension + sensor-range pass (curl file must already exist)
  python calibrate_sensors.py --hand-type right --serial-port /dev/ttyUSB2 --mode tension
"""

import argparse
import json
import os
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np
import serial

from ruka_hand.control.hand import Hand
from ruka_hand.utils.file_ops import get_repo_root

# ── Terminal helper (same as calibrate_motors.py) ────────────────────────────

def get_key() -> str:
    """Capture a single key press, including arrow escape sequences."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Serial helpers ────────────────────────────────────────────────────────────

NUMBER_OF_SENSORS = 3  # must match firmware

# Right-hand grouped joints used for span-based delta validation.
# Angles are interpreted with open position at 0 deg, closed at expected_abs_span_deg.
RIGHT_HAND_JOINT_GROUPS = [
    {"joint": "Index abduction", "sensor": 0, "motor": 8, "expected_abs_span_deg": 23.0},
    {"joint": "Index MCP", "sensor": 3, "motor": 9, "expected_abs_span_deg": 100.0},
    {"joint": "Index PIP", "sensor": 2, "motor": 7, "expected_abs_span_deg": 100.0},
    {"joint": "Index DIP", "sensor": 1, "motor": 7, "expected_abs_span_deg": 90.0},
    {"joint": "Thumb CMC", "sensor": 6, "motor": 12, "expected_abs_span_deg": 173.0},
    {"joint": "Thumb MCP", "sensor": 5, "motor": 14, "expected_abs_span_deg": 100.0},
    {"joint": "Thumb DIP", "sensor": 4, "motor": 13, "expected_abs_span_deg": 90.0},
]


def motor_tick_to_angle_with_span(
    tick: float,
    motor_open_tick: float,
    motor_closed_tick: float,
    expected_abs_span_deg: float,
) -> float:
    """Convert motor tick to angle using expected span, with open anchored at 0 deg."""
    if not np.isfinite(tick) or not np.isfinite(motor_open_tick) or not np.isfinite(motor_closed_tick):
        return float("nan")

    tick_span = motor_closed_tick - motor_open_tick
    if abs(tick_span) < 1e-9:
        return 0.0

    ratio = (tick - motor_open_tick) / tick_span
    return float(ratio * expected_abs_span_deg)


def _collect_sensor_global_ranges(structured_map: dict) -> dict:
    """Return per-sensor min/max degrees across open + all closed snapshots."""
    fully_open = structured_map.get("fully_open_sensors", {})
    joints = structured_map.get("joints", {})

    sensor_ids = set(int(k) for k in fully_open.keys())
    for motor_data in joints.values():
        closed = motor_data.get("sensor_readings_when_joint_closed", {})
        sensor_ids.update(int(k) for k in closed.keys())

    out = {}
    for sensor_id in sorted(sensor_ids):
        sid = str(sensor_id)
        vals = []
        if sid in fully_open and "deg" in fully_open[sid]:
            vals.append(float(fully_open[sid]["deg"]))
        for motor_data in joints.values():
            closed = motor_data.get("sensor_readings_when_joint_closed", {})
            if sid in closed and "deg" in closed[sid]:
                vals.append(float(closed[sid]["deg"]))

        vals = [v for v in vals if np.isfinite(v)]
        if vals:
            out[sensor_id] = {
                "min_deg": float(np.min(vals)),
                "max_deg": float(np.max(vals)),
                "span_deg": float(np.max(vals) - np.min(vals)),
            }
        else:
            out[sensor_id] = {"min_deg": float("nan"), "max_deg": float("nan"), "span_deg": float("nan")}

    return out


def analyze_structured_map_deltas(calibration_file: str, output_dir: str):
    """Analyze right-hand calibration map with span-corrected motor tick->angle conversion."""
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError:
        print("[Analysis] matplotlib is required for plotting. Install: pip install matplotlib", flush=True)
        return

    with open(calibration_file, "r") as f:
        structured_map = json.load(f)

    hand_type = structured_map.get("hand_type", "unknown")
    if hand_type != "right":
        print(
            f"[Analysis] WARNING: this analysis uses right-hand grouped spans; loaded hand_type={hand_type}",
            flush=True,
        )

    os.makedirs(output_dir, exist_ok=True)
    base_name = Path(calibration_file).stem

    fully_open = structured_map.get("fully_open_sensors", {})
    joints = structured_map.get("joints", {})
    sensor_global = _collect_sensor_global_ranges(structured_map)

    rows = []
    for g in RIGHT_HAND_JOINT_GROUPS:
        joint_name = g["joint"]
        sensor_id = g["sensor"]
        motor_id = g["motor"]
        expected_span = float(g["expected_abs_span_deg"])

        motor_data = joints.get(str(motor_id), {})
        open_tick = float(motor_data.get("motor_max_open_ticks", np.nan))
        closed_tick = float(motor_data.get("motor_max_closed_ticks", np.nan))

        motor_open_angle = motor_tick_to_angle_with_span(open_tick, open_tick, closed_tick, expected_span)
        motor_closed_angle = motor_tick_to_angle_with_span(closed_tick, open_tick, closed_tick, expected_span)
        motor_delta_abs = abs(motor_closed_angle - motor_open_angle)

        sensor_open = float(fully_open.get(str(sensor_id), {}).get("deg", np.nan))
        sensor_closed = float(
            motor_data.get("sensor_readings_when_joint_closed", {}).get(str(sensor_id), {}).get("deg", np.nan)
        )
        sensor_delta_signed = sensor_closed - sensor_open if np.isfinite(sensor_open) and np.isfinite(sensor_closed) else np.nan
        sensor_delta_abs = abs(sensor_delta_signed) if np.isfinite(sensor_delta_signed) else np.nan

        global_min = sensor_global.get(sensor_id, {}).get("min_deg", np.nan)
        global_max = sensor_global.get(sensor_id, {}).get("max_deg", np.nan)
        global_span = sensor_global.get(sensor_id, {}).get("span_deg", np.nan)

        err_pair = sensor_delta_abs - motor_delta_abs if np.isfinite(sensor_delta_abs) else np.nan
        err_global = global_span - motor_delta_abs if np.isfinite(global_span) else np.nan

        rows.append(
            {
                "joint": joint_name,
                "sensor": sensor_id,
                "motor": motor_id,
                "expected_abs_span_deg": expected_span,
                "motor_max_open_ticks": open_tick,
                "motor_max_closed_ticks": closed_tick,
                "motor_delta_abs_deg": motor_delta_abs,
                "sensor_open_deg": sensor_open,
                "sensor_closed_deg": sensor_closed,
                "sensor_pair_delta_abs_deg": sensor_delta_abs,
                "sensor_pair_delta_signed_deg": sensor_delta_signed,
                "sensor_global_min_deg": global_min,
                "sensor_global_max_deg": global_max,
                "sensor_global_span_deg": global_span,
                "pair_minus_motor_deg": err_pair,
                "global_minus_motor_deg": err_global,
            }
        )

    # Save CSV for quantitative inspection.
    csv_path = os.path.join(output_dir, f"{base_name}_right_hand_delta_comparison.csv")
    header = [
        "joint",
        "sensor",
        "motor",
        "expected_abs_span_deg",
        "motor_max_open_ticks",
        "motor_max_closed_ticks",
        "motor_delta_abs_deg",
        "sensor_open_deg",
        "sensor_closed_deg",
        "sensor_pair_delta_abs_deg",
        "sensor_pair_delta_signed_deg",
        "sensor_global_min_deg",
        "sensor_global_max_deg",
        "sensor_global_span_deg",
        "pair_minus_motor_deg",
        "global_minus_motor_deg",
    ]
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(
                ",".join(str(r[k]) for k in header) + "\n"
            )
    print(f"[Analysis] Saved CSV -> {csv_path}", flush=True)

    # Plot 1: expected/motor vs sensor deltas.
    labels = [r["joint"] for r in rows]
    x = np.arange(len(labels))
    expected_vals = np.array([r["expected_abs_span_deg"] for r in rows], dtype=float)
    motor_vals = np.array([r["motor_delta_abs_deg"] for r in rows], dtype=float)
    pair_vals = np.array([r["sensor_pair_delta_abs_deg"] for r in rows], dtype=float)
    global_vals = np.array([r["sensor_global_span_deg"] for r in rows], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    ax0 = axes[0]
    width = 0.2
    ax0.bar(x - 1.5 * width, expected_vals, width=width, label="Expected abs delta", color="#4C78A8")
    ax0.bar(x - 0.5 * width, motor_vals, width=width, label="Motor delta (from ticks)", color="#59A14F")
    ax0.bar(x + 0.5 * width, pair_vals, width=width, label="Sensor paired delta", color="#F28E2B")
    ax0.bar(x + 1.5 * width, global_vals, width=width, label="Sensor global span", color="#B07AA1")

    ax0.set_ylabel("Delta (deg)")
    ax0.set_title("Right Hand: Motor-vs-Sensor Delta Comparison")
    ax0.set_xticks(x)
    ax0.set_xticklabels(labels, rotation=20, ha="right")
    ax0.grid(axis="y", alpha=0.3)
    ax0.legend(loc="best")

    # Plot 2: pair delta error vs motor target.
    ax1 = axes[1]
    pair_err = np.array([r["pair_minus_motor_deg"] for r in rows], dtype=float)
    global_err = np.array([r["global_minus_motor_deg"] for r in rows], dtype=float)
    ax1.axhline(0.0, color="black", linewidth=1)
    ax1.bar(x - 0.2, pair_err, width=0.4, label="Pair delta - motor delta", color="#E15759")
    ax1.bar(x + 0.2, global_err, width=0.4, label="Global span - motor delta", color="#76B7B2")
    ax1.set_ylabel("Error (deg)")
    ax1.set_title("Sensor-vs-Motor Delta Error (near 0 is better)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=20, ha="right")
    ax1.grid(axis="y", alpha=0.3)
    ax1.legend(loc="best")

    plt.tight_layout()
    png_path = os.path.join(output_dir, f"{base_name}_right_hand_delta_comparison.png")
    plt.savefig(png_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Analysis] Saved plot -> {png_path}", flush=True)

    print("\n[Analysis] Summary (abs deltas):", flush=True)
    for r in rows:
        print(
            "  "
            + f"{r['joint']}: "
            + f"motor={r['motor_delta_abs_deg']:.2f}°, "
            + f"sensor_pair={r['sensor_pair_delta_abs_deg'] if np.isfinite(r['sensor_pair_delta_abs_deg']) else 'nan'}°, "
            + f"sensor_global={r['sensor_global_span_deg'] if np.isfinite(r['sensor_global_span_deg']) else 'nan'}°",
            flush=True,
        )


def _parse_sensor_line(line: str) -> dict:
    """Parse one output line from the calibration firmware.

    Expected format:  S0:123.45:1405\\tS1:200.10:2275\\tS2:300.88:3424
    Returns: {0: {"deg": 123.45, "raw": 1405}, 1: ..., 2: ...}
    Returns {} on parse failure.
    """
    result = {}
    try:
        fields = line.strip().split("\t")
        for field in fields:
            parts = field.split(":")
            if len(parts) != 3 or not parts[0].startswith("S"):
                continue
            idx = int(parts[0][1:])
            result[idx] = {"deg": float(parts[1]), "raw": int(parts[2])}
    except Exception:
        pass
    return result


class SerialSensorReader:
    """Thin wrapper around pyserial for the AS5600 calibration firmware."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        print(f"[Serial] Connecting to {port} @ {baud} baud …", flush=True)
        self.conn = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)  # let the ESP32 boot / emit its banner
        self.flush()
        # Turn off streaming so reads are deterministic during calibration
        self.send_command("s")  # toggle streaming OFF
        time.sleep(0.1)
        self.flush()
        print("[Serial] Connected. Streaming disabled (on-demand reads).", flush=True)

    def flush(self):
        self.conn.reset_input_buffer()

    def send_command(self, cmd: str):
        self.conn.write(cmd.encode())

    def read_once(self) -> dict:
        """Send the 'r' command and return parsed sensor readings.
        Retries up to 5 times on malformed lines.
        """
        for _ in range(5):
            self.flush()
            self.send_command("r")
            raw_line = self.conn.readline().decode(errors="ignore")
            # Skip firmware status lines that start with '['
            if raw_line.startswith("["):
                raw_line = self.conn.readline().decode(errors="ignore")
            data = _parse_sensor_line(raw_line)
            if data:
                return data
            time.sleep(0.05)
        return {}

    def enable_streaming(self):
        self.send_command("s")

    def close(self):
        self.conn.close()


# ── Main calibrator ───────────────────────────────────────────────────────────

class SensorHandCalibrator:
    """
    Calibrates both the RUKA motor encoder limits and the AS5600 sensor limits.

    Parameters
    ----------
    data_save_dir : str
        Directory where .npy calibration files will be written.
    hand_type : str
        "right" or "left" (same as calibrate_motors.py).
    serial_reader : SerialSensorReader
        Open serial connection to the ESP32.
    sensor_map : dict {int → int}
        Maps sensor index (0, 1, 2) to a motor_id (1-based). Motors without a
        sensor entry are still calibrated; sensor arrays will hold NaN.
    curr_lim : int
        Current threshold (raw units) for detecting the mechanical stop.
    testing : bool
        If True, print verbose binary-search debug info.
    motor_ids : list[int]
        Motor IDs to calibrate (1-based). Defaults to first 11.
    """

    def __init__(
        self,
        data_save_dir: str,
        hand_type: str,
        serial_reader: SerialSensorReader,
        sensor_map: dict,
        curr_lim: int = 50,
        testing: bool = False,
        motor_ids: list = None,
    ):
        self.hand = Hand(hand_type)
        self.serial = serial_reader
        self.curr_lim = curr_lim
        self.testing = testing
        self.motor_ids = motor_ids if motor_ids else [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.data_save_dir = data_save_dir
        self.sensor_map = sensor_map  # {sensor_idx: motor_id}

        # Reverse: motor_id → list of sensor indices that monitor it
        self.motor_to_sensors: dict = {}
        for s_idx, m_id in sensor_map.items():
            self.motor_to_sensors.setdefault(m_id, []).append(s_idx)

        n = NUMBER_OF_SENSORS
        # Per-sensor calibration arrays (indexed by sensor_idx 0..N-1)
        self._sensor_curl    = np.full(n, np.nan)
        self._sensor_tension = np.full(n, np.nan)
        self._sensor_min     = np.full(n, np.inf)
        self._sensor_max     = np.full(n, -np.inf)

        # Save paths
        self.curled_path          = os.path.join(data_save_dir, f"{hand_type}_curl_limits.npy")
        self.tension_path         = os.path.join(data_save_dir, f"{hand_type}_tension_limits.npy")
        self.sensor_curl_path     = os.path.join(data_save_dir, f"{hand_type}_sensor_curl_raw.npy")
        self.sensor_tension_path  = os.path.join(data_save_dir, f"{hand_type}_sensor_tension_raw.npy")
        self.sensor_min_path      = os.path.join(data_save_dir, f"{hand_type}_sensor_sweep_min.npy")
        self.sensor_max_path      = os.path.join(data_save_dir, f"{hand_type}_sensor_sweep_max.npy")

    # ── Sensor helpers ────────────────────────────────────────────────────────

    def _read_sensors(self) -> dict:
        return self.serial.read_once()

    def _update_min_max(self, readings: dict):
        """Update the running min/max accumulators from a sensor dict."""
        for s_idx, vals in readings.items():
            if s_idx < NUMBER_OF_SENSORS:
                raw = vals["raw"]
                if raw < self._sensor_min[s_idx]:
                    self._sensor_min[s_idx] = raw
                if raw > self._sensor_max[s_idx]:
                    self._sensor_max[s_idx] = raw

    def _sensor_summary(self) -> str:
        """One-line human-readable string of all current sensor readings."""
        data = self._read_sensors()
        parts = []
        for s in range(NUMBER_OF_SENSORS):
            if s in data:
                parts.append(f"S{s}={data[s]['raw']:4d}({data[s]['deg']:6.1f}°)")
            else:
                parts.append(f"S{s}=----")
        return "  ".join(parts)

    # ── Motor helpers (mirror calibrate_motors.py) ────────────────────────────

    def find_bound(self, motor_id: int) -> tuple:
        """Binary-search for the mechanical curled limit of one motor.

        Identical to HandCalibrator.find_bound() in calibrate_motors.py, with
        an extra AS5600 reading captured at the final position.

        Returns
        -------
        pres_pos : int
            Motor encoder ticks at the detected limit.
        sensor_at_bound : dict  {sensor_idx: {"deg": ..., "raw": ...}}
            Sensor values at the moment the limit was reached.
        """
        t = 2
        curr_lim_local = self.curr_lim
        if motor_id in [4, 5]:
            curr_lim_local = 250 if motor_id == 4 else 200
            t = 5

        if self.testing:
            print(f"------------ MOTOR {motor_id} ------------")

        if self.hand.hand_type == "right":
            start_pos = 100
            f = 1
        else:
            start_pos = 4000
            f = -1

        l_bound = 100
        u_bound = 4000
        n_motors = len(self.motor_ids)
        pos = np.array([start_pos] * n_motors)
        cur = 1_000_000
        pres_pos = start_pos

        while abs(u_bound - l_bound) > 10 or f * cur > curr_lim_local:
            com_pos = (u_bound + l_bound) // 2 - 1
            pos[motor_id - 1] = com_pos
            self.hand.set_pos(pos)
            time.sleep(t)
            cur = self.hand.read_single_cur(motor_id)
            pres_pos = self.hand.read_pos()[motor_id - 1]

            if self.testing:
                print(f"  [{u_bound}, {l_bound}]  cmd={com_pos}  pos={pres_pos}  cur={cur}")

            if f * cur < curr_lim_local:
                if self.hand.hand_type == "right":
                    l_bound = pres_pos + 1
                    u_bound -= 1
                else:
                    u_bound = pres_pos - 1
                    l_bound += 1
            else:
                if self.hand.hand_type == "right":
                    u_bound = pres_pos + 1
                else:
                    l_bound = pres_pos - 1

        sensor_at_bound = self._read_sensors()
        self._update_min_max(sensor_at_bound)
        return pres_pos, sensor_at_bound

    def find_curled(self) -> tuple:
        """Run find_bound() for every motor.

        Returns
        -------
        curled : np.ndarray  [n_motors]  — motor encoder ticks at curled limit
        sensor_at_curled : np.ndarray [n_sensors]  — AS5600 raw at curled limit
        """
        n_motors = len(self.motor_ids)
        curled = np.zeros(n_motors, dtype=int)
        sensor_at_curled = np.full(NUMBER_OF_SENSORS, np.nan)

        for i, mid in enumerate(self.motor_ids):
            print(f"\n[CURL] Motor {mid} …", flush=True)
            motor_pos, sensor_vals = self.find_bound(mid)
            curled[i] = int(motor_pos)

            # Record sensor values for sensors that map to this motor
            for s_idx in self.motor_to_sensors.get(mid, []):
                if s_idx in sensor_vals:
                    raw = sensor_vals[s_idx]["raw"]
                    sensor_at_curled[s_idx] = raw
                    self._sensor_curl[s_idx] = raw
                    print(f"  → Sensor {s_idx} at curl: {raw} raw", flush=True)

        return curled, sensor_at_curled

    # ── Tensioned estimate (same formula as calibrate_motors.py) ─────────────

    def estimate_tensioned_from_curled(self, curled: np.ndarray) -> np.ndarray:
        f = 1 if self.hand.hand_type == "right" else -1
        return np.array([int(x - f * 1100) for x in curled], dtype=int)

    # ── Sensor range sweep ────────────────────────────────────────────────────

    def measure_sensor_range(
        self,
        motor_id: int,
        motor_curled: int,
        motor_tensioned: int,
        n_steps: int = 40,
    ):
        """Sweep motor from curled to tensioned (and back) recording sensor values.

        Updates self._sensor_min / self._sensor_max in-place.
        """
        sensors_for_motor = self.motor_to_sensors.get(motor_id, [])
        if not sensors_for_motor:
            return  # no sensor mapped to this motor — nothing to sweep

        n_motors = len(self.motor_ids)
        start_pos = np.full(n_motors, motor_tensioned, dtype=float)

        # Forward sweep: tensioned → curled
        waypoints = np.linspace(motor_tensioned, motor_curled, n_steps)
        for wp in waypoints:
            start_pos[motor_id - 1] = wp
            self.hand.set_pos(start_pos)
            time.sleep(0.05)
            readings = self._read_sensors()
            self._update_min_max(readings)

        # Reverse sweep: curled → tensioned
        for wp in reversed(waypoints):
            start_pos[motor_id - 1] = wp
            self.hand.set_pos(start_pos)
            time.sleep(0.05)
            readings = self._read_sensors()
            self._update_min_max(readings)

        print(
            f"  [SWEEP] Motor {motor_id}  "
            + "  ".join(
                f"S{s} min={int(self._sensor_min[s])} max={int(self._sensor_max[s])}"
                for s in sensors_for_motor
                if not np.isinf(self._sensor_min[s])
            ),
            flush=True,
        )

    # ── Interactive tensioned refinement (mirrors calibrate_motors.py) ────────

    def interactive_refine_tensioned(
        self, tensioned_init: np.ndarray, step: int = 10
    ) -> tuple:
        """Use arrow keys to fine-tune the tensioned position motor by motor.

        Shows live AS5600 readings alongside the motor position for each motor
        that has a sensor mapped to it.

        Returns
        -------
        tensioned : np.ndarray  — refined motor ticks for each motor
        sensor_at_tensioned : np.ndarray [n_sensors]  — AS5600 raw at tensioned
        """
        current_pos = np.array(self.hand.read_pos(), dtype=float)
        tensioned = tensioned_init.copy().astype(float)
        sensor_at_tensioned = np.full(NUMBER_OF_SENSORS, np.nan)

        f = 1 if self.hand.hand_type == "right" else -1

        print("\n--- Tensioned Calibration ---")
        print("↑/→ = increase, ↓/← = decrease (±10).  Enter = confirm.  q = skip.\n")

        for mid in self.motor_ids:
            idx = mid - 1
            pos = current_pos.copy()
            pos[idx] = tensioned[idx]
            self.hand.set_pos(pos)
            time.sleep(0.2)

            while True:
                sensor_line = self._sensor_summary()
                print(
                    f"\r[Motor {mid:2d}]  tick={int(pos[idx]):4d}  |  {sensor_line}"
                    + "    ",
                    end="",
                    flush=True,
                )

                k = get_key()

                if k in ("\r", "\n"):
                    tensioned[idx] = int(pos[idx])
                    # Record sensor reading at confirmed tensioned position
                    final_readings = self._read_sensors()
                    self._update_min_max(final_readings)
                    for s_idx in self.motor_to_sensors.get(mid, []):
                        if s_idx in final_readings:
                            raw = final_readings[s_idx]["raw"]
                            sensor_at_tensioned[s_idx] = raw
                            self._sensor_tension[s_idx] = raw
                    print(
                        f"\n  Saved Motor {mid} tensioned = {int(tensioned[idx])}"
                        + (
                            f"  sensors: "
                            + ", ".join(
                                f"S{s}={int(sensor_at_tensioned[s])}"
                                for s in self.motor_to_sensors.get(mid, [])
                            )
                            if self.motor_to_sensors.get(mid)
                            else ""
                        )
                    )
                    break

                elif k in ("\x1b[A", "\x1b[C"):  # Up / Right arrow
                    pos[idx] = max(min(pos[idx] + step * f, 4090), 10)
                    self.hand.set_pos(pos)

                elif k in ("\x1b[B", "\x1b[D"):  # Down / Left arrow
                    pos[idx] = max(min(pos[idx] - step * f, 4090), 0)
                    self.hand.set_pos(pos)

                elif k.lower() == "q":
                    print(f"\n  Skipped Motor {mid}; keeping {int(tensioned[idx])}")
                    break

        print("\nFinal tensioned array:\n", tensioned.astype(int))
        return tensioned.astype(int), sensor_at_tensioned

    # ── Public calibration methods ────────────────────────────────────────────

    def save_curled_limits(self) -> np.ndarray:
        """Run curled calibration and save results."""
        curled, sensor_at_curled = self.find_curled()
        np.save(self.curled_path, curled)
        np.save(self.sensor_curl_path, sensor_at_curled)
        print(f"\nSaved motor  curl → {self.curled_path}")
        print(f"Saved sensor curl → {self.sensor_curl_path}")
        print(f"  sensor values at curl: {sensor_at_curled}")
        return curled

    def save_tensioned_limits(self, curled: np.ndarray = None):
        """Run tensioned calibration (interactive) + sensor range sweep and save."""
        if curled is None:
            if os.path.exists(self.curled_path):
                curled = np.load(self.curled_path)
            else:
                print("Curled limits not found — running curl calibration first …")
                curled = self.save_curled_limits()

        t_init = self.estimate_tensioned_from_curled(curled)

        # Sweep each motor to record sensor min/max before asking the user
        print("\n[SWEEP] Measuring sensor range for each motor …")
        for i, mid in enumerate(self.motor_ids):
            self.measure_sensor_range(mid, int(curled[i]), int(t_init[i]))

        # Interactive refinement (same UX as calibrate_motors.py)
        t_refined, sensor_at_tensioned = self.interactive_refine_tensioned(t_init, step=10)

        # Finalise sensor min/max now that tensioned positions are confirmed
        sensor_min = np.where(np.isinf(self._sensor_min), np.nan, self._sensor_min)
        sensor_max = np.where(np.isinf(self._sensor_max), np.nan, self._sensor_max)

        np.save(self.tension_path, t_refined)
        np.save(self.sensor_tension_path, sensor_at_tensioned)
        np.save(self.sensor_min_path, sensor_min.astype(float))
        np.save(self.sensor_max_path, sensor_max.astype(float))

        print(f"\nSaved motor  tension   → {self.tension_path}")
        print(f"Saved sensor tension   → {self.sensor_tension_path}")
        print(f"Saved sensor sweep min → {self.sensor_min_path}  {sensor_min}")
        print(f"Saved sensor sweep max → {self.sensor_max_path}  {sensor_max}")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate RUKA hand motors AND AS5600 magnetic sensors."
    )
    parser.add_argument(
        "-ht", "--hand-type",
        type=str, default="right",
        choices=["right", "left"],
        help="Hand to calibrate (right or left).",
    )
    parser.add_argument(
        "-p", "--serial-port",
        type=str, default="/dev/ttyUSB2",
        help="Serial port of the ESP32 running AS5600_Mux_Calibration.ino.",
    )
    parser.add_argument(
        "--baud",
        type=int, default=115200,
        help="Serial baud rate (default: 115200).",
    )
    parser.add_argument(
        "--sensor-map",
        type=str,
        default='{"0": 1, "1": 2, "2": 3}',
        help=(
            'JSON mapping sensor index → motor_id (1-based). '
            'Example: \'{"0": 1, "1": 6, "2": 11}\' '
            '(default maps sensor 0→motor 1, 1→motor 2, 2→motor 3).'
        ),
    )
    parser.add_argument(
        "--curr-lim",
        type=int, default=50,
        help="Current threshold (raw units) for curled-limit detection.",
    )
    parser.add_argument(
        "--testing",
        action="store_true",
        default=True,
        help="Print verbose binary-search debug info.",
    )
    parser.add_argument(
        "-m", "--mode",
        type=str, choices=["curl", "tension", "both"],
        default="both",
        help="Which pass to run: curl only, tension only, or both (default).",
    )
    parser.add_argument(
        "--analyze-structured-map",
        type=str,
        default="",
        help=(
            "Path to a structured calibration map JSON (for example: "
            "calibration/right_structured_calibration_map.json). "
            "When provided, runs right-hand delta analysis plots and exits "
            "without connecting to hardware."
        ),
    )
    parser.add_argument(
        "--analysis-output-dir",
        type=str,
        default="calibration_visualizations",
        help="Output directory for --analyze-structured-map artifacts.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    # Non-hardware analysis mode: uses structured calibration map + expected deltas.
    if args.analyze_structured_map:
        analyze_structured_map_deltas(args.analyze_structured_map, args.analysis_output_dir)
        sys.exit(0)

    # Parse sensor map: JSON str → {int: int}
    raw_map = json.loads(args.sensor_map)
    sensor_map = {int(k): int(v) for k, v in raw_map.items()}
    print(f"Sensor map: { {s: f'motor {m}' for s, m in sensor_map.items()} }")

    # Prepare save directory
    repo_root = get_repo_root()
    save_dir = os.path.join(repo_root, "curl_limits")
    os.makedirs(save_dir, exist_ok=True)

    # Open serial connection to ESP32
    serial_reader = SerialSensorReader(args.serial_port, baud=args.baud)

    try:
        calibrator = SensorHandCalibrator(
            data_save_dir=save_dir,
            hand_type=args.hand_type,
            serial_reader=serial_reader,
            sensor_map=sensor_map,
            curr_lim=args.curr_lim,
            testing=args.testing,
        )

        curled = None

        if args.mode in ("curl", "both"):
            curled = calibrator.save_curled_limits()

        if args.mode in ("tension", "both"):
            calibrator.save_tensioned_limits(curled)

    finally:
        serial_reader.enable_streaming()  # restore streaming mode
        serial_reader.close()
        print("\nDone.")
