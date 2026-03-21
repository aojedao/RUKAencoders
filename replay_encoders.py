"""replay_encoders.py
====================
Runs the same hand trajectory as retargeting/replay_demo.py while
simultaneously logging:

  1. Motor data  — commanded + actual positions for all 16 Dynamixel motors
  2. Sensor data — raw (0-4095) and degree values from the 3 AS5600 sensors
                   on the ESP32 (AS5600_Mux_Calibration.ino firmware)

Both logs share the same t0 (UNIX epoch in seconds), so timestamps from the
two sources can be directly compared / merged for sync analysis.

Sync mechanism
--------------
  1. Flush the ESP32 serial buffer.
  2. Send 's' to ENABLE streaming on the ESP32.
  3. Record  t0 = time.time()  immediately (this is the shared origin).
  4. Start the background serial-reader thread.
  5. Start hand movement.
  All timestamps are stored as  (time.time() - t0)  seconds from t0.

Saved files (in ./encoder_logs/ by default)
--------------------------------------------
  motor_log.npy    — structured array: t_rel, commanded[16], actual[16]
  sensor_log.npy   — structured array: t_rel, raw[3], deg[3]
  motor_log.csv    — human-readable version of motor_log
  sensor_log.csv   — human-readable version of sensor_log
  t0.txt           — absolute t0 (UNIX timestamp) for external alignment

Usage
-----
  # Basic usage:
  python replay_encoders.py --serial-port /dev/ttyUSB2

  # With right hand and plotting:
  python replay_encoders.py --serial-port /dev/ttyUSB2 --hand right --plot

  # Custom trajectory label and output dir:
  python replay_encoders.py --serial-port /dev/ttyUSB2 \\
      --label my_run --out-dir /tmp/logs --plot
      
  # Replay mode: plot from existing logs (no data collection):
  python replay_encoders.py --replay-only --label my_run
  python replay_encoders.py --replay-only --label trial_01 --hand right --out-dir /tmp/logs
      
Plotting
--------
  Use --plot flag to generate angle comparison plots after data collection.
  Use --replay-only to regenerate plots from existing log files without running
  a new data collection (useful for iterating on plot improvements).
  
  The plots show motor angles (computed from motor positions) vs sensor angles
  from the AS5600 encoders, allowing you to verify sensor calibration and
  validate the motor-to-angle conversion.
"""

import argparse
import csv
import os
import queue
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from types import SimpleNamespace

from ruka_hand.control.hand import Hand
from ruka_hand.control.rukav2_teleop import RUKAv2Handler
from ruka_hand.utils.trajectory import move_to_pos

# ── Number of sensors (must match firmware) ───────────────────────────────────
N_SENSORS = 7  # Updated to support all 7 AS5600 sensors
N_MOTORS  = 16

# ── Sensor-to-Motor Mapping ───────────────────────────────────────────────────
# Each sensor measures the angle of a specific motor:
#   Sensor 0 (s0) → Motor 8  (m8)
#   Sensor 1 (s1) → Motor 7  (m7)
#   Sensor 2 (s2) → Motor 7  (m7)
#   Sensor 3 (s3) → Motor 9  (m9)
#   Sensor 4 (s4) → Motor 13 (m13)
#   Sensor 5 (s5) → Motor 14 (m14)
#   Sensor 6 (s6) → Motor 12 (m12)

# ── Serial helpers ────────────────────────────────────────────────────────────

def _parse_sensor_line(line: str) -> dict | None:
    """Parse one output line from AS5600_Mux_Calibration.ino.

    Expected:  S0:123.45:1405\\tS1:200.10:2275\\tS2:300.88:3424
    Returns:   {0: {"deg": 123.45, "raw": 1405}, 1: ..., 2: ...}
    Returns None on any parse error or if the line is a firmware status message.
    """
    line = line.strip()
    if not line or line.startswith("["):
        return None
    result = {}
    try:
        for field in line.split("\t"):
            parts = field.split(":")
            if len(parts) != 3 or not parts[0].startswith("S"):
                return None
            idx = int(parts[0][1:])
            result[idx] = {"deg": float(parts[1]), "raw": int(parts[2])}
    except Exception:
        return None
    return result if result else None


class _SensorReaderThread(threading.Thread):
    """Background thread: reads the ESP32 serial stream and queues timestamped records.

    Parameters
    ----------
    port : str           Serial device path.
    baud : int           Baud rate (default 115200 to match firmware).
    t0   : float         Shared time origin (UNIX seconds).
    out_q : queue.Queue  Records are put here as dicts:
                           {"t_rel": float, "raw": [int×N], "deg": [float×N]}
    stop_event : threading.Event  Set this to stop the thread cleanly.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        t0: float,
        out_q: queue.Queue,
        stop_event: threading.Event,
    ):
        super().__init__(daemon=True)
        self.port        = port
        self.baud        = baud
        self.t0          = t0
        self.out_q       = out_q
        self.stop_event  = stop_event

    def run(self):
        try:
            conn = serial.Serial(self.port, self.baud, timeout=0.5)
        except serial.SerialException as e:
            print(f"[SensorThread] ERROR: could not open {self.port}: {e}")
            return

        print(f"[SensorThread] Streaming from {self.port} …", flush=True)
        while not self.stop_event.is_set():
            try:
                raw_line = conn.readline().decode(errors="ignore")
            except Exception:
                break

            t_recv = time.time() - self.t0
            parsed = _parse_sensor_line(raw_line)
            if parsed is None:
                continue

            raw  = [parsed.get(i, {}).get("raw", -1)   for i in range(N_SENSORS)]
            deg  = [parsed.get(i, {}).get("deg", float("nan")) for i in range(N_SENSORS)]
            self.out_q.put({"t_rel": t_recv, "raw": raw, "deg": deg})

        conn.close()
        print("[SensorThread] Stopped.", flush=True)


# ── Motor-logging wrapper around move_to_pos ─────────────────────────────────

def move_and_log(
    curr_pos,
    des_pos,
    hand: Hand,
    t0: float,
    motor_log: list,
    traj_len: int = 50,
    sleep_time: float = 0.01,
):
    """Identical to move_to_pos but also logs commanded + actual positions.

    Each step appends to motor_log:
      {"t_rel": float, "commanded": list[int], "actual": list[int]}
    """
    if traj_len == 1:
        trajectory = [des_pos]
    else:
        trajectory = np.linspace(curr_pos, des_pos, traj_len)[1:]

    for hand_pos in trajectory:
        t_cmd = time.time()
        hand.set_pos(hand_pos)

        # Read back actual position immediately after commanding
        actual = hand.read_pos()

        t_rel  = t_cmd - t0
        motor_log.append({
            "t_rel":     t_rel,
            "commanded": [int(v) for v in hand_pos],
            "actual":    [int(v) if v is not None else -1 for v in actual],
        })

        time.sleep(sleep_time)


# ── Save helpers ──────────────────────────────────────────────────────────────

def _save_motor_log(motor_log: list, out_dir: str, label: str):
    """Save motor log as .npy structured array and .csv."""
    n = len(motor_log)
    if n == 0:
        print("[Save] Motor log is empty — nothing saved.")
        return

    # Structured numpy array
    dt = np.dtype([
        ("t_rel",     np.float64),
        ("commanded", np.int32,   (N_MOTORS,)),
        ("actual",    np.int32,   (N_MOTORS,)),
    ])
    arr = np.zeros(n, dtype=dt)
    for i, row in enumerate(motor_log):
        arr[i]["t_rel"]     = row["t_rel"]
        arr[i]["commanded"] = row["commanded"]
        arr[i]["actual"]    = row["actual"]

    npy_path = os.path.join(out_dir, f"{label}_motor_log.npy")
    np.save(npy_path, arr)
    print(f"[Save] Motor log  → {npy_path}  ({n} rows)", flush=True)

    # CSV
    csv_path = os.path.join(out_dir, f"{label}_motor_log.csv")
    cmd_cols = [f"cmd_{i}" for i in range(N_MOTORS)]
    act_cols = [f"act_{i}" for i in range(N_MOTORS)]
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_rel"] + cmd_cols + act_cols)
        for row in motor_log:
            writer.writerow([row["t_rel"]] + row["commanded"] + row["actual"])
    print(f"[Save] Motor log  → {csv_path}", flush=True)


def _save_sensor_log(sensor_records: list, out_dir: str, label: str):
    """Save sensor log as .npy structured array and .csv."""
    n = len(sensor_records)
    if n == 0:
        print("[Save] Sensor log is empty — check serial connection.")
        return

    dt = np.dtype([
        ("t_rel", np.float64),
        ("raw",   np.int32,   (N_SENSORS,)),
        ("deg",   np.float32, (N_SENSORS,)),
    ])
    arr = np.zeros(n, dtype=dt)
    for i, row in enumerate(sensor_records):
        arr[i]["t_rel"] = row["t_rel"]
        arr[i]["raw"]   = row["raw"]
        arr[i]["deg"]   = row["deg"]

    npy_path = os.path.join(out_dir, f"{label}_sensor_log.npy")
    np.save(npy_path, arr)
    print(f"[Save] Sensor log → {npy_path}  ({n} rows)", flush=True)

    csv_path = os.path.join(out_dir, f"{label}_sensor_log.csv")
    raw_cols = [f"raw_{i}" for i in range(N_SENSORS)]
    deg_cols = [f"deg_{i}" for i in range(N_SENSORS)]
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_rel"] + raw_cols + deg_cols)
        for row in sensor_records:
            writer.writerow([row["t_rel"]] + row["raw"] + [f"{d:.2f}" for d in row["deg"]])
    print(f"[Save] Sensor log → {csv_path}", flush=True)


def _save_t0(t0: float, out_dir: str, label: str):
    path = os.path.join(out_dir, f"{label}_t0.txt")
    with open(path, "w") as f:
        f.write(f"{t0:.6f}\n")
    print(f"[Save] t0 = {t0:.6f}  → {path}", flush=True)


def _make_unique_run_label(out_dir: str, base_label: str) -> str:
    """Create a non-overwriting label using date/time, with collision fallback."""
    safe_base = (base_label or "run").strip() or "run"
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = f"{timestamp}_{safe_base}"
    idx = 1

    while (
        os.path.exists(os.path.join(out_dir, f"{candidate}_motor_log.npy"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_motor_log.csv"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_sensor_log.npy"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_sensor_log.csv"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_t0.txt"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_hand_bounds.npz"))
        or os.path.exists(os.path.join(out_dir, f"{candidate}_angle_comparison.png"))
    ):
        candidate = f"{timestamp}_{safe_base}_{idx:02d}"
        idx += 1

    return candidate


def _save_hand_bounds(hand: Hand, out_dir: str, label: str):
    path = os.path.join(out_dir, f"{label}_hand_bounds.npz")
    np.savez(
        path,
        tensioned_pos=np.array(hand.tensioned_pos, dtype=float),
        curled_bound=np.array(hand.curled_bound, dtype=float),
    )
    print(f"[Save] Hand bounds → {path}", flush=True)


def _load_hand_bounds(out_dir: str, label: str, hand_type: str):
    path = os.path.join(out_dir, f"{label}_hand_bounds.npz")
    if os.path.exists(path):
        data = np.load(path)
        print(f"[Plot] Loaded hand bounds from {path}", flush=True)
        return SimpleNamespace(
            tensioned_pos=np.array(data["tensioned_pos"], dtype=float),
            curled_bound=np.array(data["curled_bound"], dtype=float),
        )

    print(f"[Plot] WARNING: No saved hand bounds found at {path}", flush=True)
    print("[Plot] Attempting to load hand bounds from live hand connection...", flush=True)
    try:
        hand = Hand(hand_type)
        bounds = SimpleNamespace(
            tensioned_pos=np.array(hand.tensioned_pos, dtype=float),
            curled_bound=np.array(hand.curled_bound, dtype=float),
        )
        hand.close()
        return bounds
    except Exception as e:
        print(f"[Plot] ERROR: Could not load hand bounds from file or live connection", flush=True)
        print(f"[Plot]   Hand error: {e}", flush=True)
        print(f"[Plot]   Cannot proceed without hand bounds for motor position mapping", flush=True)
        return None


def _sensor_angles_to_motor_positions(sensor_angles: np.ndarray, motor_idx: int, hand_bounds) -> np.ndarray:
    """Use library compute_motor_pos to map sensor angles to motor-position units."""
    proxy = SimpleNamespace(hand=hand_bounds)
    mapped = np.zeros(len(sensor_angles), dtype=float)
    for i, angle in enumerate(sensor_angles):
        joint_angles = np.zeros(16, dtype=float)
        joint_angles[motor_idx] = angle
        mapped[i] = RUKAv2Handler.compute_motor_pos(proxy, joint_angles)[motor_idx]
    return mapped


def plot_motor_and_sensor_data(out_dir: str, label: str, hand_type: str = "left", motor_log_data=None, sensor_log_data=None):
    """Plot motor positions vs sensor-mapped motor positions.
    
    Can work in two modes:
    1. Live plotting: Pass motor_log_data and sensor_log_data directly from memory
    2. Replay mode: Leave data params as None to load from disk files
    
    Parameters
    ----------
    out_dir : str
        Directory to save plots (and load from in replay mode)
    label : str
        Prefix of the log files or run label
    hand_type : str
        "left" or "right" hand type
    motor_log_data : list of dict, optional
        In-memory motor log with keys: t_rel, commanded, actual
    sensor_log_data : list of dict, optional
        In-memory sensor log with keys: t_rel, raw, deg
    """
    # Determine if we're in live mode or replay mode
    if motor_log_data is not None and sensor_log_data is not None:
        # Live mode: convert in-memory data to structured arrays
        print(f"[Plot] Using in-memory data ({len(motor_log_data)} motor samples, {len(sensor_log_data)} sensor samples)", flush=True)
        
        # Convert motor log to structured array
        dt_motor = np.dtype([
            ("t_rel",     np.float64),
            ("commanded", np.int32,   (N_MOTORS,)),
            ("actual",    np.int32,   (N_MOTORS,)),
        ])
        motor_log = np.zeros(len(motor_log_data), dtype=dt_motor)
        for i, row in enumerate(motor_log_data):
            motor_log[i]["t_rel"]     = row["t_rel"]
            motor_log[i]["commanded"] = row["commanded"]
            motor_log[i]["actual"]    = row["actual"]
        
        # Convert sensor log to structured array
        dt_sensor = np.dtype([
            ("t_rel", np.float64),
            ("raw",   np.int32,   (N_SENSORS,)),
            ("deg",   np.float32, (N_SENSORS,)),
        ])
        sensor_log = np.zeros(len(sensor_log_data), dtype=dt_sensor)
        for i, row in enumerate(sensor_log_data):
            sensor_log[i]["t_rel"] = row["t_rel"]
            sensor_log[i]["raw"]   = row["raw"]
            sensor_log[i]["deg"]   = row["deg"]
    else:
        # Replay mode: load from disk
        motor_log_path = os.path.join(out_dir, f"{label}_motor_log.npy")
        sensor_log_path = os.path.join(out_dir, f"{label}_sensor_log.npy")
        
        if not os.path.exists(motor_log_path) or not os.path.exists(sensor_log_path):
            print(f"[Plot] ERROR: Could not find log files with label '{label}'", flush=True)
            print(f"[Plot]   Looking for:", flush=True)
            print(f"[Plot]     - {motor_log_path}", flush=True)
            print(f"[Plot]     - {sensor_log_path}", flush=True)
            print(f"[Plot]   Files in {out_dir}:", flush=True)
            try:
                files = os.listdir(out_dir)
                if files:
                    for f in sorted(files):
                        print(f"[Plot]     {f}", flush=True)
                else:
                    print(f"[Plot]     (directory is empty)", flush=True)
            except Exception as e:
                print(f"[Plot]     (could not list directory: {e})", flush=True)
            return
        
        # Load data from disk
        print(f"[Plot] Loading logs from disk with label '{label}'", flush=True)
        motor_log = np.load(motor_log_path)
        sensor_log = np.load(sensor_log_path)
    
    print(f"[Plot] Loaded {len(motor_log)} motor samples, {len(sensor_log)} sensor samples")
    
    # Extract motor data
    motor_times = motor_log["t_rel"]
    commanded_positions = motor_log["commanded"]
    actual_positions = motor_log["actual"]
    
    hand_bounds = _load_hand_bounds(out_dir, label, hand_type)
    if hand_bounds is None:
        print("[Plot] Cannot map sensor angles using library compute_motor_pos without hand bounds.", flush=True)
        return
    
    # Extract sensor data
    sensor_times = sensor_log["t_rel"]
    sensor_degrees = sensor_log["deg"]

    # Hand-specific sensor direction correction
    sensor_plot_degrees = sensor_degrees.copy()
    if hand_type == "right":
        sensor_plot_degrees[:, 0] *= -1
        print("[Plot] Applied Sensor 0 direction inversion for right hand", flush=True)
    
    # Create figure with subplots for each sensor
    fig, axes = plt.subplots(N_SENSORS, 1, figsize=(12, 4 * N_SENSORS))
    if N_SENSORS == 1:
        axes = [axes]
    
    # Mapping of sensors to motors (sensors track specific motor angles)
    # s0→m8, s1→m7, s2→m7, s3→m9, s4→m13, s5→m14, s6→m12
    sensor_motor_map = {
        0: [8],    # Sensor 0 → Motor 8
        1: [7],    # Sensor 1 → Motor 7
        2: [7],    # Sensor 2 → Motor 7
        3: [9],    # Sensor 3 → Motor 9
        4: [13],   # Sensor 4 → Motor 13
        5: [14],   # Sensor 5 → Motor 14
        6: [12]    # Sensor 6 → Motor 12
    }
    
    for sensor_idx in range(N_SENSORS):
        ax = axes[sensor_idx]
        
        # Plot sensor mapped through library compute_motor_pos into motor-position units
        if sensor_idx in sensor_motor_map:
            for motor_idx in sensor_motor_map[sensor_idx]:
                sensor_mapped_positions = _sensor_angles_to_motor_positions(
                    sensor_plot_degrees[:, sensor_idx],
                    motor_idx,
                    hand_bounds,
                )
                ax.plot(sensor_times, sensor_mapped_positions,
                        'b-', linewidth=2, label=f'Sensor {sensor_idx} mapped→Motor {motor_idx}', alpha=0.85)
                ax.plot(motor_times, commanded_positions[:, motor_idx],
                        '--', linewidth=1.5, label=f'Motor {motor_idx} commanded', alpha=0.65)
                ax.plot(motor_times, actual_positions[:, motor_idx],
                        '-', linewidth=1.0, label=f'Motor {motor_idx} actual', alpha=0.65)
        
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Motor Position', fontsize=12)
        motor_label = sensor_motor_map.get(sensor_idx, ["?"])[0]
        ax.set_title(f"Sensor {sensor_idx} vs Motor {motor_label} Position", fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_path = os.path.join(out_dir, f"{label}_angle_comparison.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"[Plot] Saved plot → {plot_path}")
    plt.show()


# ── Trajectory definition (mirrors replay_demo.py) ────────────────────────────

def build_sequence(hand: Hand, hand_type: str = "left") -> list[tuple[str, list]]:
    """
    Returns an ordered list of (label, position) pairs that define the demo.

    Edit this function to change the trajectory.  Each entry causes one
    move_and_log() call from the current position to that target.
    
    Parameters
    ----------
    hand : Hand
        Hand instance
    hand_type : str
        Either "left" or "right"
    """
    # -- same positions from the active section of replay_demo.py --
    left_hand = [2297, 1261, 1629, 3477, 2010, 1870, 2090, 2837,
                 2790, 1694, 1440, 1450, 2294, 1700, 2900, 2400]
    right_hand= [3559, 2864, 2247, 1891, 3407, 849, 3098, 1641, 1490, 853, 230, 1500, 3455, 2860, 2000, 1850]

    # Optionally add more waypoints here, e.g.:
    # init_pos  = [3435, 2140, 2746, 2160, 2320, 1920, 2500, 1850, ...]
    # ring_pos  = [...]
    # pinky_pos = [...]
    # final_pos = [...]
    
                        #     [3559, 2864, 2247, 1891, 3407, 849,  3098, 1641, 1490, 853, 230, 1500, 3455, 2860, 2000, 1850]
    # True! index_thumb_pinch_pos =   [2455, 2864, 1384, 1891, 2857, 1849, 1500, 1641,  380, 1271, 1430, 1400, 2576, 2150, 2000, 1850]
    index_thumb_pinch_pos =   [2455, 2864, 1384, 1891, 2857, 1849, 1500, 1796,  380, 1271, 1530, 1400, 3250, 2150, 2000, 1850]
    wallet_pos    =           [2455, 2864, 1384, 1891, 2857, 1849, 3810, 1796, 1590, 1271, 1530, 1400,3250, 2150, 2000, 1850]
                      #        [2815, 2864, 1384, 1891, 2857, 1824, 2000, 1676, 600, 1947, 1990, 800, 2576, 2050, 2000, 1200]

# sensors                                                         s1   s0  m7            s6    s4   s5
 
                    #                                            |index       |middle   |thumb         |wrist     |
                     #              1,   2,   3,   4,   5,   6,   7,   8,  9,   10,  11,  12,  13,  14,  15,  16                    
    index_thumb_pinch_sensor  = [2557,2978,1928,1944,3063,1849,2159,1560, 900,1271, 911,1514,2812,2381,2000,1850]             
    closed_thumb_pinch_sensor = [2557,2978,1928,1944,3063,1849,3810,1796,1590,1271,1530,1400,3250,1950,2000,1850]                      
    # index_thumb_pinch_pos = [3559, 2864, 2247, 1891, 3407, 849,  2000, 1641, 380, 853, 230, 900, 2576, 2150, 2000, 1850]

    #wallet_pos = [2815, 2864, 1384, 1891, 2857, 1824, 2000, 1676, 600, 1947, 1990, 800, 2576, 2050, 2000, 1200]
   
    chosen_hand = right_hand if hand_type == "right" else left_hand
    label = f"{hand_type}_hand"
    
    return [
        #(label, chosen_hand),
        ("wallet_pos", wallet_pos),
        ("index_thumb_pinch_pos", index_thumb_pinch_sensor),
        ("closed_thumb_pinch_pos", closed_thumb_pinch_sensor),
        ("index_thumb_pinch_pos", index_thumb_pinch_sensor),
        ("closed_thumb_pinch_pos", closed_thumb_pinch_sensor),
        ("index_thumb_pinch_pos", index_thumb_pinch_sensor),
        ("wallet_pos", wallet_pos),
        ("index_thumb_pinch_pos", index_thumb_pinch_sensor),
        ("wallet_pos", wallet_pos),
        ("index_thumb_pinch_pos", index_thumb_pinch_sensor),
        ("wallet_pos", wallet_pos)
        # ("ring_pos",  ring_pos),
        # ("pinky_pos", pinky_pos),
        # ("final_pos", final_pos),
    ]


# ── Main ──────────────────────────────────────────────────────────────────────

def replay_only_mode(out_dir: str, label: str, hand_type: str):
    """Plot existing log files without running a new data collection.
    
    Parameters
    ----------
    out_dir : str
        Directory containing the log files
    label : str
        Prefix of the log files to load
    hand_type : str
        "left" or "right" hand type
    """
    print(f"[Replay Mode] Loading existing logs from {out_dir} with label '{label}' ...\n", flush=True)
    
    motor_log_path = os.path.join(out_dir, f"{label}_motor_log.npy")
    sensor_log_path = os.path.join(out_dir, f"{label}_sensor_log.npy")
    
    if not os.path.exists(motor_log_path):
        print(f"[Error] Motor log not found: {motor_log_path}")
        print(f"Available files in {out_dir}:")
        if os.path.exists(out_dir):
            for f in sorted(os.listdir(out_dir)):
                print(f"  - {f}")
        return
    
    if not os.path.exists(sensor_log_path):
        print(f"[Error] Sensor log not found: {sensor_log_path}")
        return
    
    print(f"[Replay Mode] Found log files:")
    print(f"  - {motor_log_path}")
    print(f"  - {sensor_log_path}")
    print()
    
    plot_motor_and_sensor_data(out_dir, label, hand_type)
    print("\n[Done] Replay mode complete.", flush=True)


def main(serial_port: str, baud: int, out_dir: str, label: str, traj_len: int, hand_type: str, plot: bool):
    os.makedirs(out_dir, exist_ok=True)

    run_label = _make_unique_run_label(out_dir, label)

    print(f"[Config] Recording mode with selected hand: {hand_type}", flush=True)
    print(f"[Config] Log label for this run: {run_label}", flush=True)

    # ── 1. Connect to motor hand ──────────────────────────────────────────────
    print(f"[Init] Connecting to {hand_type.capitalize()} Hand …", flush=True)
    hand = Hand(hand_type)

    # ── 2. Prepare serial connection to ESP32 ─────────────────────────────────
    print(f"[Init] Opening serial port {serial_port} …", flush=True)
    esp_conn = serial.Serial(serial_port, baud, timeout=0.5)
    time.sleep(2.0)                       # wait for ESP32 boot
    esp_conn.reset_input_buffer()

    # Make sure streaming is OFF before sync (sends 's'; firmware toggles)
    # We don't know the current state, so read a line first to probe
    esp_conn.write(b"h")                  # request help — safe no-op
    time.sleep(0.1)
    esp_conn.reset_input_buffer()
    # Disable streaming (send 's' twice if unsure; firmware just toggles)
    esp_conn.write(b"s")                  # toggle → likely OFF
    time.sleep(0.1)
    esp_conn.reset_input_buffer()

    # ── 3. SYNC POINT: enable streaming → record t0 ───────────────────────────
    # The instant we tell the ESP32 to start streaming is our shared t0.
    # Any sensor record whose t_rel >= 0 arrived after the sync point.
    print("\n[SYNC] Enabling ESP32 streaming and recording t0 …", flush=True)
    esp_conn.write(b"s")                  # toggle → ON
    t0 = time.time()                      # ← SHARED TIME ORIGIN
    print(f"[SYNC] t0 = {t0:.6f}", flush=True)

    # ── 4. Start background sensor reader ────────────────────────────────────
    sensor_q    = queue.Queue()
    stop_event  = threading.Event()
    reader = _SensorReaderThread(
        port=serial_port,
        baud=baud,
        t0=t0,
        out_q=sensor_q,
        stop_event=stop_event,
    )
    # Override the reader's serial with the already-open connection
    # (avoid opening the port twice)
    reader.start()

    # Brief pause so the reader thread captures a few baseline samples
    time.sleep(0.5)

    # ── 5. Run trajectory ────────────────────────────────────────────────────
    motor_log = []
    sequence  = build_sequence(hand, hand_type)

    print(f"\n[Demo] Starting trajectory ({len(sequence)} step(s)) …", flush=True)
    time.sleep(1.0)                       # motor warm-up wait (mirrors replay_demo.py)

    for step_label, des_pos in sequence:
        curr_pos = hand.read_pos()
        print(f"[Demo] → {step_label}", flush=True)
        move_and_log(
            curr_pos  = curr_pos,
            des_pos   = des_pos,
            hand      = hand,
            t0        = t0,
            motor_log = motor_log,
            traj_len  = traj_len,
        )

    print("[Demo] Trajectory complete.", flush=True)

    # Brief pause to capture trailing sensor samples after movement ends
    time.sleep(0.5)

    # ── 6. Stop sensor reader and drain queue ─────────────────────────────────
    stop_event.set()
    reader.join(timeout=2.0)
    esp_conn.write(b"s")                  # toggle streaming OFF
    esp_conn.close()

    sensor_records = []
    while not sensor_q.empty():
        sensor_records.append(sensor_q.get_nowait())

    print(
        f"\n[Stats]  Motor samples:  {len(motor_log)}"
        f"   Sensor samples: {len(sensor_records)}",
        flush=True,
    )

    # ── 7. Save ───────────────────────────────────────────────────────────────
    _save_t0(t0, out_dir, run_label)
    _save_hand_bounds(hand, out_dir, run_label)
    _save_motor_log(motor_log, out_dir, run_label)
    _save_sensor_log(sensor_records, out_dir, run_label)

    print("\n[Done] All files saved.", flush=True)
    print(f"[Done] Saved files use label: {run_label}", flush=True)
    
    # ── 8. Plot (optional) ────────────────────────────────────────────────────────
    if plot:
        print("\n[Plot] Generating angle comparison plots...", flush=True)
        plot_motor_and_sensor_data(out_dir, run_label, hand_type, 
                                   motor_log_data=motor_log, 
                                   sensor_log_data=sensor_records)

# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Run replay_demo trajectory while logging motors + AS5600 sensors, or replay plots from existing logs."
    )
    parser.add_argument(
        "--replay-only",
        action="store_true",
        help="Replay mode: plot existing log files without running new data collection. Requires --label to specify which logs to load.",
    )
    parser.add_argument(
        "-p", "--serial-port",
        type=str, default="/dev/ttyACM0",
        help="Serial port of the ESP32 running AS5600_Mux_Calibration.ino (default: /dev/ttyACM0). Ignored in replay-only mode.",
    )
    parser.add_argument(
        "--baud",
        type=int, default=115200,
        help="Serial baud rate (default: 115200).",
    )
    parser.add_argument(
        "--out-dir",
        type=str, default="encoder_logs",
        help="Directory to save log files (default: ./encoder_logs/).",
    )
    parser.add_argument(
        "--label",
        type=str, default="run",
        help="Prefix for output files, e.g. 'trial_01' (default: 'run').",
    )
    parser.add_argument(
        "--traj-len",
        type=int, default=50,
        help="Number of interpolation steps per movement (default: 50).",
    )
    parser.add_argument(
        "--hand",
        type=str, default="left", choices=["left", "right"],
        help="Which hand to use for recording/replay: 'left' or 'right' (default: 'left').",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate plots comparing motor angles and sensor angles after data collection.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    
    if args.replay_only:
        # Replay mode: just plot existing logs
        replay_only_mode(
            out_dir   = args.out_dir,
            label     = args.label,
            hand_type = args.hand,
        )
    else:
        # Normal mode: collect data and optionally plot
        main(
            serial_port = args.serial_port,
            baud        = args.baud,
            out_dir     = args.out_dir,
            label       = args.label,
            traj_len    = args.traj_len,
            hand_type   = args.hand,
            plot        = args.plot,
        )
