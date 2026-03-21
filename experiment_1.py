"""experiment_1.py — Encoder vs Motor Position Error Mapping
===============================================================
Commands a single motor (or a set of mapped motors) to a series of
discrete target positions, waits for mechanical settling, reads the
AS5600 magnetic angle from the ESP32, and logs both values.

The saved CSV / NPY can be loaded directly into a plotting script to
draw "motor thinks it is here" vs "sensor actually reads here" curves,
revealing the per-joint angle error.

Motor ↔ angle mapping
----------------------
  tensioned_pos  →  0 %  (fingers fully open / tendons tensioned)
  curled_bound   → 100 %  (fingers fully curled)

  Tick = tensioned + (pct/100) × (curled − tensioned)

  motor_estimated_deg = (actual_tick − tensioned) / (curled − tensioned) × 100
                        (i.e. percentage-of-range, same unit as target_pct)

  sensor_deg         = AS5600 degrees (0-360), averaged over --n-samples reads

Settling detection
------------------
  After each set_pos() call we poll read_pos() until the joint is
  within --settle-tol ticks of the target for --settle-n consecutive
  reads, or --settle-timeout seconds elapse (whichever comes first).
  The "settled" flag in the log tells you whether the motor actually
  reached the target.

Saved files (in --out-dir, prefixed with --label)
--------------------------------------------------
  {label}_exp1.npy   — structured numpy array (all columns)
  {label}_exp1.csv   — human-readable version
  {label}_t0.txt     — absolute t0 (UNIX seconds)

Usage examples
--------------
  # Default: test all sensor-mapped motors at 0, 25, 50, 75, 100 %
  python experiment_1.py --serial-port /dev/ttyUSB2 \\
      --sensor-map '{"0": 1, "1": 6, "2": 11}'

  # Custom target percentages
  python experiment_1.py --serial-port /dev/ttyUSB2 \\
      --sensor-map '{"0": 1}' \\
      --targets 0 15 30 45 60 75 90 100

  # Test a specific single motor (index, 1-based)
  python experiment_1.py --serial-port /dev/ttyUSB2 \\
      --sensor-map '{"0": 1}' --motor-id 1 \\
      --targets 0 20 40 60 80 100
"""

import argparse
import csv
import json
import os
import time

import numpy as np
import serial

from ruka_hand.control.hand import Hand
from ruka_hand.utils.trajectory import move_to_pos

# ── Number of AS5600 sensors (must match firmware) ────────────────────────────
N_SENSORS = 3


# ── Serial helpers ────────────────────────────────────────────────────────────

def _parse_sensor_line(line: str) -> dict | None:
    """Parse one output line from AS5600_Mux_Calibration.ino.

    Format:  S0:123.45:1405\\tS1:200.10:2275\\tS2:300.88:3424
    Returns: {0: {"deg": 123.45, "raw": 1405}, ...}  or None on error.
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


def read_sensor_once(conn: serial.Serial, retries: int = 10) -> dict:
    """Request one sensor snapshot via the 'r' command.

    Returns: {sensor_idx: {"deg": float, "raw": int}, ...}
    Empty dict on failure.
    """
    for _ in range(retries):
        conn.reset_input_buffer()
        conn.write(b"r")
        raw_line = conn.readline().decode(errors="ignore")
        if raw_line.startswith("["):               # firmware status line — skip
            raw_line = conn.readline().decode(errors="ignore")
        parsed = _parse_sensor_line(raw_line)
        if parsed:
            return parsed
        time.sleep(0.02)
    return {}


def read_sensor_avg(conn: serial.Serial, n: int = 5) -> dict:
    """Average n sensor readings.

    Returns: {sensor_idx: {"deg_avg": float, "raw_avg": float}}
    """
    accum: dict = {}
    counts: dict = {}
    for _ in range(n):
        data = read_sensor_once(conn)
        for idx, vals in data.items():
            if idx not in accum:
                accum[idx]  = {"deg": 0.0, "raw": 0.0}
                counts[idx] = 0
            accum[idx]["deg"] += vals["deg"]
            accum[idx]["raw"] += vals["raw"]
            counts[idx]       += 1
        time.sleep(0.02)

    result = {}
    for idx in accum:
        c = counts[idx]
        result[idx] = {
            "deg_avg": accum[idx]["deg"] / c,
            "raw_avg": accum[idx]["raw"] / c,
        }
    return result


# ── Settling detection ────────────────────────────────────────────────────────

def wait_for_settle(
    hand: Hand,
    motor_idx: int,          # 0-based index into hand.motors list
    target_tick: int,
    tol: int   = 20,         # ticks tolerance
    n_consec: int = 3,       # consecutive reads within tolerance
    timeout: float = 5.0,    # seconds
    poll_interval: float = 0.05,
) -> tuple[bool, int]:
    """Poll actual motor position until it settles near target_tick.

    Returns (settled: bool, final_actual_tick: int).
    """
    consec = 0
    t_start = time.time()

    while time.time() - t_start < timeout:
        pos = hand.read_pos()
        actual = pos[motor_idx] if pos[motor_idx] is not None else -1
        if abs(actual - target_tick) <= tol:
            consec += 1
            if consec >= n_consec:
                return True, actual
        else:
            consec = 0
        time.sleep(poll_interval)

    # Timed out — return whatever the motor ended up at
    pos = hand.read_pos()
    final = pos[motor_idx] if pos[motor_idx] is not None else -1
    return False, final


# ── Tick ↔ percentage conversion ─────────────────────────────────────────────

def pct_to_tick(pct: float, tensioned: int, curled: int) -> int:
    """Convert 0-100 % joint flexion to motor encoder ticks."""
    return int(round(tensioned + (pct / 100.0) * (curled - tensioned)))


def tick_to_pct(tick: int, tensioned: int, curled: int) -> float:
    """Convert motor encoder ticks to 0-100 % joint flexion."""
    span = curled - tensioned
    if span == 0:
        return 0.0
    return (tick - tensioned) / span * 100.0


# ── Core experiment ───────────────────────────────────────────────────────────

def run_experiment(
    hand: Hand,
    esp_conn: serial.Serial,
    sensor_map: dict,          # {sensor_idx(int): motor_id(int, 1-based)}
    targets_pct: list,         # list of float, 0-100 %
    t0: float,
    motor_filter: int | None,  # if not None, only test this motor_id
    n_samples: int,
    settle_tol: int,
    settle_n: int,
    settle_timeout: float,
    traj_len: int,
) -> list[dict]:
    """
    For every (motor, target) pair:
      1. Move motor to 0% first (reset / tensioned position).
      2. Move slowly to target % via move_to_pos interpolation.
      3. Wait for mechanical settling.
      4. Average n_samples AS5600 readings.
      5. Append record to log.

    Returns list of record dicts.
    """
    # Build motor → sensor list mapping
    motor_to_sensors: dict = {}
    for s_idx, m_id in sensor_map.items():
        motor_to_sensors.setdefault(m_id, []).append(s_idx)

    motors_to_test = sorted(motor_to_sensors.keys())
    if motor_filter is not None:
        motors_to_test = [m for m in motors_to_test if m == motor_filter]

    if not motors_to_test:
        print("[Warn] No motors to test. Check --sensor-map / --motor-id.")
        return []

    log = []
    n_motors = len(hand.motors)
    base_pos = np.array(hand.read_pos(), dtype=float)  # current full-hand position

    for motor_id in motors_to_test:
        motor_idx = motor_id - 1          # 0-based index into position array
        s_indices = motor_to_sensors[motor_id]
        tensioned = int(hand.tensioned_pos[motor_idx])
        curled    = int(hand.curled_bound[motor_idx])

        print(
            f"\n{'='*60}\n"
            f"[Motor {motor_id:2d}]  tensioned={tensioned}  curled={curled}\n"
            f"  sensors mapped: {s_indices}\n"
            f"  targets (%):    {targets_pct}\n"
            f"{'='*60}", flush=True
        )

        # ── Reset motor to 0 % (tensioned / open) ────────────────────────────
        reset_pos = base_pos.copy()
        reset_pos[motor_idx] = tensioned
        curr = np.array(hand.read_pos(), dtype=float)
        print(f"  [Reset] → 0 % (tick {tensioned})", flush=True)
        move_to_pos(curr_pos=curr, des_pos=reset_pos, hand=hand, traj_len=traj_len)
        time.sleep(0.3)

        for target_pct in targets_pct:
            target_tick = pct_to_tick(target_pct, tensioned, curled)

            # ── Move to target ────────────────────────────────────────────────
            curr = np.array(hand.read_pos(), dtype=float)
            target_pos = base_pos.copy()
            target_pos[motor_idx] = target_tick

            t_cmd = time.time()
            move_to_pos(curr_pos=curr, des_pos=target_pos, hand=hand, traj_len=traj_len)

            # ── Wait for settle ───────────────────────────────────────────────
            settled, actual_tick = wait_for_settle(
                hand, motor_idx, target_tick,
                tol=settle_tol, n_consec=settle_n, timeout=settle_timeout,
            )
            t_settle = time.time()

            # ── Read sensor (averaged) ────────────────────────────────────────
            sensor_avg = read_sensor_avg(esp_conn, n=n_samples)

            # ── Compute motor-estimated percentage from actual tick ────────────
            motor_pct = tick_to_pct(actual_tick, tensioned, curled)

            # ── Build record ──────────────────────────────────────────────────
            record_base = {
                "t_rel":         t_settle - t0,
                "motor_id":      motor_id,
                "target_pct":    round(target_pct, 2),
                "target_tick":   target_tick,
                "actual_tick":   actual_tick,
                "motor_pct":     round(motor_pct, 3),
                "settled":       int(settled),
            }

            for s_idx in s_indices:
                if s_idx in sensor_avg:
                    sa = sensor_avg[s_idx]
                    record_base[f"s{s_idx}_deg"] = round(sa["deg_avg"], 3)
                    record_base[f"s{s_idx}_raw"] = round(sa["raw_avg"], 1)
                    error = sa["deg_avg"] - motor_pct  # error in "%" / deg units
                    record_base[f"s{s_idx}_error"] = round(error, 3)
                else:
                    record_base[f"s{s_idx}_deg"]   = float("nan")
                    record_base[f"s{s_idx}_raw"]   = float("nan")
                    record_base[f"s{s_idx}_error"] = float("nan")

            log.append(record_base)

            status = "✓ settled" if settled else "✗ timeout"
            sensor_str = "  ".join(
                f"S{s}={record_base.get(f's{s}_deg', '?'):.2f}°"
                for s in s_indices
                if not isinstance(record_base.get(f"s{s}_deg"), float)
                   or not np.isnan(record_base.get(f"s{s}_deg", float("nan")))
            )
            print(
                f"  [{status}]  target={target_pct:5.1f}%  "
                f"tick cmd={target_tick} act={actual_tick}  "
                f"motor={motor_pct:5.1f}%  {sensor_str}",
                flush=True,
            )

        # ── Reset back to 0 % after each motor ───────────────────────────────
        reset_pos2 = base_pos.copy()
        reset_pos2[motor_idx] = tensioned
        curr = np.array(hand.read_pos(), dtype=float)
        move_to_pos(curr_pos=curr, des_pos=reset_pos2, hand=hand, traj_len=traj_len)
        time.sleep(0.3)

    return log


# ── Save helpers ──────────────────────────────────────────────────────────────

def save_log(log: list[dict], out_dir: str, label: str, t0: float):
    os.makedirs(out_dir, exist_ok=True)

    # ── t0 ────────────────────────────────────────────────────────────────────
    t0_path = os.path.join(out_dir, f"{label}_t0.txt")
    with open(t0_path, "w") as f:
        f.write(f"{t0:.6f}\n")

    if not log:
        print("[Save] Log is empty — nothing more to save.")
        return

    # ── Determine column order ────────────────────────────────────────────────
    # Fixed columns first, then dynamic sensor columns in sorted order
    fixed_cols = [
        "t_rel", "motor_id", "target_pct", "target_tick",
        "actual_tick", "motor_pct", "settled",
    ]
    sensor_cols = sorted(
        c for c in log[0].keys() if c not in fixed_cols
    )
    all_cols = fixed_cols + sensor_cols

    # ── CSV ───────────────────────────────────────────────────────────────────
    csv_path = os.path.join(out_dir, f"{label}_exp1.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_cols, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(log)
    print(f"[Save] CSV  → {csv_path}  ({len(log)} rows)", flush=True)

    # ── NPY (structured array) ────────────────────────────────────────────────
    # Build a simple float array; motor_id and settled stored as float too
    # so the dtype stays uniform (avoids structured-array complexity for plotting)
    mat = np.array([[row.get(c, float("nan")) for c in all_cols] for row in log],
                   dtype=np.float64)
    npy_path = os.path.join(out_dir, f"{label}_exp1.npy")
    np.save(npy_path, mat)

    # Also save column names as a companion text file for easy loading
    col_path = os.path.join(out_dir, f"{label}_exp1_columns.txt")
    with open(col_path, "w") as f:
        f.write("\n".join(all_cols) + "\n")

    print(f"[Save] NPY  → {npy_path}  shape={mat.shape}", flush=True)
    print(f"[Save] Cols → {col_path}", flush=True)

    # ── Pretty summary ────────────────────────────────────────────────────────
    print("\n── Summary ──────────────────────────────────────────────────────")
    print(f"  {'motor':>6}  {'target%':>8}  {'motor%':>8}  {'s_deg':>8}  {'error':>8}  {'settled':>7}")
    for row in log:
        mid  = int(row["motor_id"])
        tpct = row["target_pct"]
        mpct = row["motor_pct"]
        # pick first sensor for summary
        s_cols = [c for c in sensor_cols if c.endswith("_deg")]
        sdeg   = row.get(s_cols[0], float("nan")) if s_cols else float("nan")
        ecol   = s_cols[0].replace("_deg", "_error") if s_cols else None
        err    = row.get(ecol, float("nan")) if ecol else float("nan")
        sett   = "✓" if row["settled"] else "✗"
        print(f"  {mid:>6}  {tpct:>8.1f}  {mpct:>8.2f}  {sdeg:>8.2f}  {err:>8.2f}  {sett:>7}")
    print("─────────────────────────────────────────────────────────────────")


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Experiment 1: step motor through discrete positions, "
            "record AS5600 sensor angle at each, compute error."
        )
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
            "JSON mapping sensor_index → motor_id (1-based). "
            "Only motors present here are tested. "
            "Example: '{\"0\": 1, \"1\": 6, \"2\": 11}'"
        ),
    )
    parser.add_argument(
        "--motor-id",
        type=int, default=None,
        help="If set, only test this single motor_id (must be in --sensor-map).",
    )
    parser.add_argument(
        "--targets",
        nargs="+",
        type=float,
        default=[0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 100.0],
        help=(
            "Target positions as %% of full range (0=open, 100=curled). "
            "Default: 0 15 30 45 60 75 90 100"
        ),
    )
    parser.add_argument(
        "--hand-type",
        type=str, default="right",
        choices=["right", "left", "v1_right"],
        help="Hand type (default: right).",
    )
    parser.add_argument(
        "--n-samples",
        type=int, default=10,
        help="Number of AS5600 readings to average at each target (default: 10).",
    )
    parser.add_argument(
        "--settle-tol",
        type=int, default=20,
        help="Settling tolerance in encoder ticks (default: 20).",
    )
    parser.add_argument(
        "--settle-n",
        type=int, default=3,
        help="Consecutive reads within tolerance to declare settled (default: 3).",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float, default=5.0,
        help="Max seconds to wait for settling per step (default: 5.0).",
    )
    parser.add_argument(
        "--traj-len",
        type=int, default=30,
        help="Interpolation steps per move (default: 30; slower = more stable).",
    )
    parser.add_argument(
        "--out-dir",
        type=str, default="encoder_logs",
        help="Output directory (default: ./encoder_logs/).",
    )
    parser.add_argument(
        "--label",
        type=str, default="exp1",
        help="File label prefix (default: 'exp1').",
    )
    return parser.parse_args()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    raw_map    = json.loads(args.sensor_map)
    sensor_map = {int(k): int(v) for k, v in raw_map.items()}
    targets    = sorted(set(np.clip(args.targets, 0.0, 100.0)))

    print(f"[Config] Hand type   : {args.hand_type}")
    print(f"[Config] Serial port : {args.serial_port}")
    print(f"[Config] Sensor map  : {sensor_map}")
    print(f"[Config] Targets (%%) : {targets}")
    print(f"[Config] n_samples   : {args.n_samples}")
    print(f"[Config] Settle tol  : {args.settle_tol} ticks")
    print(f"[Config] Output dir  : {args.out_dir}")

    # ── Connect to hand ───────────────────────────────────────────────────────
    print("\n[Init] Connecting to Hand …", flush=True)
    hand = Hand(args.hand_type)

    # ── Connect to ESP32 ──────────────────────────────────────────────────────
    print(f"[Init] Opening serial port {args.serial_port} …", flush=True)
    esp_conn = serial.Serial(args.serial_port, args.baud, timeout=1.0)
    time.sleep(2.0)                      # wait for ESP32 boot
    esp_conn.reset_input_buffer()

    # Disable streaming; we'll use on-demand 'r' commands
    esp_conn.write(b"s")                 # toggle (assume was ON → now OFF)
    time.sleep(0.1)
    esp_conn.reset_input_buffer()
    # Send once more to ensure OFF regardless of boot state
    sample = read_sensor_once(esp_conn, retries=3)
    if not sample:
        # Streaming might still be on — try toggling again
        esp_conn.write(b"s")
        time.sleep(0.1)
        esp_conn.reset_input_buffer()

    # ── Sync point ────────────────────────────────────────────────────────────
    print("\n[SYNC] Recording t0 …", flush=True)
    t0 = time.time()
    print(f"[SYNC] t0 = {t0:.6f}", flush=True)

    # ── Run experiment ────────────────────────────────────────────────────────
    try:
        log = run_experiment(
            hand           = hand,
            esp_conn       = esp_conn,
            sensor_map     = sensor_map,
            targets_pct    = targets,
            t0             = t0,
            motor_filter   = args.motor_id,
            n_samples      = args.n_samples,
            settle_tol     = args.settle_tol,
            settle_n       = args.settle_n,
            settle_timeout = args.settle_timeout,
            traj_len       = args.traj_len,
        )
    finally:
        # Always restore streaming and close
        esp_conn.write(b"s")             # toggle streaming back ON
        esp_conn.close()
        hand.close()

    # ── Save ──────────────────────────────────────────────────────────────────
    save_log(log, args.out_dir, args.label, t0)
    print("\n[Done]", flush=True)


if __name__ == "__main__":
    main()
