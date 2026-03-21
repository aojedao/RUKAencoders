"""process_old_logs.py
=====================
Reprocess old-format log files through ``motor_pos_to_joint_angles`` and produce:

  1. A unified CSV  (<label>_processed.csv) with columns:
       t_rel,
       cmd_0..15, act_0..15,
       ang_cmd_0..15  (joint angles inferred from commanded positions, degrees),
       ang_act_0..15  (joint angles inferred from actual   positions, degrees),
       sensor_t_rel, raw_0..2, deg_0..2

  2. Comparison plots: recovered joint angles (commanded & actual) vs sensor
     degrees for the sensors that map to a known motor.

Supported input formats
-----------------------
  auto (default)  – tries .npy first, then .csv
  npy             – <label>_motor_log.npy + <label>_sensor_log.npy
  csv             – <label>_motor_log.csv + <label>_sensor_log.csv

Hand-bounds lookup order
------------------------
  1. <label>_hand_bounds.npz  in --out-dir
  2. Any other *_hand_bounds.npz found in --out-dir  (printed as a warning)
  3. Abort with a helpful message

Usage
-----
  # Auto-detect format, 'run' label:
  python process_old_logs.py

  # Explicit label / directory / format:
  python process_old_logs.py --label first_test --out-dir encoder_logs --hand left
  python process_old_logs.py --label run --format npy --no-plot

  # Override which bounds file to use:
  python process_old_logs.py --label first_test --bounds-file encoder_logs/run_hand_bounds.npz
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
from types import SimpleNamespace

# ── re-use helpers from replay_encoders ──────────────────────────────────────
from replay_encoders import motor_pos_to_joint_angles, N_MOTORS, N_SENSORS

# Sensors-to-motor index mapping (same as in replay_encoders.py)
SENSOR_MOTOR_MAP = {
    0: 8,   # Sensor 0 → motor index 8  (index MCP)
    1: 7,   # Sensor 1 → motor index 7  (index splay)
    2: 6,   # Sensor 2 → motor index 6  (index PIP/DIP)
}

# Joint / finger names for nicer axis labels
MOTOR_NAMES = [
    "pinky_mcp(0)", "pinky_splay(1)", "pinky_pip(2)",
    "ring_splay(3)", "ring_mcp(4)",   "ring_pip(5)",
    "index_pip(6)", "index_splay(7)", "index_mcp(8)",
    "mid_pip(9)",   "mid_mcp(10)",    "thumb_ip(11)",
    "thumb_cmc(12)","thumb_mcp(13)", "wrist_yaw(14)", "wrist_pitch(15)",
]


# ── loaders ──────────────────────────────────────────────────────────────────

def load_hand_bounds(out_dir: str, label: str, bounds_file: str | None = None):
    """Load hand-calibration bounds from a .npz file.

    Search order
    ------------
    1. ``bounds_file``  if explicitly provided via CLI.
    2. ``<out_dir>/<label>_hand_bounds.npz``  (label-specific).
    3. Any ``*_hand_bounds.npz`` found in ``out_dir``  (with a warning).
    """
    # 1. Explicit override
    if bounds_file:
        path = bounds_file
        if not os.path.exists(path):
            raise FileNotFoundError(f"Specified bounds file not found: {path}")
        print(f"[Load] Hand bounds (explicit) ← {path}", flush=True)

    else:
        # 2. Label-specific file
        path = os.path.join(out_dir, f"{label}_hand_bounds.npz")
        if not os.path.exists(path):
            # 3. Fallback: any bounds file in the directory
            candidates = [
                f for f in os.listdir(out_dir)
                if f.endswith("_hand_bounds.npz")
            ]
            if not candidates:
                raise FileNotFoundError(
                    f"No hand-bounds file found for label '{label}' in '{out_dir}'.\n"
                    "Run a live session first (saves hand_bounds.npz automatically), or\n"
                    "use --bounds-file <path> to specify one explicitly."
                )
            path = os.path.join(out_dir, sorted(candidates)[0])
            print(
                f"[Load] WARNING: '{label}_hand_bounds.npz' not found; "
                f"falling back to → {path}",
                flush=True,
            )
        else:
            print(f"[Load] Hand bounds ← {path}", flush=True)

    data = np.load(path)
    return SimpleNamespace(
        tensioned_pos=np.array(data["tensioned_pos"], dtype=float),
        curled_bound=np.array(data["curled_bound"],   dtype=float),
    )


# ── .npy loaders ─────────────────────────────────────────────────────────────

def load_motor_npy(out_dir: str, label: str):
    """Load motor log from a .npy structured array.

    The array must have fields: t_rel (float64), commanded (int32, 16),
    actual (int32, 16) — as saved by replay_encoders._save_motor_log_npy.
    """
    path = os.path.join(out_dir, f"{label}_motor_log.npy")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Motor log .npy not found: {path}")
    arr = np.load(path)
    rows = [
        {
            "t_rel":     float(arr[i]["t_rel"]),
            "commanded": list(arr[i]["commanded"].astype(int)),
            "actual":    list(arr[i]["actual"].astype(int)),
        }
        for i in range(len(arr))
    ]
    print(f"[Load] Motor log  ← {path}  ({len(rows)} rows)", flush=True)
    return rows


def load_sensor_npy(out_dir: str, label: str):
    """Load sensor log from a .npy structured array.

    The array must have fields: t_rel (float64), raw (int32, N_SENSORS),
    deg (float32, N_SENSORS) — as saved by replay_encoders._save_sensor_log_npy.
    """
    path = os.path.join(out_dir, f"{label}_sensor_log.npy")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Sensor log .npy not found: {path}")
    arr = np.load(path)
    rows = [
        {
            "t_rel": float(arr[i]["t_rel"]),
            "raw":   list(arr[i]["raw"].astype(int)),
            "deg":   list(arr[i]["deg"].astype(float)),
        }
        for i in range(len(arr))
    ]
    print(f"[Load] Sensor log ← {path}  ({len(rows)} rows)", flush=True)
    return rows


# ── .csv loaders ──────────────────────────────────────────────────────────────

def load_motor_csv(out_dir: str, label: str):
    """Load motor log CSV.

    Accepts both old-format (only cmd/act columns) and new unified-format
    CSVs (which also contain ang_cmd/ang_act columns — those are ignored).

    Expected header (minimum):
        t_rel, cmd_0..cmd_15, act_0..act_15
    """
    path = os.path.join(out_dir, f"{label}_motor_log.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Motor log CSV not found: {path}")
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "t_rel":     float(row["t_rel"]),
                "commanded": [int(float(row[f"cmd_{i}"])) for i in range(N_MOTORS)],
                "actual":    [int(float(row[f"act_{i}"])) for i in range(N_MOTORS)],
            })
    print(f"[Load] Motor log  ← {path}  ({len(rows)} rows)", flush=True)
    return rows


def load_sensor_csv(out_dir: str, label: str):
    """Load sensor log CSV.

    Expected header (minimum):
        t_rel, raw_0..raw_{N-1}, deg_0..deg_{N-1}
    """
    path = os.path.join(out_dir, f"{label}_sensor_log.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Sensor log CSV not found: {path}")
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "t_rel": float(row["t_rel"]),
                "raw":   [int(float(row[f"raw_{i}"]))   for i in range(N_SENSORS)],
                "deg":   [float(row[f"deg_{i}"]) for i in range(N_SENSORS)],
            })
    print(f"[Load] Sensor log ← {path}  ({len(rows)} rows)", flush=True)
    return rows


# ── auto-detect loader ────────────────────────────────────────────────────────

def load_logs(out_dir: str, label: str, fmt: str = "auto"):
    """Load motor + sensor logs, auto-detecting format when fmt='auto'.

    Parameters
    ----------
    fmt : {'auto', 'npy', 'csv'}
        'auto'  – tries .npy first; falls back to .csv.
        'npy'   – loads .npy structured arrays only.
        'csv'   – loads .csv files only.
    """
    if fmt == "npy":
        return load_motor_npy(out_dir, label), load_sensor_npy(out_dir, label)

    if fmt == "csv":
        return load_motor_csv(out_dir, label), load_sensor_csv(out_dir, label)

    # auto: try npy first
    npy_motor  = os.path.join(out_dir, f"{label}_motor_log.npy")
    npy_sensor = os.path.join(out_dir, f"{label}_sensor_log.npy")
    csv_motor  = os.path.join(out_dir, f"{label}_motor_log.csv")
    csv_sensor = os.path.join(out_dir, f"{label}_sensor_log.csv")

    if os.path.exists(npy_motor) and os.path.exists(npy_sensor):
        print("[Load] Auto-detected format: .npy", flush=True)
        return load_motor_npy(out_dir, label), load_sensor_npy(out_dir, label)

    if os.path.exists(csv_motor) and os.path.exists(csv_sensor):
        print("[Load] Auto-detected format: .csv", flush=True)
        return load_motor_csv(out_dir, label), load_sensor_csv(out_dir, label)

    raise FileNotFoundError(
        f"No motor/sensor log files found for label '{label}' in '{out_dir}'.\n"
        f"Tried:\n  {npy_motor}\n  {csv_motor}"
    )


# ── nearest-neighbour join ───────────────────────────────────────────────────

def _nearest_sensor(t, sensor_times_arr, sensor_records):
    if len(sensor_times_arr) == 0:
        return None
    idx = int(np.argmin(np.abs(sensor_times_arr - t)))
    return sensor_records[idx]


# ── unified CSV writer ───────────────────────────────────────────────────────

def save_unified_csv(motor_rows, sensor_rows, hand_bounds, out_dir, label):
    sensor_times_arr = np.array([r["t_rel"] for r in sensor_rows]) if sensor_rows else np.array([])

    cmd_cols     = [f"cmd_{i}"     for i in range(N_MOTORS)]
    act_cols     = [f"act_{i}"     for i in range(N_MOTORS)]
    ang_cmd_cols = [f"ang_cmd_{i}" for i in range(N_MOTORS)]
    ang_act_cols = [f"ang_act_{i}" for i in range(N_MOTORS)]
    raw_cols     = [f"raw_{i}"     for i in range(N_SENSORS)]
    deg_cols     = [f"deg_{i}"     for i in range(N_SENSORS)]

    header = (
        ["t_rel"]
        + cmd_cols + act_cols
        + ang_cmd_cols + ang_act_cols
        + ["sensor_t_rel"] + raw_cols + deg_cols
    )

    out_path = os.path.join(out_dir, f"{label}_processed.csv")
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for row in motor_rows:
            t_rel     = row["t_rel"]
            commanded = list(row["commanded"])
            actual    = list(row["actual"])

            ang_cmd = [f"{v:.4f}" for v in motor_pos_to_joint_angles(commanded, hand_bounds)]
            ang_act = [f"{v:.4f}" for v in motor_pos_to_joint_angles(actual,    hand_bounds)]

            s = _nearest_sensor(t_rel, sensor_times_arr, sensor_rows)
            if s is not None:
                sensor_t = f"{s['t_rel']:.6f}"
                raw_vals = [str(v) for v in s["raw"]]
                deg_vals = [f"{d:.4f}" for d in s["deg"]]
            else:
                sensor_t = "nan"
                raw_vals = ["nan"] * N_SENSORS
                deg_vals = ["nan"] * N_SENSORS

            writer.writerow(
                [f"{t_rel:.6f}"]
                + [str(v) for v in commanded]
                + [str(v) for v in actual]
                + ang_cmd + ang_act
                + [sensor_t] + raw_vals + deg_vals
            )

    print(f"[Save] Unified CSV → {out_path}  ({len(motor_rows)} rows)", flush=True)
    return out_path


# ── comparison plots ─────────────────────────────────────────────────────────

def plot_comparison(motor_rows, sensor_rows, hand_bounds, out_dir, label, hand_type):
    """Plot recovered joint angles vs AS5600 sensor readings for mapped motors."""
    motor_times = np.array([r["t_rel"] for r in motor_rows])
    commanded   = np.array([r["commanded"] for r in motor_rows], dtype=float)
    actual      = np.array([r["actual"]    for r in motor_rows], dtype=float)

    sensor_times = np.array([r["t_rel"] for r in sensor_rows])
    sensor_deg   = np.array([r["deg"]   for r in sensor_rows])

    # Direction correction for right hand sensor 0
    if hand_type == "right":
        sensor_deg[:, 0] *= -1

    # Compute inverted joint angles for every motor row
    ang_cmd = np.array([motor_pos_to_joint_angles(cmd, hand_bounds) for cmd in commanded])
    ang_act = np.array([motor_pos_to_joint_angles(act, hand_bounds) for act in actual])

    n_plots = len(SENSOR_MOTOR_MAP)
    fig, axes = plt.subplots(n_plots, 1, figsize=(13, 4.5 * n_plots))
    if n_plots == 1:
        axes = [axes]

    for ax, (s_idx, m_idx) in zip(axes, SENSOR_MOTOR_MAP.items()):
        m_name = MOTOR_NAMES[m_idx]

        ax.plot(motor_times, ang_cmd[:, m_idx],
                color="steelblue", lw=2,   label=f"ang_cmd_{m_idx} (inverted from commanded)", alpha=0.9)
        ax.plot(motor_times, ang_act[:, m_idx],
                color="darkorange", lw=1.5, ls="--", label=f"ang_act_{m_idx}  (inverted from actual)", alpha=0.85)
        ax.plot(sensor_times, sensor_deg[:, s_idx],
                color="forestgreen", lw=1.5, label=f"Sensor {s_idx} (AS5600, deg)", alpha=0.8)

        ax.set_xlabel("Time (s)", fontsize=11)
        ax.set_ylabel("Angle (°)", fontsize=11)
        ax.set_title(
            f"Sensor {s_idx}  ↔  Motor {m_idx}: {m_name}\n"
            f"— Recovered joint angle vs AS5600 reading",
            fontsize=12, fontweight="bold"
        )
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(out_dir, f"{label}_processed_angle_comparison.png")
    plt.savefig(plot_path, dpi=150, bbox_inches="tight")
    print(f"[Plot] Saved → {plot_path}", flush=True)
    plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description=(
            "Reprocess motor_log + sensor_log (npy or csv) through "
            "motor_pos_to_joint_angles and produce a unified CSV + comparison plots."
        )
    )
    p.add_argument("--label",       type=str, default="run",
                   help="File prefix to load, e.g. 'run', 'first_test' (default: 'run').")
    p.add_argument("--out-dir",     type=str, default="encoder_logs",
                   help="Directory containing the log files (default: ./encoder_logs/).")
    p.add_argument("--hand",        type=str, default="left", choices=["left", "right"],
                   help="Hand type for sensor direction correction (default: left).")
    p.add_argument("--format",      type=str, default="auto", choices=["auto", "npy", "csv"],
                   help="Input file format: 'auto' (default), 'npy', or 'csv'.")
    p.add_argument("--bounds-file", type=str, default=None,
                   help="Explicit path to a *_hand_bounds.npz file (overrides label-based lookup).")
    p.add_argument("--no-plot",     action="store_true",
                   help="Skip generating comparison plots.")
    return p.parse_args()


def main():
    args  = parse_args()
    label = args.label
    out   = args.out_dir

    print(
        f"\n[Config] label={label}  out-dir={out}  "
        f"hand={args.hand}  format={args.format}\n",
        flush=True,
    )

    hand_bounds          = load_hand_bounds(out, label, bounds_file=args.bounds_file)
    motor_rows, sensor_rows = load_logs(out, label, fmt=args.format)

    csv_path = save_unified_csv(motor_rows, sensor_rows, hand_bounds, out, label)
    print(f"\n[Done] Unified CSV written: {csv_path}")

    if not args.no_plot:
        print("\n[Plot] Generating comparison plots …", flush=True)
        plot_comparison(motor_rows, sensor_rows, hand_bounds, out, label, args.hand)
        print("[Done] Plots saved.", flush=True)


if __name__ == "__main__":
    main()
