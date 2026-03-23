import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Normalize commanded vs sensor signals from a log.")
    parser.add_argument("--log", type=str, required=True, help="Path to the CSV log file to process")
    parser.add_argument("--out", type=str, default="normalized_plot.png", help="Output plot image filename in current dir")
    args = parser.parse_args()

    # Always use the manually filtered boundaries as the ground truth reference
    json_path = os.path.join(os.path.dirname(__file__), "..", "manually_filtered_paper_limits.json")

    if not os.path.exists(args.log):
        print(f"ERROR: Cannot find log file: {args.log}")
        sys.exit(1)
        
    if not os.path.exists(json_path):
        print(f"ERROR: Cannot find reference JSON file: {json_path}")
        sys.exit(1)

    with open(json_path, "r") as f:
        limits = json.load(f)

    df = pd.read_csv(args.log)

    print(f"Loaded {len(df)} rows from {args.log}")

    num_sensors = len(limits)
    fig, axes = plt.subplots(num_sensors, 1, figsize=(14, 4 * num_sensors), sharex=True)
    if num_sensors == 1:
        axes = [axes]

    # Typically time is t_rel, otherwise just use dataframe row index
    if "t_rel" in df.columns:
        x_axis = df["t_rel"]
        x_label = "Time (s)"
    else:
        x_axis = df.index
        x_label = "Packet Index"

    # Iterate over the JSON keys (sensory indices)
    plot_idx = 0
    for s_idx_str, config in limits.items():
        ax = axes[plot_idx]
        plot_idx += 1
        
        s_idx = int(s_idx_str)
        m_id = config["motor_id"]
        arr_idx = m_id - 1
        name = config["name"]

        cmd_col = f"cmd_{arr_idx}"
        deg_col = f"deg_{s_idx}"

        if cmd_col not in df.columns:
            print(f"WARNING: Commanded column {cmd_col} not found in log for Motor {m_id}.")
            continue
        if deg_col not in df.columns:
            print(f"WARNING: Sensor column {deg_col} not found in log for Sensor {s_idx}.")
            continue

        cmd_raw = df[cmd_col]
        deg_raw = df[deg_col]

        c_min = config["commanded_min_ticks"]
        c_max = config["commanded_max_ticks"]
        c_span = c_max - c_min

        d_min = config["filtered_sensor_min_deg"]
        d_max = config["filtered_sensor_max_deg"]
        d_span = d_max - d_min

        if c_span == 0:
            norm_cmd = np.zeros_like(cmd_raw, dtype=float)
        else:
            norm_cmd = (cmd_raw - c_min) / float(c_span)

        if d_span == 0:
            norm_deg = np.zeros_like(deg_raw, dtype=float)
        else:
            norm_deg = (deg_raw - d_min) / float(d_span)

        # Invert specific sensor signals so they track positively with their motors
        if s_idx in [0, 2, 4, 6]:
            norm_deg = 1.0 - norm_deg

        # Plot the normalized signals
        ax.plot(x_axis, norm_cmd, label=f"Norm. Command (M{m_id})", color="blue", linewidth=1.5, alpha=0.9)
        ax.plot(x_axis, norm_deg, label=f"Norm. Sensor (S{s_idx})", color="orange", linewidth=1.5, alpha=0.9)

        # Add grid, title, and legend
        ax.set_title(f"Sensor {s_idx} vs Motor {m_id} ({name})", fontweight='bold')
        ax.set_ylabel("Normalized [0, 1]")
        
        # Auto-scale upper bound specifically for Sensor 6 since it tends to over-span
        if s_idx == 6:
            s6_max = max(1.1, float(np.max(norm_deg)) * 1.1)
            ax.set_ylim(-0.1, s6_max)
        else:
            ax.set_ylim(-0.1, 1.1)
            
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right")

    axes[-1].set_xlabel(x_label)
    
    # h_pad heavily pads the vertical space specifically so titles don't get squished in GUI preview
    plt.tight_layout(pad=1.8, h_pad=2.5)
    plt.savefig(args.out, dpi=150, bbox_inches='tight')
    print(f"Success! Normalized comparison plot saved to: {args.out}")
    plt.show()

if __name__ == "__main__":
    main()
