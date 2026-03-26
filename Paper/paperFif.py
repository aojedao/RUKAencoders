import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Normalize commanded vs sensor signals from a log, splitting by phase.")
    parser.add_argument("--log", type=str, default="selectedBest.csv", help="Path to the CSV log file to process")
    parser.add_argument("--out", type=str, default="normalized_plot_phase.png", help="Output plot image filename in current dir")
    args = parser.parse_args()

    json_path = os.path.join(os.path.dirname(__file__), "..", "manually_filtered_paper_limits_redo.json")

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

    # Process only specific sensors: Action, Index PIP, Index MCP, Thumb CMC
    # 0: Index Abduction, 2: Index PIP, 3: Index MCP, 4: Thumb CMC
    target_sensors = ["4", "5", "0", "2"]
    
    fig, axes = plt.subplots(2, 2, figsize=(25, 10), sharex=True)
    axes_flat = axes.flatten()

    # Initialize x_label in case it's missed
    x_label = "Time (s)"

    plot_idx = 0
    for s_idx_str in target_sensors:
        if s_idx_str not in limits:
            continue
            
        config = limits[s_idx_str]
        ax = axes_flat[plot_idx]
        plot_idx += 1
        
        s_idx = int(s_idx_str)
        m_id = config["motor_id"]
        arr_idx = m_id - 1
        name = config["name"]

        # Find the phase matching this joint (e.g., "Index Abduction" -> "Index_Abduction")
        phase_substr = name.replace(" ", "_")
        
        # Filter dataframe for phase
        df_sub = df[df["phase"].str.contains(phase_substr, na=False)].copy()
        
        if df_sub.empty:
            print(f"WARNING: No data found for phase {phase_substr} in log.")
            continue
            
        if "t_rel" in df_sub.columns:
            # Normalize time axis to start at 0
            x_axis = df_sub["t_rel"] - df_sub["t_rel"].iloc[0]
            x_label = "Time (s)"
        else:
            x_axis = np.arange(len(df_sub))
            x_label = "Packet Index"

        cmd_col = f"cmd_{arr_idx}"
        deg_col = f"deg_{s_idx}"

        if cmd_col not in df_sub.columns:
            print(f"WARNING: Commanded column {cmd_col} not found in log for Motor {m_id}.")
            continue
        if deg_col not in df_sub.columns:
            print(f"WARNING: Sensor column {deg_col} not found in log for Sensor {s_idx}.")
            continue

        cmd_raw = df_sub[cmd_col]
        deg_raw = df_sub[deg_col]

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
            
        # Convert back to degrees for the plots based on minimum and max bounds
        deg_cmd = norm_cmd * d_span + d_min
        deg_act = norm_deg * d_span + d_min

        # Plot the deg signals with alpha/theta labels
        ax.plot(x_axis, deg_cmd, label=r"Commanded", color="blue", linewidth=1.5, alpha=0.9)
        ax.plot(x_axis, deg_act, label=r"Actual", color="orange", linewidth=1.5, alpha=0.9)

        # Add grid, title, and legend
        ax.set_title(f"Joint Angle Comparison vs Time ({name})", fontweight='bold')
        ax.set_ylabel("Angle\n(deg)", rotation=0, labelpad=30, va="center")
        
        # Adjust Y limits
        if s_idx == 2:  # Index PIP - Make it larger explicitly
            ax.set_ylim(d_min - 5, d_max + 20)
        elif s_idx == 4: # Thumb CMC - Specific override
            ax.set_ylim(90, 225)
        else:
            ax.set_ylim(d_min - 5, d_max + 5)
            
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right")

    # Set x labels only for the bottom row
    axes[1, 0].set_xlabel(x_label)
    axes[1, 1].set_xlabel(x_label)
    
    plt.tight_layout(pad=1.8, h_pad=2.5)
    plt.savefig(args.out, dpi=150, bbox_inches='tight')
    print(f"Success! Normalized comparison plot saved to: {args.out}")
    # Don't show the plot to avoid blocking the script
    # plt.show()

if __name__ == "__main__":
    main()
