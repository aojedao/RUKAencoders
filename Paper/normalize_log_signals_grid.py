import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Grid up to 6 logs into a 2x3 matrix of figures.")
    parser.add_argument("--logs", nargs='+', required=True, help="List of up to 6 CSV log files")
    parser.add_argument("--out", type=str, default="normalized_grid_plot.png", help="Output plot image filename")
    args = parser.parse_args()

    # Always use the manually filtered boundaries as the ground truth reference
    json_path = os.path.join(os.path.dirname(__file__), "..", "manually_filtered_paper_limits_redo.json")

    if not os.path.exists(json_path):
        print(f"ERROR: Cannot find reference JSON file: {json_path}")
        sys.exit(1)

    with open(json_path, "r") as f:
        limits = json.load(f)

    num_sensors = len(limits)
    
    # Restrict to maximum 6 logs to conform to the 2x3 grid strict requirement
    logs = args.logs[:6]
    
    print(f"Compiling {len(logs)} log files into a 2x3 master grid...")
    
    # Create the master figure
    # We use a massive figure to hold 6 entire mini-figures
    fig = plt.figure(figsize=(30, 20))
    
    # Subfigures are perfect for this: they create independent 7-stack subplots inside each grid cell!
    subfigs = fig.subfigures(2, 3, wspace=0.03, hspace=0.05).flatten()

    for log_idx, log_file in enumerate(logs):
        if not os.path.exists(log_file):
            print(f"Skipping missing file: {log_file}")
            continue
            
        subfig = subfigs[log_idx]
        df = pd.read_csv(log_file)
        
        # Add the log file name as the master title for this specific cell
        log_name = os.path.basename(log_file).replace(".csv", "")
        # Keep the title smaller and perfectly horizontal to avoid covering the plots
        subfig.suptitle(f"Log {log_idx + 1}: {log_name}", fontsize=11, fontweight='bold', color='navy', y=0.98)
        
        # Now create the standard 7-sensor stack INSIDE this subfigure
        axes = subfig.subplots(num_sensors, 1, sharex=True)
        if num_sensors == 1:
            axes = [axes]
            
        # Physically carve out 8% of the top spacing exclusively for the title
        subfig.subplots_adjust(top=0.92, hspace=0.4)
            
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

            if cmd_col not in df.columns or deg_col not in df.columns:
                ax.text(0.5, 0.5, "Missing Data", ha='center', va='center', color='red', transform=ax.transAxes)
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

            # --- User Fix: Invert specified sensors ---
            if s_idx in [0, 2, 4, 6]:
                norm_deg = 1.0 - norm_deg

            # Plot the normalized signals
            ax.plot(x_axis, norm_cmd, label=f"Cmd S{s_idx}", color="blue", linewidth=1.5, alpha=0.9)
            ax.plot(x_axis, norm_deg, label=f"Act S{s_idx}", color="orange", linewidth=1.5, alpha=0.9)

            # Minimalist titles so 42 plots don't look overly cluttered
            ax.set_title(f"S{s_idx} vs M{m_id} ({name})", fontsize=10, fontweight='bold', pad=3)
            
            # Auto-scale upper bound specifically for Sensor 6
            if s_idx == 6:
                s6_max = max(1.1, float(np.max(norm_deg)) * 1.1)
                ax.set_ylim(-0.1, s6_max)
            else:
                ax.set_ylim(-0.1, 1.1)
                
            ax.grid(alpha=0.3)
            
            # Print Y-labels only for the extreme left column of the 2x3 grid to save space
            if log_idx in [0, 3]:
                ax.set_ylabel("[0, 1]", fontsize=9)
            else:
                ax.set_yticklabels([])
                
            # Only put a legend on the very top-left graph to save drawing space
            if log_idx == 0 and plot_idx == 1:
                ax.legend(loc="upper right", fontsize=8)

        axes[-1].set_xlabel(x_label, fontsize=10)

    # Empty unfilled subfigures gracefully
    for i in range(len(logs), 6):
        axes = subfigs[i].subplots(1, 1)
        axes.axis('off')
        axes.text(0.5, 0.5, "Empty Slot", ha='center', va='center', color='gray', fontsize=16)

    plt.savefig(args.out, dpi=200, bbox_inches='tight')
    print(f"Success! Highly dense 2x3 log grid saved to: {args.out}")
    plt.show()

if __name__ == "__main__":
    main()
