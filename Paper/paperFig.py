import argparse
import json
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Map the joint names from the JSON to the specific greek subindices requested
JOINT_SUBINDICES = {
    0: "iabd",  # Index Abduction
    1: "idip",  # Index DIP
    2: "ipip",  # Index PIP
    3: "imcp",  # Index MCP
    4: "tcmc",  # Thumb CMC
    5: "tmcp",  # Thumb MCP
    6: "tdip",  # Thumb DIP
}

def main():
    parser = argparse.ArgumentParser(description="Crop and plot specifically active testing phases per sensor.")
    parser.add_argument("--log", type=str, default="selectedBest.csv", help="Path to the unified CSV to process")
    parser.add_argument("--out", type=str, default="paperFig_result.png", help="Output plot image filename")
    args = parser.parse_args()

    # Always use the manually filtered boundaries as the ground truth reference
    json_path = os.path.join(os.path.dirname(__file__), "..", "manually_filtered_paper_limits_redo.json")

    # If the user provides a relative path like "selectedBest.csv", resolve it locally in Paper/ first
    if not os.path.exists(args.log):
        local_log = os.path.join(os.path.dirname(__file__), args.log)
        if os.path.exists(local_log):
            args.log = local_log
        else:
            print(f"ERROR: Cannot find log file: {args.log}")
            sys.exit(1)
            
    if not os.path.exists(json_path):
        print(f"ERROR: Cannot find reference JSON file: {json_path}")
        sys.exit(1)

    with open(json_path, "r") as f:
        limits = json.load(f)

    df_full = pd.read_csv(args.log)
    print(f"Loaded {len(df_full)} rows from {args.log}")

    num_sensors = len(limits)
    fig, axes = plt.subplots(num_sensors, 1, figsize=(14, 3 * num_sensors), sharex=True)
    if num_sensors == 1:
        axes = [axes]

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

        # Filter the DataFrame down to ONLY the active test phase for this specific sensor
        if "phase" in df_full.columns:
            # We look for the phase tag strictly ending in _S# to grab all 20 points
            df = df_full[df_full['phase'].str.endswith(f"_S{s_idx}", na=False)].copy()
        else:
            df = df_full.copy()
            
        if df.empty:
            ax.text(0.5, 0.5, f"No active phase data found for {name} (S{s_idx})", 
                    ha='center', va='center', color='red', transform=ax.transAxes)
            continue

        if cmd_col not in df.columns or deg_col not in df.columns:
            ax.text(0.5, 0.5, f"Missing Data for {name}", ha='center', va='center', color='red', transform=ax.transAxes)
            continue

        if "t_rel" in df.columns:
            # Shift the time window to start exactly at 0 so all durations overlay perfectly
            x_axis = df["t_rel"] - df["t_rel"].min()
            x_label = "Test Phase Duration (s)"
        else:
            x_axis = np.arange(len(df))
            x_label = "Relative Packet Index"
            
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
            
        subindex = JOINT_SUBINDICES.get(s_idx, f"s{s_idx}")

        # Plot the normalized signals using LaTeX formatting for Theta (Motor) and Alpha (Sensor)
        ax.plot(x_axis, norm_cmd, label=rf"$\theta_{{{subindex}}}$", color="blue", linewidth=2.0, alpha=0.9)
        ax.plot(x_axis, norm_deg, label=rf"$\alpha_{{{subindex}}}$", color="orange", linewidth=2.0, alpha=0.9)

        # Plot Styling
        # Title is now placed discreetly inside the plot or just left clean
        ax.set_ylabel(name, fontsize=12, fontweight='bold', rotation=0, labelpad=50, va='center')
        
        # Auto-scale upper bound specifically for Sensor 6
        if s_idx == 6:
            s6_max = max(1.1, float(np.max(norm_deg)) * 1.1)
            ax.set_ylim(-0.1, s6_max)
        else:
            ax.set_ylim(-0.1, 1.1)
            
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=11)

    axes[-1].set_xlabel(x_label, fontsize=12)
    
    # Place the "Normalized Tracking" descriptor in the overall master title
    plt.suptitle("Normalized Phase-Isolated Joint Tracking", fontsize=16, fontweight='bold', y=0.98)
    
    # Adjust tight layout
    plt.tight_layout(pad=1.5, h_pad=1.0)
    
    plt.savefig(args.out, dpi=200, bbox_inches='tight')
    print(f"Success! Cropped tracking plot saved to: {args.out}")
    plt.show()

if __name__ == "__main__":
    main()
