import json
import matplotlib.pyplot as plt
import numpy as np
import os

# Path to the backup JSON
JSON_PATH = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders/calibration/right_structured_calibration_map.20260321_190039.bak.json"

def main():
    if not os.path.exists(JSON_PATH):
        print(f"Error: File not found: {JSON_PATH}")
        return

    with open(JSON_PATH, "r") as f:
        data = json.load(f)

    # Prepare data
    joints = data.get("joints", {})
    open_sensors = data.get("fully_open_sensors", {})
    live_limits = data.get("sensor_live_limits_deg", {})
    
    motor_ids = sorted([int(k) for k in joints.keys()])
    num_plots = 1 + len(motor_ids) + 1 # Open + Joints + Limits
    
    fig, axes = plt.subplots(num_plots, 1, figsize=(12, 4 * num_plots))
    if num_plots == 1:
        axes = [axes]
    
    # --- 1. Top Plot: Fully Open Sensors ---
    ax = axes[0]
    s_ids = sorted([int(k) for k in open_sensors.keys()])
    if s_ids:
        s_degs = [open_sensors[str(s)]["deg"] for s in s_ids]
        ax.bar([f"S{s}" for s in s_ids], s_degs, color="skyblue", edgecolor="navy")
        ax.set_title("1. Fully Open Sensors Readings", fontsize=14, fontweight='bold')
        ax.set_ylabel("Degrees")
        ax.grid(axis='y', ls='--', alpha=0.5)
        for j, v in enumerate(s_degs):
            ax.text(j, v + 2, f"{v:.1f}", ha='center', fontsize=9)

    # --- 2. Middle Plots: Per-Joint Closed Snaps ---
    for i, m_id in enumerate(motor_ids):
        ax = axes[i + 1]
        m_data = joints[str(m_id)]
        closed_data = m_data.get("sensor_readings_when_joint_closed", {})
        
        s_ids_j = sorted([int(k) for k in closed_data.keys()])
        if s_ids_j:
            s_degs = [closed_data[str(s)]["deg"] for s in s_ids_j]
            ax.bar([f"S{s}" for s in s_ids_j], s_degs, color="salmon", edgecolor="crimson")
            ax.set_title(f"Joint (Motor {m_id}) Closed Snapshot", fontsize=12, fontweight='bold')
            ax.set_ylabel("Degrees")
            ax.grid(axis='y', ls='--', alpha=0.4)
            for j, v in enumerate(s_degs):
                ax.text(j, v + 2, f"{v:.1f}", ha='center', fontsize=9)

    # --- 3. Final Plot: Sensor Live Limits ---
    ax = axes[-1]
    s_ids_l = sorted([int(k) for k in live_limits.keys()])
    if s_ids_l:
        labels = [f"S{s}" for s in s_ids_l]
        mins = [live_limits[str(s)]["min_deg"] for s in s_ids_l]
        maxs = [live_limits[str(s)]["max_deg"] for s in s_ids_l]
        heights = [mx - mn for mn, mx in zip(mins, maxs)]
        
        ax.bar(labels, heights, bottom=mins, color="lightgreen", edgecolor="darkgreen", alpha=0.8)
        ax.set_title("Sensor Live Range Limits (Min to Max Observed)", fontsize=14, fontweight='bold')
        ax.set_ylabel("Degrees")
        ax.grid(axis='y', ls='--', alpha=0.5)
        
        # Labels for min/max
        for j, (mn, mx) in enumerate(zip(mins, maxs)):
            ax.text(j, mn - 5, f"{mn:.1f}", ha='center', va='top', fontsize=8, color='darkgreen')
            ax.text(j, mx + 5, f"{mx:.1f}", ha='center', va='bottom', fontsize=8, color='darkgreen')

    plt.tight_layout()
    plt.suptitle(f"Calibration State: {os.path.basename(JSON_PATH)}", fontsize=18, fontweight='bold', y=1.0)
    plt.show()

if __name__ == "__main__":
    main()
