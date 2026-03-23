import os
import pandas as pd
import matplotlib.pyplot as plt
from glob import glob

# Configuration
try:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_DIR = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders"
    
LOG_DIR = os.path.join(BASE_DIR, "encoder_logs")

# ======================================================================
# --- CHANGE THIS TARGET TO EXPLORE DIFFERENT SIGNALS ---
#
# Common Targets:
#  - Sensors: "deg_0", "deg_4", "deg_6"
#  - Motor Angles (Calculated): "act_angle_9", "act_angle_14"
#  - Motor Ticks (Raw): "act_9", "cmd_12"
# ======================================================================
TARGET_COLUMN = "deg_1"

# Number of recent logs to analyze side-by-side
NUM_LOGS = 15

def main():
    # Find unified logs
    unified_logs = glob(os.path.join(LOG_DIR, "*unified_log.csv"))
    if not unified_logs:
        print(f"No unified logs found in {LOG_DIR}.")
        return
        
    # Sort files by modification time (oldest to newest)
    unified_logs.sort(key=os.path.getmtime)
    
    # Grab the last `NUM_LOGS`
    recent_logs = unified_logs[-NUM_LOGS:]
    
    print(f"Analyzing signal: '{TARGET_COLUMN}'")
    print(f"Plotting the {len(recent_logs)} most recent logs side-by-side...")
    
    # -----------------------------------------------------
    # User Fix: Create historical ranges across ALL recent logs
    # -----------------------------------------------------
    print("Extracting global historical ranges for all tracking columns...")
    global_ranges = {}
    
    for log_path in recent_logs:
        df = pd.read_csv(log_path)
        if 't_rel' in df.columns:
            t_min = df['t_rel'].min()
            t_max = df['t_rel'].max()
            df = df[(df['t_rel'] >= t_min + 10.0) & (df['t_rel'] <= t_max - 10.0)]
            
        for col in df.columns:
            if col.startswith("deg_") or col.startswith("act_angle_"):
                c_min = float(df[col].min())
                c_max = float(df[col].max())
                
                if col not in global_ranges:
                    global_ranges[col] = {"min": c_min, "max": c_max}
                else:
                    global_ranges[col]["min"] = min(global_ranges[col]["min"], c_min)
                    global_ranges[col]["max"] = max(global_ranges[col]["max"], c_max)

    import json
    hist_out = os.path.join(BASE_DIR, "historical_sensor_ranges.json")
    with open(hist_out, "w") as f:
        json.dump(global_ranges, f, indent=4)
    print(f"-> Saved universal historical bounds to {hist_out}")
    
    # Setup the plot
    fig, axes = plt.subplots(len(recent_logs), 1, figsize=(14, 3 * len(recent_logs)), sharex=False, sharey=True)
    if len(recent_logs) == 1:
        axes = [axes]  # Ensure it's iterable
        
    for i, log_path in enumerate(recent_logs):
        log_name = os.path.basename(log_path)
        df = pd.read_csv(log_path)
        
        # -----------------------------------------------------
        # User Fix: Ignore the first and last 10 seconds of data
        # -----------------------------------------------------
        if 't_rel' in df.columns:
            t_min = df['t_rel'].min()
            t_max = df['t_rel'].max()
            df = df[(df['t_rel'] >= t_min + 10.0) & (df['t_rel'] <= t_max - 10.0)].copy()
        
        ax = axes[i]
        
        if TARGET_COLUMN not in df.columns:
            ax.text(0.5, 0.5, f"Column '{TARGET_COLUMN}' not found in this log", 
                    ha='center', va='center', transform=ax.transAxes, color='red', fontsize=12)
            ax.set_title(f"Log {i+1}: {log_name}", fontsize=11, fontweight='bold')
            continue
            
        t = df['t_rel']
        vals = df[TARGET_COLUMN]
        
        # Plot the data
        ax.plot(t, vals, label=TARGET_COLUMN, color="purple", lw=1.5)
        
        # Calculate some quick stats to help judge noise & range
        v_min = vals.min()
        v_max = vals.max()
        v_span = v_max - v_min
        
        ax.set_title(f"Log {i+1}: {log_name} | Span: {v_span:.2f}° (Min: {v_min:.2f}°, Max: {v_max:.2f}°)", fontsize=11, fontweight='bold')
        ax.set_ylabel("Measured Value", fontsize=10)
        ax.grid(True, alpha=0.4)
        ax.legend(loc="upper right", fontsize=9)
        
    axes[-1].set_xlabel("Time (s) within log", fontsize=12)
    
    # Align labels vertically so they don't stagger based on tick widths
    fig.align_ylabels(axes)
    
    plt.suptitle(f"Historical Consistency & Noise Analysis for '{TARGET_COLUMN}'", fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)  # Make room for the main super title
    plt.show()

if __name__ == "__main__":
    main()
