# %%
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from glob import glob
from live_calibration import RIGHT_HAND_JOINT_GROUPS

# Configuration
try:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_DIR = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders"
    
LOG_DIR = os.path.join(BASE_DIR, "encoder_logs")

# ======================================================================
# CONFIGURATION
#
# Set this to False to hide the Error comparison column entirely:
PLOT_ERROR_COLUMN = True
#
# Set this to False to hide the specific log filename from the main plot title:
SHOW_LOG_TITLE = False
#
# Choose which log to plot. Set to "latest" to automatically grab the newest log,
# or set to a specific filename, e.g., "sensor_recap__20260321_183412_unified_log.csv"
TARGET_LOG = "latest"
# ======================================================================


# %%
# Load Theoretical Mapping
reference_data = {}
for g in RIGHT_HAND_JOINT_GROUPS:
    motor_val = g["motor"]
    
    # User Fix: Swap motors 8 and 9 explicitly
    if motor_val == 8:
        motor_val = 9
    elif motor_val == 9:
        motor_val = 8
        
    reference_data[g["joint"]] = {
        "sensor": g["sensor"],
        "motor": motor_val,
        "expected_span": g["expected_abs_span_deg"]
    }

# %%
# Load Latest Log and Generate Error Report
unified_logs = glob(os.path.join(LOG_DIR, "*unified_log.csv"))
if not unified_logs:
    print("\nNo unified logs found.")
else:
    # Resolve target log file
    if TARGET_LOG == "latest":
        latest_unified_path = max(unified_logs, key=os.path.getmtime)
    else:
        latest_unified_path = os.path.join(LOG_DIR, TARGET_LOG)
        if not os.path.exists(latest_unified_path):
            raise FileNotFoundError(f"Specified target log does not exist: {latest_unified_path}")
            
    latest_unified = os.path.basename(latest_unified_path)
    print(f"Generating Error Report for log: {latest_unified}")
    
    import json
    hist_path = os.path.join(BASE_DIR, "historical_sensor_ranges.json")
    global_ranges = {}
    if os.path.exists(hist_path):
        with open(hist_path, "r") as f:
            global_ranges = json.load(f)
        print(f"Loaded historical ranges from {os.path.basename(hist_path)}")
    else:
        print("WARNING: historical_sensor_ranges.json not found! Falling back to intra-log scaling.")
        
    unified_df = pd.read_csv(latest_unified_path)
    
    # -----------------------------------------------------
    # User Fix: Ignore the first and last 10 seconds of data
    # -----------------------------------------------------
    if 't_rel' in unified_df.columns:
        t_min = unified_df['t_rel'].min()
        t_max = unified_df['t_rel'].max()
        unified_df = unified_df[(unified_df['t_rel'] >= t_min + 10.0) & (unified_df['t_rel'] <= t_max - 10.0)].copy()
    
    num_joints = len(reference_data)
    
    # Create the subplot grid based on PLOT_ERROR_COLUMN toggle
    num_cols = 2 if PLOT_ERROR_COLUMN else 1
    fig, axes = plt.subplots(num_joints, num_cols, figsize=(18 if PLOT_ERROR_COLUMN else 9, 3.5 * num_joints), sharex=True)
    plt.subplots_adjust(hspace=0.35, wspace=0.15)
    
    # Ensure axes is a 2D array for consistent indexing
    if num_joints == 1:
        axes = np.array([axes])
    if num_cols == 1:
        axes = axes.reshape(-1, 1)
    
    # Helper: normalize a signal to 0-1 range
    def norm(s): return (s - s.min()) / (s.max() - s.min()) if (s.max() - s.min()) > 1e-6 else s*0
    
    row = 0
    for target_joint, joint_info in reference_data.items():
        mot_id = joint_info['motor']
        sen_id = joint_info['sensor']
        
        # User Fix: Motors 12, 13, 14 shifted by -1
        calc_mot_id = mot_id - 1 if mot_id in [12, 13, 14] else mot_id
        
        col_ang_act = f"act_angle_{calc_mot_id}"
        col_sensor = f"deg_{sen_id}"
        
        ax_left = axes[row, 0]
        ax_right = axes[row, 1] if PLOT_ERROR_COLUMN else None
        
        if col_ang_act in unified_df.columns and col_sensor in unified_df.columns:
            t = unified_df["t_rel"]
            
            # Custom Offsets
            if sen_id == 0:
                sensor_offset = 10.0
            else:
                sensor_offset = -100.0
                
            # Extract raw
            calc_vals_raw = unified_df[col_ang_act]
            sens_vals_raw = unified_df[col_sensor].copy()
            
            # User Fix: Invert specific sensors
            if sen_id in [0, 5]:
                sens_vals_raw = -1.0 * sens_vals_raw
                
            sens_span_raw = sens_vals_raw.max() - sens_vals_raw.min()
            sens_vals_shifted = sens_vals_raw + sensor_offset
            
            # -----------------------------------------------------
            # User Fix: Use Historical Ranges for calibration contrasting
            # -----------------------------------------------------
            if col_sensor in global_ranges:
                hist_min_raw = global_ranges[col_sensor]["min"]
                hist_max_raw = global_ranges[col_sensor]["max"]
                
                if sen_id in [0, 5]:  # Inversions remain consistent
                    hist_min = -1.0 * hist_max_raw
                    hist_max = -1.0 * hist_min_raw
                else:
                    hist_min = hist_min_raw
                    hist_max = hist_max_raw
                    
                hist_span = hist_max - hist_min
            else:
                hist_span = sens_span_raw
                hist_min = sens_vals_raw.min()
                
            # Calculate Sensor-Matched Motor Curve using HISTORICAL bounds
            calc_matched = (norm(calc_vals_raw) * hist_span) + hist_min + sensor_offset
            
            # Calculate Error (Motor expectation vs Actual Sensor behavior)
            error = calc_matched - sens_vals_shifted
            
            rmse = np.sqrt(np.mean(error**2))
            max_err = error.abs().max()
            
            # Normalize error to sensor operational span (in %)
            nrmse = (rmse / sens_span_raw) * 100.0 if sens_span_raw > 1e-6 else 0.0
            nmax = (max_err / sens_span_raw) * 100.0 if sens_span_raw > 1e-6 else 0.0
            
            # --- LEFT COLUMN: Aligned/Scaled Overlay ---
            ax_left.plot(t, calc_matched, label=f"Motor {calc_mot_id}", color="green", lw=2)
            ax_left.plot(t, sens_vals_shifted, label=f"Sensor {sen_id}", color="darkorange", ls="--", lw=1.5)
            
            ax_left.set_title(f"{target_joint}: Signal Tracking Overlay", fontsize=12, fontweight='bold')
            ax_left.set_ylabel("Degrees")
            ax_left.legend(loc="upper right", fontsize=9)
            ax_left.grid(True, alpha=0.3)
            
            # --- RIGHT COLUMN: Error Delta ---
            if PLOT_ERROR_COLUMN:
                ax_right.plot(t, error, label="Error (Motor - Sensor)", color="crimson", lw=1.5)
                ax_right.axhline(0, color='black', lw=1.5, ls='--')
                
                # Annotate Normalized RMSE and Max Error on the plot
                metrics_txt = f"NRMSE: {nrmse:.1f}%\nMax Norm Err: {nmax:.1f}%"
                ax_right.text(0.02, 0.95, metrics_txt, transform=ax_right.transAxes, 
                              verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
                              
                ax_right.set_title(f"{target_joint}: Tracking Error", fontsize=12, fontweight='bold')
                ax_right.set_ylabel("Degrees")
                ax_right.legend(loc="upper right", fontsize=9)
                ax_right.grid(True, alpha=0.3)
            
        else:
            ax_left.set_title(f"{target_joint}")
            ax_left.text(0.5, 0.5, f"Missing {col_ang_act} or {col_sensor}", ha='center', va='center', color='red')
            if PLOT_ERROR_COLUMN:
                ax_right.text(0.5, 0.5, "No Data", ha='center', va='center')
            
        row += 1
        
    axes[-1, 0].set_xlabel("Time (s)", fontsize=11)
    if PLOT_ERROR_COLUMN:
        axes[-1, 1].set_xlabel("Time (s)", fontsize=11)
        
    # Align labels vertically so they don't stagger based on tick widths
    fig.align_ylabels(axes[:, 0])
    if PLOT_ERROR_COLUMN:
        fig.align_ylabels(axes[:, 1])
        
    if SHOW_LOG_TITLE:
        main_title = f"Calculated Motor Position vs Sensor Reading\nLog: {latest_unified}"
    else:
        main_title = "Calculated Motor Position vs Sensor Reading"
    
    plt.suptitle(main_title, fontsize=16, fontweight='bold', y=0.92)
    plt.show()

# %%
