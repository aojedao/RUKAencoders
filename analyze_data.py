# %% [markdown]
# # Data Analysis Notebook
# This script is designed to analyze RUKA encoder data from CSV, JSON, TXT, and NPY files.
# It uses the `# %%` format to be compatible with VS Code's Jupyter Interactive window and Jupyter Notebooks.

# %%
import pandas as pd
import numpy as np
import json
import os
import matplotlib.pyplot as plt
import seaborn as sns
from glob import glob

# Define display if it doesn't exist (e.g. running outside of Jupyter)
try:
    from IPython.display import display
except (ImportError, NameError):
    display = print

# Set plot style
sns.set_theme(style="whitegrid")
plt.rcParams['figure.figsize'] = [12, 6]

# %%
# Configuration: Define paths to data
try:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if run in a pure interactive environment where __file__ is undefined
    BASE_DIR = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders"

LOG_DIR = os.path.join(BASE_DIR, "encoder_logs")
CALIB_DIR = os.path.join(BASE_DIR, "calibration")
CALIB_VIS_DIR = os.path.join(BASE_DIR, "calibration_visualizations")
MD_REF_FILE = os.path.join(BASE_DIR, "Right-Hand Joint Span Reference.md")

# %% [markdown]
# ## Data Loading Utility Functions

# %%
def load_csv_log(filename):
    """Load a CSV log file into a pandas DataFrame."""
    path = os.path.join(LOG_DIR, filename)
    if os.path.exists(path):
        df = pd.read_csv(path)
        print(f"Loaded {filename} with {len(df)} rows.")
        return df
    print(f"File not found: {path}")
    return None

def load_npy_log(filename):
    """Load a .npy or .npz log file into a numpy array."""
    path = os.path.join(LOG_DIR, filename)
    if os.path.exists(path):
        data = np.load(path)
        print(f"Loaded {filename} with shape {data.shape if isinstance(data, np.ndarray) else data.files}.")
        return data
    print(f"File not found: {path}")
    return None

def load_calibration_map(filename="right_structured_calibration_map.json"):
    """Load the structured calibration JSON map."""
    path = os.path.join(CALIB_DIR, filename)
    if os.path.exists(path):
        with open(path, 'r') as f:
            data = json.load(f)
        print(f"Loaded calibration map from {filename}.")
        return data
    print(f"File not found: {path}")
    return None

def get_t0(filename):
    """Read the start timestamp from a .txt file."""
    path = os.path.join(LOG_DIR, filename)
    if os.path.exists(path):
        with open(path, 'r') as f:
            t0 = float(f.read().strip())
        print(f"Start timestamp (t0): {t0}")
        return t0
    print(f"File not found: {path}")
    return None

# %% [markdown]
# ## Exploratory Data Analysis (EDA)
# Let's try loading a sample motor log and visualizing it.

# %%

# %% [markdown]
# ## Calibration Data
# Visualize the range of motion from the calibration map.

# %%
calib_data = load_calibration_map()
if calib_data:
    # Example: Print joint ranges
    for joint, data in calib_data.items():
        if isinstance(data, dict) and 'range' in data:
            print(f"Joint {joint}: Range {data['range']}")

# %% [markdown]
# ## Calibration Visualizations
# Load and display the sensor calibration range tables.

# %%
# Load and display calibration visualization tables
complete_ranges_file = os.path.join(CALIB_VIS_DIR, "right_structured_calibration_map_sensor_complete_ranges.csv")
grouped_ranges_file = os.path.join(CALIB_VIS_DIR, "right_structured_calibration_map_sensor_ranges_grouped_by_joint.csv")

if os.path.exists(complete_ranges_file):
    complete_ranges_df = pd.read_csv(complete_ranges_file)
    print("--- Sensor Complete Ranges ---")
    display(complete_ranges_df)
else:
    print(f"File not found: {complete_ranges_file}")

if os.path.exists(grouped_ranges_file):
    grouped_ranges_df = pd.read_csv(grouped_ranges_file)
    print("\n--- Sensor Ranges Grouped by Joint ---")
    display(grouped_ranges_df)
else:
    print(f"File not found: {grouped_ranges_file}")

# %% [markdown]
# ## Joint Data Comparison
# Comparing theoretical ranges against logged and calculated data.
# Theoretical data is loaded from `Right-Hand Joint Span Reference.md`.

# %%
# Import the current (most likely correct) live mappings instead of the Markdown file
import re
from live_calibration import RIGHT_HAND_JOINT_GROUPS

reference_data = {}
for g in RIGHT_HAND_JOINT_GROUPS:
    motor_val = g["motor"]
    
    # -----------------------------------------------------
    # User Fix: Swap motors 8 and 9 explicitly in this file
    # -----------------------------------------------------
    if motor_val == 8:
        motor_val = 9
    elif motor_val == 9:
        motor_val = 8
        
    reference_data[g["joint"]] = {
        "sensor": g["sensor"],
        "motor": motor_val,
        "expected_span": g["expected_abs_span_deg"]
    }

print("Loaded Theoretical Ranges (from live_calibration.py, with 8/9 swapped):")
for joint, data in reference_data.items():
    print(f"  {joint}: Motor {data['motor']}, Sensor {data['sensor']} -> Span: {data['expected_span']}°")

# %%
# Analyze all joints
unified_logs = glob(os.path.join(LOG_DIR, "*unified_log.csv"))
if unified_logs:
    # Sort files by modification time to get the genuinely newest log
    latest_unified_path = max(unified_logs, key=os.path.getmtime)
    latest_unified = os.path.basename(latest_unified_path)
    print(f"\nAnalyzing latest unified log: {latest_unified}")
    unified_df = load_csv_log(latest_unified)
    
    if unified_df is not None:
        # -----------------------------------------------------
        # User Fix: Ignore the first and last 10 seconds of data
        # -----------------------------------------------------
        if 't_rel' in unified_df.columns:
            t_min = unified_df['t_rel'].min()
            t_max = unified_df['t_rel'].max()
            unified_df = unified_df[(unified_df['t_rel'] >= t_min + 10.0) & (unified_df['t_rel'] <= t_max - 10.0)].copy()
            
        for target_joint in reference_data.keys():
            joint_info = reference_data[target_joint]
            mot_id = joint_info['motor']
            sen_id = joint_info['sensor']
            theo_span = joint_info['expected_span']
            
            # -----------------------------------------------------
            # User Fix: Motors 12, 13, 14 are shifted by -1 in log
            # -----------------------------------------------------
            calc_mot_id = mot_id - 1 if mot_id in [12, 13, 14] else mot_id
            
            col_ang_act = f"act_angle_{calc_mot_id}"
            col_sensor = f"deg_{sen_id}"
            
            if col_ang_act in unified_df.columns and col_sensor in unified_df.columns:
                t = unified_df["t_rel"]
                
                # -----------------------------------------------------
                # User Fix: Custom offsets for Sensor 0
                # -----------------------------------------------------
                if sen_id == 0:
                    motor_offset = 50.0
                    sensor_offset = 10.0
                else:
                    motor_offset = 190.0
                    sensor_offset = -100.0
                
                # Helper: normalize a signal to 0-1 range
                def norm(s): return (s - s.min()) / (s.max() - s.min()) if (s.max() - s.min()) > 1e-6 else s*0
                
                # --- 1. Calculated data (Current Theoretical Logic) ---
                calc_vals_raw = unified_df[col_ang_act]
                calc_span_raw = calc_vals_raw.max() - calc_vals_raw.min()
                calc_vals_shifted = calc_vals_raw + motor_offset
                
                # --- 2. Sensor data (Actual Readings) ---
                sens_vals_raw = unified_df[col_sensor].copy()
                
                # -----------------------------------------------------
                # User Fix: Invert sensors 0 and 5
                # -----------------------------------------------------
                if sen_id in [0, 5]:
                    sens_vals_raw = -1.0 * sens_vals_raw
                    
                sens_span_raw = sens_vals_raw.max() - sens_vals_raw.min()
                sens_vals_shifted = sens_vals_raw + sensor_offset
                
                # --- 3. "Sensor-Matched" Motor Calculation (Magnitude scaling only) ---
                # We map the motor motion into the same range as the sensor readout.
                calc_matched = (norm(calc_vals_raw) * sens_span_raw) + sens_vals_raw.min() + sensor_offset
                
                # Scale Factor for report
                emp_scale_factor = sens_span_raw / calc_span_raw if calc_span_raw > 1e-6 else 1.0
                
                # --- 4. Report ---
                print("\n" + "="*60)
                print(f"COMPARISON REPORT: {target_joint} (Calibration vs Real-World)")
                print("="*60)
                print(f"Sources:")
                print(f"  Theoretical: live_calibration.py (RIGHT_HAND_JOINT_GROUPS)")
                print(f"  Log Data:    {latest_unified}")
                print(f"  Offsets:     Motor {motor_offset:+}, Sensor {sensor_offset:+}")
                print(f"\nRanges:")
                print(f"  1. Theoretical Design Span: {theo_span:.2f}°")
                print(f"  2. In-Log Calculated Span:  {calc_span_raw:.2f}° (Current math, column {col_ang_act})")
                print(f"  3. In-Log Sensor Span:      {sens_span_raw:.2f}° (Actual recorded)")
                
                print(f"\nEmpirical Tuning:")
                print(f"  4. Scaling Correction:      {emp_scale_factor:.4f}x")
                if emp_scale_factor > 0:
                    print(f"     (Motor math overestimates movement magnitude by {(1.0/emp_scale_factor):.2f}x)")
                
                # Previous Calib values
                if 'grouped_ranges_df' in locals():
                    calib_row = grouped_ranges_df[grouped_ranges_df["joint"] == target_joint]
                    if not calib_row.empty:
                        calib_span = calib_row["calculated_span_deg"].values[0]
                        print(f"  5. Global Calib. Span:      {calib_span:.2f}° (Fixed map)")
                print("="*60)
        
                # --- 5. Plotting (Subplots) ---
                fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
                plt.subplots_adjust(hspace=0.25)
                
                # Top Axis: RAW (Current Calibration logic)
                ax1.plot(t, calc_vals_shifted, label=f"Motor {calc_mot_id} Computed Angle (Theo)", color="steelblue", lw=2)
                ax1.plot(t, sens_vals_shifted, label=f"Sensor {sen_id} Reading (Raw)", color="darkorange", ls="--", lw=2.5)
                ax1.set_title(f"{target_joint}: Theoretical Mapping Comparison\n(Magnitude is currently mismatched)", fontsize=13)
                ax1.set_ylabel("Angle (degrees)", fontsize=11)
                ax1.legend(loc="upper right", fontsize=10)
                ax1.grid(True, alpha=0.3)
                
                # Bottom Axis: ALIGNED (Scaled to match sensor)
                ax2.plot(t, calc_matched, label=f"Motor {calc_mot_id} Rescaled to Sensor", color="green", lw=2)
                ax2.plot(t, sens_vals_shifted, label=f"Sensor {sen_id} Reading (Raw)", color="darkorange", ls="--", lw=1.5, alpha=0.9)
                ax2.set_title(f"{target_joint}: Signal Shape Correlation\n(Motor calculation scaled down by {emp_scale_factor:.2f}x)", fontsize=13)
                ax2.set_ylabel("Angle (degrees)", fontsize=11)
                ax2.set_xlabel("Time (s)", fontsize=11)
                ax2.legend(loc="upper right", fontsize=10)
                ax2.grid(True, alpha=0.3)
                
                plt.show()
            else:
                print(f"\n[{target_joint}] Columns not found. Expected: {col_ang_act}, {col_sensor}")
else:
    print("\nNo unified logs found.")
