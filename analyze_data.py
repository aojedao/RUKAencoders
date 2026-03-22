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
LOG_DIR = "encoder_logs"
CALIB_DIR = "calibration"
CALIB_VIS_DIR = "calibration_visualizations"

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
