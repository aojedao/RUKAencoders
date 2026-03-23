import os
import pandas as pd
import matplotlib.pyplot as plt

# Configuration
try:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    BASE_DIR = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders"
    
LOG_DIR = os.path.join(BASE_DIR, "encoder_logs")

# Specific log files requested by user
TARGET_LOGS = [
    "20260321_125054_test_sensor_fix_sensor_log.csv",
    "20260321_125054_test_sensor_fix_unified.csv",
    "20260321_125243_complete_test_sensor_log.csv",
    "20260321_125243_complete_test_unified.csv"
]

# Joint/Sensor Mapping for readable headers
SENSOR_LABELS = {
    0: "Index Abduction",
    1: "Index DIP",
    2: "Index PIP",
    3: "Index MCP",
    4: "Thumb DIP",
    5: "Thumb MCP",
    6: "Thumb CMC"
}

def main():
    if not os.path.exists(LOG_DIR):
        print(f"Error: LOG_DIR not found: {LOG_DIR}")
        return
        
    # Filter to only the files that actually exist
    existing_paths = []
    for log_name in TARGET_LOGS:
        p = os.path.join(LOG_DIR, log_name)
        if os.path.exists(p):
            existing_paths.append(p)
        else:
            print(f"Warning: Missing file: {p}")
            
    if not existing_paths:
        print("No matching logs found from the specific list.")
        return
        
    num_rows = len(existing_paths)
    num_cols = 7 # One column per sensor (0 to 6)
    
    # Large figure for a 4x7 grid
    fig, axes = plt.subplots(num_rows, num_cols, figsize=(28, 4.5 * num_rows), sharex='col', sharey='col')
    
    for row_idx, file_path in enumerate(existing_paths):
        full_name = os.path.basename(file_path).replace(".csv", "")
        # Break file name into multiple lines for legibility
        row_label = full_name.replace("_", "\n")
        print(f"Processing row {row_idx + 1}: {full_name}")
        
        try:
            df = pd.read_csv(file_path)
            
            # --- User Fix: Ignore first/last 10 seconds of data ---
            time_col = None
            for t_opt in ["t_rel", "t", "time"]:
                if t_opt in df.columns:
                    time_col = t_opt
                    break
            
            if time_col:
                t_min = df[time_col].min()
                t_max = df[time_col].max()
                df = df[(df[time_col] >= t_min + 10.0) & (df[time_col] <= t_max - 10.0)].copy()
            
            if df.empty:
                for col_idx in range(num_cols):
                    axes[row_idx, col_idx].text(0.5, 0.5, "Empty (10s skip)", ha='center', va='center')
                continue
                
            # Plot each sensor in its own column
            for col_idx in range(num_cols):
                ax = axes[row_idx, col_idx]
                col_name = f"deg_{col_idx}"
                
                if col_name in df.columns and time_col:
                    ax.plot(df[time_col], df[col_name], color="teal", lw=1.5)
                    
                    v_min, v_max = df[col_name].min(), df[col_name].max()
                    v_span = v_max - v_min
                    ax.set_title(f"Span: {v_span:.1f}°", fontsize=10, alpha=0.7)
                else:
                    ax.text(0.5, 0.5, f"Missing {col_name}", ha='center', va='center', color='red')
                
                # --- Label Column (only on top row) ---
                if row_idx == 0:
                    s_name = SENSOR_LABELS.get(col_idx, "Unknown")
                    ax.annotate(f"S{col_idx}:\n{s_name}", xy=(0.5, 1.3), xycoords='axes fraction', 
                                ha='center', fontsize=12, fontweight='bold', color='navy', 
                                bbox=dict(boxstyle="round,pad=0.3", fc="aliceblue", ec="navy", lw=2))

                # --- Label Row (only on first column) ---
                if col_idx == 0:
                    ax.set_ylabel("Degrees", fontsize=10)
                    # Use a very generous offset and a smaller font to ensure the filename is readable
                    ax.annotate(row_label, xy=(-0.75, 0.5), xycoords='axes fraction', 
                                rotation=0, va='center', ha='center', fontsize=9, fontweight='bold', color="maroon")

                if row_idx == num_rows - 1:
                    ax.set_xlabel("Time (s)")
                
                ax.grid(True, alpha=0.2)
            
        except Exception as e:
            for col_idx in range(num_cols):
                axes[row_idx, col_idx].text(0.5, 0.5, f"Error: {e}", ha='center', color='red')
            
    # Perfect alignment for Y-labels
    for c in range(num_cols):
        fig.align_ylabels(axes[:, c])
    
    plt.suptitle("Complete Sensor Traces Matrix (4x7 Behavioral breakdown)", 
                 fontsize=22, fontweight='bold', y=0.99)
    # Give a lot of room on the left (0.15) and top (0.93)
    plt.tight_layout(rect=[0.15, 0, 1, 0.93]) 
    plt.show()

if __name__ == "__main__":
    main()
