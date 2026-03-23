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
REFERENCE_FILE = "20260321_125054_test_sensor_fix_sensor_log.csv"
TARGET_COLUMN = "deg_1"

def main():
    ref_path = os.path.join(LOG_DIR, REFERENCE_FILE)
    if not os.path.exists(ref_path):
        print(f"Error: Reference file not found: {ref_path}")
        return
        
    ref_mtime = os.path.getmtime(ref_path)
    print(f"Reference file: {REFERENCE_FILE}")
    print(f"Modified at: {ref_mtime}")
    
    # 1. Find all CSV logs in the directory
    all_csvs = glob(os.path.join(LOG_DIR, "*.csv"))
    
    # 2. Filter logs: modified on or after the reference file
    target_logs = []
    for log_path in all_csvs:
        if os.path.getmtime(log_path) >= ref_mtime:
            # We also check if it's the reference file itself or later
            target_logs.append(log_path)
            
    # Sort files by modification time (oldest to newest)
    target_logs.sort(key=os.path.getmtime)
    
    print(f"Found {len(target_logs)} logs modified after or including reference.")
    
    # 3. Setup the plot
    num_logs = len(target_logs)
    if num_logs == 0:
        print("No matching logs found.")
        return
        
    fig, axes = plt.subplots(num_logs, 1, figsize=(14, 3 * num_logs), sharex=False, sharey=True)
    if num_logs == 1:
        axes = [axes]
        
    for i, log_path in enumerate(target_logs):
        log_name = os.path.basename(log_path)
        ax = axes[i]
        
        try:
            df = pd.read_csv(log_path)
            
            # Find time column
            time_col = None
            for t_opt in ["t_rel", "t", "time"]:
                if t_opt in df.columns:
                    time_col = t_opt
                    break
                    
            if TARGET_COLUMN not in df.columns or time_col is None:
                ax.text(0.5, 0.5, f"Missing {TARGET_COLUMN} or time column in {log_name}", 
                        ha='center', va='center', transform=ax.transAxes, color='red')
                ax.set_title(f"Log {i+1}: {log_name}", fontsize=11, fontweight='bold')
                continue
                
            # --- User Fix: Ignore first/last 10 seconds ---
            t_min = df[time_col].min()
            t_max = df[time_col].max()
            df = df[(df[time_col] >= t_min + 10.0) & (df[time_col] <= t_max - 10.0)].copy()
            
            if df.empty:
                ax.text(0.5, 0.5, "Empty data after 10s trimming", ha='center', va='center', transform=ax.transAxes)
                ax.set_title(f"Log {i+1}: {log_name}", fontsize=11, fontweight='bold')
                continue

            t = df[time_col]
            vals = df[TARGET_COLUMN]
            
            ax.plot(t, vals, label=TARGET_COLUMN, color="teal", lw=1.5)
            
            v_min, v_max = vals.min(), vals.max()
            v_span = v_max - v_min
            ax.set_title(f"Log {i+1}: {log_name} | Span: {v_span:.2f}° (Min: {v_min:.2f}°, Max: {v_max:.2f}°)", 
                         fontsize=11, fontweight='bold')
            ax.set_ylabel("Degrees")
            ax.grid(True, alpha=0.4)
            ax.legend(loc="upper right", fontsize=9)
            
        except Exception as e:
            ax.text(0.5, 0.5, f"Error reading {log_name}: {str(e)}", ha='center', va='center', transform=ax.transAxes, color='red')
            
    # Align labels vertically so they don't stagger
    fig.align_ylabels(axes)
    
    axes[-1].set_xlabel("Time (s)", fontsize=12)
    plt.suptitle(f"History of '{TARGET_COLUMN}' for Logs since {REFERENCE_FILE}", fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.subplots_adjust(top=1.0 - (0.05 / (num_logs/10) if num_logs > 5 else 0.05)) # Adjust top margin based on plot density
    plt.show()

if __name__ == "__main__":
    main()
