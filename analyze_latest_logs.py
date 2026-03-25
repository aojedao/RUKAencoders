import os
import pandas as pd
import glob

# Configuration
LOG_DIR = "/home/aojedao/Documents/NYU/GRAIL/RUKAencoders/random_generator_logs"
N_LATEST_LOGS = 25

# Columns of interest based on the user's request "6, 7, etc"
CMD_INDICES = range(6, 16)  # cmd_6 to cmd_15
DEG_INDICES = range(0, 7)   # deg_0 to deg_6

def analyze_logs():
    # 1. Get the latest 25 logs
    log_files = glob.glob(os.path.join(LOG_DIR, "*.csv"))
    if not log_files:
        print(f"No log files found in {LOG_DIR}")
        return
        
    # Sort files by name (which contains the timestamp) to get the latest
    log_files.sort(reverse=True)
    latest_logs = log_files[:N_LATEST_LOGS]
    
    print(f"Analyzing the latest {len(latest_logs)} logs...")
    
    # Data structures to store global min/max for each column
    # format: { col_name: {"min": (val, log, t), "max": (val, log, t)} }
    stats = {}

    for log_path in latest_logs:
        log_name = os.path.basename(log_path)
        try:
            # Low memory usage for large CSVs
            df = pd.read_csv(log_path)
            
            if df.empty:
                continue
                
            cols_to_check = [f"cmd_{i}" for i in CMD_INDICES if f"cmd_{i}" in df.columns]
            cols_to_check += [f"deg_{i}" for i in DEG_INDICES if f"deg_{i}" in df.columns]
            
            for col in cols_to_check:
                if col not in stats:
                    stats[col] = {"min": (float('inf'), "", 0.0), "max": (float('-inf'), "", 0.0)}
                
                # Filter out nans for this column
                temp_df = df[[col, "t_rel"]].dropna()
                if temp_df.empty:
                    continue
                
                # Find min and its timestamp
                min_idx = temp_df[col].idxmin()
                min_row = temp_df.loc[min_idx]
                min_val = min_row[col]
                if min_val < stats[col]["min"][0]:
                    stats[col]["min"] = (min_val, log_name, min_row["t_rel"])
                
                # Find max and its timestamp
                max_idx = temp_df[col].idxmax()
                max_row = temp_df.loc[max_idx]
                max_val = max_row[col]
                if max_val > stats[col]["max"][0]:
                    stats[col]["max"] = (max_val, log_name, max_row["t_rel"])
                    
        except Exception as e:
            print(f"Error reading {log_name}: {e}")

    # 2. Output the results
    print("\n" + "="*85)
    print(f"{'COLUMN':<10} | {'TYPE':<5} | {'VALUE':<10} | {'TIMESTAMP (t_rel)':<18} | {'LOG FILE'}")
    print("-" * 85)
    
    # Sort columns by type (deg then cmd) then by index
    sorted_cols = sorted(stats.keys(), key=lambda x: (x.split("_")[0], int(x.split("_")[1])))
    
    for col in sorted_cols:
        s = stats[col]
        # Skip if no data was found
        if s["min"][0] == float('inf'):
            continue
            
        # Format values
        v_min = s["min"][0]
        v_max = s["max"][0]
        
        if "cmd" in col:
            fmt_min = f"{int(v_min)}"
            fmt_max = f"{int(v_max)}"
        else:
            fmt_min = f"{v_min:.2f}"
            fmt_max = f"{v_max:.2f}"
            
        print(f"{col:<10} | {'MIN':<5} | {fmt_min:<10} | {s['min'][2]:<18.4f} | {s['min'][1]}")
        print(f"{col:<10} | {'MAX':<5} | {fmt_max:<10} | {s['max'][2]:<18.4f} | {s['max'][1]}")
        print("-" * 85)

if __name__ == "__main__":
    analyze_logs()
