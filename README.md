# RUKAencoders

This repository contains tools for capturing, analyzing, and recalibrating the magnetic angle sensors (AS5600) on the RUKA hand.

## Overview of the Workflow

The ideal chronological sequence for evaluating and correcting sensor drift is as follows:

### 1. Test Generation & Execution
- **`random_joint_generator.py`**: The primary tool for generating structured test data. It iterates through every joint sequentially, generating a set number of random points within the safe physical bounds defined in the JSON limits.
  - **Command**: `python random_joint_generator.py --points-per-joint 5 --wait-time 8.0`
  - It automatically restores every joint to the "Open Hand" position after its specific test cycle finishes.
  - Logs are saved to the `random_generator_logs/` directory with a timestamp and parameter summary in the filename.

### 2. Normalized Data Analysis (Paper Metrics)
Once data is collected, you can analyze the tracking performance by normalizing both motor commands and sensor readings into a shared `[0, 1]` range. This uses the **`manually_filtered_paper_limits_redo.json`** as the ground truth for scaling.

- **`normalize_log_signals.py`**: Visualizes a single log file in a 7-row stack (one for each sensor).
  - **Command**: `python Paper/normalize_log_signals.py --log path/to/log.csv --out result.png`
- **`normalize_log_signals_grid.py`**: Compiles up to 6 different logs into a single `2x3` master grid for batch comparison.
  - **Command**: `python Paper/normalize_log_signals_grid.py --logs log1.csv log2.csv ... --out grid_comparison.png`
  - **Scaling Logic**: It normalizes motor ticks using the `commanded_min/max` and sensor degrees using the `filtered_sensor_min/max` from the redo JSON. It also automatically inverts sensors 0, 2, 4, and 6 so they track positively with their motors.

### 3. Historical Consistency & Extraction
- **`analyze_signal_history.py`**: Reads all your recent unified logs to visually inspect noise and consistency over time, saving absolute global min/max boundaries into `historical_sensor_ranges.json`.
- **`error_report.py`**: Compares the most recent log's actual sensor trace against theoretical motor positions to mathematically score tracking error (RMSE).

---

## Technical Reference Tables

The following table serves as the ultimate "source of truth", utilizing the limits defined in **`manually_filtered_paper_limits_redo.json`**.

### Comprehensive Joint-Sensor-Motor Mapping & Bounds

| Joint Name | Sensor | Motor | Array Idx | Cmd Min | Cmd Max | Manually Filtered Range (Redo) |
| :--- | :---: | :---: | :---: | ---: | ---: | :--- |
| **Index Abduction** | S0 | M8 | 7 | 1250 | 1641 | 73.4° to 96.94° |
| **Index DIP** | S1 | M7 | 6 | 1600 | 2983 | 170.0° to 250.0° |
| **Index PIP** | S2 | M7 | 6 | 1600 | 2983 | 20.0° to 120.0° |
| **Index MCP** | S3 | M9 | 8 | 592 | 1249 | 100.0° to 250.0° |
| **Thumb CMC** | S4 | M13 | 12 | 2000 | 3042 | 105.0° to 210.0° |
| **Thumb MCP** | S5 | M14 | 13 | 1350 | 2288 | 102.0° to 170.0° |
| **Thumb DIP** | S6 | M12 | 11 | 492 | 1060 | 5.0° to 60.0° |

> **Context Notes**:
> - **Normalization**: The analysis scripts normalize signals using the formula `(val - min) / (max - min)`.
> - **Inversions**: Sensors 0, 2, 4, and 6 are inverted in the plot logic (`1.0 - norm_deg`) to align with motor command directions.
