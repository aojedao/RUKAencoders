# RUKAencoders

This repository contains tools for capturing, analyzing, and recalibrating the magnetic angle sensors (AS5600) on the RUKA hand.

## Overview of the Workflow

The ideal chronological sequence for evaluating and correcting sensor drift is as follows:

### 1. Data Collection
- **`replay_encoders.py`**: Runs a fixed trajectory on the hand (e.g., wallet pinch) and saves raw logs of both motor and sensor data. Run this 5 to 10 times to capture varying states and build a reliable statistical sample of the physical hand's limits. *(Note: Ensure you export as a unified log using your unification method before proceeding to Phase 2).*

### 2. Extracting Global Limits & History Analysis
- **`analyze_signal_history.py`**: Reads all your recent unified logs. It plots them side-by-side to let you visually inspect noise and consistency over time, AND it automatically extracts the absolute global min/max boundaries across all runs, saving them into `historical_sensor_ranges.json`.
- **`analyze_deg0_history.py`**: A specific version of the signal history script tailored for analyzing the `deg_0` sensor.

### 3. Error Analysis & Visualization
- **`error_report.py`**: Reads `historical_sensor_ranges.json` and compares the most recent log's actual sensor trace against the theoretical motor positions (mapping ticks to expected degrees). This mathematically scores the tracking error (RMSE).
- **`analyze_complete_logs.py`**: Generates a grid matrix of all sensors side-by-side with their linked motor commands, giving a comprehensive visual overview of the hand's performance for a single run.

### 4. Hardware Recalibration
- **`recalibrate_sensors_only.py`**: If Phase 3 reveals unacceptable error or drift, use this script. It is a **live interactive script** that connects directly to the hand and ESP32. It guides you to physically open/close the hand to capture new boundary snapshots, updating the core `calibration/right_structured_calibration_map.json` file. It also logs the raw trace of the calibration sequence to `calibration_trace_log.csv` so you can review the noise profile of your manual recalibration.

---

## Technical Reference Tables

The following table serves as the ultimate "source of truth", merging true mapping knowledge (matching the data analysis scripts), the practical array commands, calculated motor angles accurately indexed, and the resulting physical sensor outputs.

### Comprehensive Joint-Sensor-Motor Mapping & Bounds

| Joint Name | Sensor | Motor | Array Idx | Cmd Min | Cmd Max | Calculated Motor Angle Limit | Manually Filtered Range | Live Calibration Limits |
| :--- | :---: | :---: | :---: | ---: | ---: | :--- | :--- | :--- |
| **Index Abduction** | S0 | M8 | 7 | 1250 | 1641 | -27.67° to -17.84° (`act_angle_7`) | 85.0° to 100.9° | 76.07° to 88.00° |
| **Index DIP** | S1 | M7 | 6 | 1600 | 2983 | -43.28° to 17.00° (`act_angle_6`) | 55.0° to 236.0° | 207.77° to 243.21° |
| **Index PIP** | S2 | M7 | 6 | 1600 | 2983 | -43.28° to 17.00° (`act_angle_6`) | 15.0° to 355.0° | 9.42° to 125.11° |
| **Index MCP** | S3 | M9 | 8 | 592 | 1249 | 50.16° to 84.91° (`act_angle_8`) | 100.0° to 300.0° | 34.43° to 66.18° |
| **Thumb DIP** | S4 | M13 | 12 | 2000 | 3042 | -190.77° to -105.82° (`act_angle_12`) | 105.0° to 210.0° | 92.42° to 119.62° |
| **Thumb MCP** | S5 | M14 | 13 | 1350 | 2288 | -34.03° to 34.55° (`act_angle_13`) | 102.0° to 170.0° | 171.21° to 173.37° |
| **Thumb CMC** | S6 | M12 | 11 | 492 | 1060 | 6.00° to 279.75° (`act_angle_11`) | 40.0° to 80.0° | 22.24° to 46.29° |

> **Context Notes**:
> - **Array Index & Cmd Ranges**: Motor dynamically actuated mapped to a 0-15 Array format. The table shows real-world tick limits extracted via test suites from tracking bounds (incorporating updated manual boundaries).
> - **Calculated Motor Angle Limit**: Mathematical angle dynamically derived from motor ticks, utilizing the `act_angle_X` range data saved inside history arrays. *(Note: M8 and M9 math representations internally swap map locations during calculations).*
> - **Manually Filtered Range**: Shows the real physical limits completely scrubbed of any raw communication peaks mapped dynamically via the custom `manually_filtered_paper_limits.json` profile!
