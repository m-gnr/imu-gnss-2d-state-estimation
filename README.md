# IMU-GNSS 2D State Estimation

A modular, config-driven Python simulation for IMU-GNSS based 2D vehicle state estimation. The project generates a ground-truth vehicle trajectory, simulates noisy sensors and GNSS faults, compares multiple estimation filters, and exports plots and RMSE metrics.

## Features

- Ground-truth 2D vehicle trajectory generation
- IMU sensor model with accelerometer `ax`, `ay` and gyroscope yaw rate
- GNSS sensor model with `x`, `y`, `vx`, `vy`
- Sensor noise, bias, and scale factor modeling
- IMU calibration with biases estimated **from the data** (constant-velocity window), not taken from ground truth
- GNSS delay of 200 ms
- GNSS fault scenarios:
  - Dropout between 300-310 s
  - 500 m position jump between 400-401 s
  - Frozen GNSS data between 500-505 s
- Filter comparison:
  - Raw GNSS
  - Low-pass Filter
  - Complementary Filter (IMU dead-reckoning during GNSS dropout)
  - Extended Kalman Filter (EKF) with innovation gating for outlier/fault rejection
- RMSE metric calculation for position, velocity, and yaw
- Plot generation under `outputs/plots`
- RMSE CSV output under `outputs/data`

## Project Structure

```text
imu-gnss-2d-state-estimation/
├── main.py
├── config.yaml
├── requirements.txt
├── src/
│   ├── config_loader.py
│   ├── route_generator.py
│   ├── sensor_models.py
│   ├── faults.py
│   ├── calibration.py
│   ├── filters.py
│   ├── metrics.py
│   └── plotting.py
├── outputs/
│   ├── plots/
│   └── data/
└── report/
    └── IMU_GNSS_2D_State_Estimation_Raporu.pdf
```

## Installation

```bash
python -m pip install -r requirements.txt
```

## Usage

```bash
python main.py
```

The simulation reads parameters from `config.yaml`, runs the trajectory, sensor, fault, calibration, filtering, and evaluation pipeline, then writes generated artifacts to the `outputs` directory.

## Generated Outputs

Plots are saved under `outputs/plots`:

- `true_trajectory.png`
- `trajectory_comparison.png`
- `x_position_error.png`
- `y_position_error.png`
- `position_error_magnitude.png`
- `speed_comparison.png`
- `velocity_components.png`
- `yaw_comparison.png`
- `gnss_fault_scenarios.png`
- `imu_ax_calibration.png`
- `imu_ay_calibration.png`
- `gyro_calibration.png`
- `ekf_covariance.png`
- `ekf_error_3sigma.png`

RMSE metrics are saved as CSV output under `outputs/data/rmse_table.csv`.

A full Turkish project report (9 sections, with embedded plots and the RMSE table) is generated under `report/`.

## GUI (minimap)

A config-driven, GTA-style heading-up minimap that replays the run. It is fully
decoupled from the pipeline: it only reads `outputs/data/timeseries.npz`, which
`main.py` exports. Run the pipeline once, then launch the GUI:

```bash
python main.py        # produces outputs/data/timeseries.npz
python run_gui.py     # opens the minimap window
```

Features: vehicle-centred heading-up (rotating) minimap with the ground-truth
route, the selected filter trail and GNSS points (invalid ones in red); a HUD
(speed, time, method, position error); on-screen fault warnings; a media-player
timeline you can scrub back and forth; and live error strips (position, speed
and yaw error vs ground truth) for the active method, synced to the playhead. Everything —
window size, colours, zoom, trail length, default method, playback speeds,
which strips to show — is read from the `gui:` section of `config.yaml`.

Controls: `Space` play/pause · `← →` ±5 s · `↑ ↓` speed · `M` cycle method ·
`O` heading-up/north-up · `G` toggle GNSS · `T`/`E` toggle ground-truth/estimate
· drag the timeline to scrub.

## Results

RMSE of each method against the ground truth (lower is better):

| Method | Position RMSE [m] | Velocity RMSE [m/s] | Yaw RMSE [°] |
|---|---|---|---|
| Raw GNSS | 25.18 | 0.697 | 1.289 |
| Low-pass Filter | 26.01 | 0.683 | 1.039 |
| Complementary Filter | 17.48 | 0.411 | 1.688 |
| **EKF** | **5.01** | **0.325** | **1.688** |

The EKF performs best across position and velocity; its innovation gating rejects the
500 m position jump and the GNSS freeze faults, keeping the error low where the other
methods spike. Yaw RMSE is identical for the EKF and complementary filter because
neither receives a direct heading measurement from GNSS (both integrate the calibrated
gyro).
