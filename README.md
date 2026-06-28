# IMU-GNSS 2D State Estimation

A modular, config-driven Python simulation for IMU-GNSS based 2D vehicle state estimation. The project generates a ground-truth vehicle trajectory, simulates noisy sensors and GNSS faults, compares multiple estimation filters, and exports plots and RMSE metrics.

## Features

- Ground-truth 2D vehicle trajectory generation
- IMU sensor model with accelerometer `ax`, `ay` and gyroscope yaw rate
- GNSS sensor model with `x`, `y`, `vx`, `vy`
- Sensor noise, bias, and scale factor modeling
- IMU calibration
- GNSS delay of 200 ms
- GNSS fault scenarios:
  - Dropout between 300-310 s
  - 500 m position jump between 400-401 s
  - Frozen GNSS data between 500-505 s
- Filter comparison:
  - Raw GNSS
  - Low-pass Filter
  - Complementary Filter
- RMSE metric calculation
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

- `trajectory_comparison.png`
- `position_error_magnitude.png`
- `speed_comparison.png`
- `yaw_comparison.png`
- `gnss_fault_scenarios.png`
- `imu_ax_calibration.png`
- `imu_ay_calibration.png`
- `gyro_calibration.png`

RMSE metrics are saved as CSV output under `outputs/data`.
