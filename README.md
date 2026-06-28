# IMU-GNSS 2D State Estimation

This project implements a modular and config-driven simulation framework for 2D vehicle state estimation using IMU and GNSS sensors.

The system generates a ground-truth vehicle trajectory, simulates noisy IMU and GNSS measurements, applies sensor calibration, injects GNSS delay and fault scenarios, and compares different filtering approaches.

## Project Scope

- 2D vehicle motion simulation
- IMU sensor modeling
- GNSS sensor modeling
- Sensor bias, scale factor, and noise simulation
- GNSS delay and fault injection
- Sensor calibration
- Raw GNSS, low-pass filter, complementary filter, and Kalman/EKF comparison
- Error and trajectory visualization

## Sensors

- IMU: accelerometer and gyroscope
- GNSS: position and velocity measurements

## Configuration

All simulation parameters, route segments, sensor noise values, fault scenarios, and filter parameters are defined in `config.yaml`.

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

## Run

pip install -r requirements.txt
python main.py