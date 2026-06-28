from typing import Any

import numpy as np


def generate_imu(
    true_motion: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Generate noisy IMU measurements from ground-truth motion.

    IMU outputs:
    - ax
    - ay
    - gyro_z / yaw_rate
    """
    imu_cfg = config["imu"]

    ax_true = true_motion["ax"]
    ay_true = true_motion["ay"]
    psi_dot_true = true_motion["psi_dot"]

    n = len(true_motion["t"])

    accel_noise_std = float(imu_cfg["accel_noise_std"])
    gyro_noise_std = float(imu_cfg["gyro_noise_std"])

    accel_bias_x = float(imu_cfg["accel_bias"]["ax"])
    accel_bias_y = float(imu_cfg["accel_bias"]["ay"])
    gyro_bias = float(imu_cfg["gyro_bias"])

    accel_scale_x = float(imu_cfg["accel_scale"]["ax"])
    accel_scale_y = float(imu_cfg["accel_scale"]["ay"])
    gyro_scale = float(imu_cfg["gyro_scale"])

    imu_ax = (
        accel_scale_x * ax_true
        + accel_bias_x
        + np.random.normal(0.0, accel_noise_std, n)
    )

    imu_ay = (
        accel_scale_y * ay_true
        + accel_bias_y
        + np.random.normal(0.0, accel_noise_std, n)
    )

    imu_gyro_z = (
        gyro_scale * psi_dot_true
        + gyro_bias
        + np.random.normal(0.0, gyro_noise_std, n)
    )

    return {
        "t": true_motion["t"],
        "ax": imu_ax,
        "ay": imu_ay,
        "gyro_z": imu_gyro_z,
        "ax_true": ax_true,
        "ay_true": ay_true,
        "gyro_z_true": psi_dot_true,
    }


def generate_gnss(
    true_motion: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Generate noisy GNSS measurements from ground-truth motion.

    GNSS outputs:
    - x
    - y
    - vx
    - vy

    GNSS frequency is lower than main simulation frequency.
    """
    sim_cfg = config["simulation"]
    gnss_cfg = config["gnss"]

    main_frequency = float(sim_cfg["main_frequency"])
    gnss_frequency = float(gnss_cfg["frequency"])

    step = int(main_frequency / gnss_frequency)

    if step <= 0:
        raise ValueError("GNSS sampling step must be greater than zero.")

    indices = np.arange(0, len(true_motion["t"]), step)

    position_noise_std = float(gnss_cfg["position_noise_std"])
    velocity_noise_std = float(gnss_cfg["velocity_noise_std"])

    gnss_t = true_motion["t"][indices]

    gnss_x = true_motion["x"][indices] + np.random.normal(
        0.0,
        position_noise_std,
        len(indices),
    )

    gnss_y = true_motion["y"][indices] + np.random.normal(
        0.0,
        position_noise_std,
        len(indices),
    )

    gnss_vx = true_motion["vx"][indices] + np.random.normal(
        0.0,
        velocity_noise_std,
        len(indices),
    )

    gnss_vy = true_motion["vy"][indices] + np.random.normal(
        0.0,
        velocity_noise_std,
        len(indices),
    )

    valid = np.ones(len(indices), dtype=bool)

    return {
        "t": gnss_t,
        "x": gnss_x,
        "y": gnss_y,
        "vx": gnss_vx,
        "vy": gnss_vy,
        "valid": valid,
        "indices": indices,
    }