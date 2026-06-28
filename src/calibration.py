from typing import Any

import numpy as np


def calibrate_imu(
    imu: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Calibrate the IMU using biases estimated *from the data*.

    Bias estimation
    ---------------
    The biases are NOT taken from the config (that would assume we already
    know the true error). Instead they are estimated from a calibration
    window in which the vehicle moves at constant speed on a straight line,
    so the true acceleration and yaw-rate are approximately zero. In that
    window the IMU output equals (bias + noise), so averaging it yields the
    bias estimate.

    Scale factors are assumed known from the sensor datasheet / factory
    calibration and are read from the config.

    Correction model: calibrated = (raw - bias_estimate) / scale.

    Random noise is not removed by calibration.
    """
    imu_cfg = config["imu"]
    cal_cfg = config["calibration"]

    accel_scale_x = float(imu_cfg["accel_scale"]["ax"])
    accel_scale_y = float(imu_cfg["accel_scale"]["ay"])
    gyro_scale = float(imu_cfg["gyro_scale"])

    bias = estimate_biases(imu, cal_cfg)

    calibrated_ax = (imu["ax"] - bias["ax"]) / accel_scale_x
    calibrated_ay = (imu["ay"] - bias["ay"]) / accel_scale_y
    calibrated_gyro_z = (imu["gyro_z"] - bias["gyro_z"]) / gyro_scale

    return {
        "t": imu["t"],
        "ax": calibrated_ax,
        "ay": calibrated_ay,
        "gyro_z": calibrated_gyro_z,
        "estimated_bias": bias,
    }


def estimate_biases(
    imu: dict[str, np.ndarray],
    cal_cfg: dict[str, Any],
) -> dict[str, float]:
    """
    Estimate accelerometer and gyroscope biases by averaging the raw IMU
    output over a constant-velocity, straight-line calibration window where
    the true input is ~zero.
    """
    t = imu["t"]
    start = float(cal_cfg["bias_window_start"])
    end = float(cal_cfg["bias_window_end"])

    mask = (t >= start) & (t <= end)

    if not np.any(mask):
        raise ValueError(
            f"Calibration window [{start}, {end}] contains no IMU samples."
        )

    return {
        "ax": float(np.mean(imu["ax"][mask])),
        "ay": float(np.mean(imu["ay"][mask])),
        "gyro_z": float(np.mean(imu["gyro_z"][mask])),
    }


def compute_calibration_error_stats(
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
) -> dict[str, float]:
    """
    Compute before/after RMSE values for the IMU calibration, plus the
    estimated-vs-true bias comparison.
    """
    raw_ax_rmse = _rmse(imu["ax_true"], imu["ax"])
    raw_ay_rmse = _rmse(imu["ay_true"], imu["ay"])
    raw_gyro_rmse = _rmse(imu["gyro_z_true"], imu["gyro_z"])

    cal_ax_rmse = _rmse(imu["ax_true"], calibrated_imu["ax"])
    cal_ay_rmse = _rmse(imu["ay_true"], calibrated_imu["ay"])
    cal_gyro_rmse = _rmse(imu["gyro_z_true"], calibrated_imu["gyro_z"])

    stats = {
        "raw_ax_rmse": raw_ax_rmse,
        "calibrated_ax_rmse": cal_ax_rmse,
        "raw_ay_rmse": raw_ay_rmse,
        "calibrated_ay_rmse": cal_ay_rmse,
        "raw_gyro_rmse": raw_gyro_rmse,
        "calibrated_gyro_rmse": cal_gyro_rmse,
    }

    bias = calibrated_imu.get("estimated_bias")
    if bias is not None:
        stats["estimated_bias_ax"] = bias["ax"]
        stats["estimated_bias_ay"] = bias["ay"]
        stats["estimated_bias_gyro"] = bias["gyro_z"]

    return stats


def _rmse(reference: np.ndarray, estimate: np.ndarray) -> float:
    return float(np.sqrt(np.mean((reference - estimate) ** 2)))
