from typing import Any

import numpy as np


def calibrate_imu(
    imu: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Apply simple deterministic IMU calibration.

    Bias and scale factor errors are corrected using the parameters
    defined in config.yaml.

    Random noise is not removed completely by calibration.
    """
    imu_cfg = config["imu"]

    accel_bias_x = float(imu_cfg["accel_bias"]["ax"])
    accel_bias_y = float(imu_cfg["accel_bias"]["ay"])
    gyro_bias = float(imu_cfg["gyro_bias"])

    accel_scale_x = float(imu_cfg["accel_scale"]["ax"])
    accel_scale_y = float(imu_cfg["accel_scale"]["ay"])
    gyro_scale = float(imu_cfg["gyro_scale"])

    calibrated_ax = (imu["ax"] - accel_bias_x) / accel_scale_x
    calibrated_ay = (imu["ay"] - accel_bias_y) / accel_scale_y
    calibrated_gyro_z = (imu["gyro_z"] - gyro_bias) / gyro_scale

    return {
        "t": imu["t"],
        "ax": calibrated_ax,
        "ay": calibrated_ay,
        "gyro_z": calibrated_gyro_z,
    }


def compute_calibration_error_stats(
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
) -> dict[str, float]:
    """
    Compute simple before/after RMSE values for IMU calibration.
    """
    raw_ax_rmse = _rmse(imu["ax_true"], imu["ax"])
    raw_ay_rmse = _rmse(imu["ay_true"], imu["ay"])
    raw_gyro_rmse = _rmse(imu["gyro_z_true"], imu["gyro_z"])

    cal_ax_rmse = _rmse(imu["ax_true"], calibrated_imu["ax"])
    cal_ay_rmse = _rmse(imu["ay_true"], calibrated_imu["ay"])
    cal_gyro_rmse = _rmse(imu["gyro_z_true"], calibrated_imu["gyro_z"])

    return {
        "raw_ax_rmse": raw_ax_rmse,
        "calibrated_ax_rmse": cal_ax_rmse,
        "raw_ay_rmse": raw_ay_rmse,
        "calibrated_ay_rmse": cal_ay_rmse,
        "raw_gyro_rmse": raw_gyro_rmse,
        "calibrated_gyro_rmse": cal_gyro_rmse,
    }


def _rmse(reference: np.ndarray, estimate: np.ndarray) -> float:
    return float(np.sqrt(np.mean((reference - estimate) ** 2)))