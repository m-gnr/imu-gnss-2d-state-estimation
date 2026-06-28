from typing import Any

import numpy as np


def raw_gnss_estimate(
    gnss: dict[str, np.ndarray],
    target_time: np.ndarray,
) -> dict[str, np.ndarray]:
    """
    Convert sparse GNSS measurements to main simulation time using interpolation.

    Invalid GNSS measurements are ignored.
    """
    valid = gnss["valid"]

    if np.sum(valid) < 2:
        raise ValueError("Not enough valid GNSS samples for interpolation.")

    t_valid = gnss["t"][valid]

    x = np.interp(target_time, t_valid, gnss["x"][valid])
    y = np.interp(target_time, t_valid, gnss["y"][valid])
    vx = np.interp(target_time, t_valid, gnss["vx"][valid])
    vy = np.interp(target_time, t_valid, gnss["vy"][valid])

    psi = np.unwrap(np.arctan2(vy, vx))

    return {
        "t": target_time,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "psi": psi,
    }


def low_pass_filter_estimate(
    raw_estimate: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Apply first-order low-pass filter to raw GNSS estimate.
    """
    alpha = float(config["filters"]["low_pass"]["alpha"])

    return {
        "t": raw_estimate["t"],
        "x": _low_pass(raw_estimate["x"], alpha),
        "y": _low_pass(raw_estimate["y"], alpha),
        "vx": _low_pass(raw_estimate["vx"], alpha),
        "vy": _low_pass(raw_estimate["vy"], alpha),
        "psi": _low_pass(raw_estimate["psi"], alpha),
    }


def complementary_filter(
    target_time: np.ndarray,
    calibrated_imu: dict[str, np.ndarray],
    raw_gnss: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Simple complementary filter.

    IMU is used for short-term propagation.
    GNSS is used for long-term correction.
    """
    dt = float(config["simulation"]["dt"])
    alpha = float(config["filters"]["complementary"]["alpha"])

    n = len(target_time)

    x = np.zeros(n)
    y = np.zeros(n)
    vx = np.zeros(n)
    vy = np.zeros(n)
    psi = np.zeros(n)

    x[0] = raw_gnss["x"][0]
    y[0] = raw_gnss["y"][0]
    vx[0] = raw_gnss["vx"][0]
    vy[0] = raw_gnss["vy"][0]
    psi[0] = raw_gnss["psi"][0]

    for k in range(1, n):
        # IMU propagation
        vx_pred = vx[k - 1] + calibrated_imu["ax"][k] * dt
        vy_pred = vy[k - 1] + calibrated_imu["ay"][k] * dt

        x_pred = x[k - 1] + vx_pred * dt
        y_pred = y[k - 1] + vy_pred * dt

        psi_pred = psi[k - 1] + calibrated_imu["gyro_z"][k] * dt

        # GNSS correction
        x[k] = alpha * x_pred + (1.0 - alpha) * raw_gnss["x"][k]
        y[k] = alpha * y_pred + (1.0 - alpha) * raw_gnss["y"][k]
        vx[k] = alpha * vx_pred + (1.0 - alpha) * raw_gnss["vx"][k]
        vy[k] = alpha * vy_pred + (1.0 - alpha) * raw_gnss["vy"][k]
        psi[k] = alpha * psi_pred + (1.0 - alpha) * raw_gnss["psi"][k]

    return {
        "t": target_time,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "psi": psi,
    }


def _low_pass(data: np.ndarray, alpha: float) -> np.ndarray:
    """
    First-order recursive low-pass filter.
    """
    filtered = np.zeros_like(data)
    filtered[0] = data[0]

    for k in range(1, len(data)):
        filtered[k] = alpha * data[k] + (1.0 - alpha) * filtered[k - 1]

    return filtered