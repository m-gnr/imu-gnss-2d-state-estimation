from typing import Any

import numpy as np

# Speed (m/s) below which GNSS velocity is dominated by noise and the
# heading atan2(vy, vx) becomes meaningless. Below this we hold the last
# reliable heading instead of trusting the noisy estimate.
_MIN_SPEED_FOR_HEADING = 1.0


def raw_gnss_estimate(
    gnss: dict[str, np.ndarray],
    target_time: np.ndarray,
) -> dict[str, np.ndarray]:
    """
    Convert sparse GNSS measurements to the main simulation time using
    interpolation. Invalid GNSS measurements are ignored.

    Also returns a per-sample ``valid`` mask at the main time rate, which is
    True wherever the main time is covered by valid GNSS samples and False
    inside dropout gaps. Downstream filters use it to stop trusting GNSS
    during dropouts.
    """
    valid = gnss["valid"]

    if np.sum(valid) < 2:
        raise ValueError("Not enough valid GNSS samples for interpolation.")

    t_valid = gnss["t"][valid]

    x = np.interp(target_time, t_valid, gnss["x"][valid])
    y = np.interp(target_time, t_valid, gnss["y"][valid])
    vx = np.interp(target_time, t_valid, gnss["vx"][valid])
    vy = np.interp(target_time, t_valid, gnss["vy"][valid])

    psi = _heading_from_velocity(vx, vy)

    # Validity at the main rate: drops below 1 inside a dropout gap.
    valid_main = (
        np.interp(target_time, gnss["t"], valid.astype(float)) > 0.999
    )

    return {
        "t": target_time,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "psi": psi,
        "valid": valid_main,
    }


def low_pass_filter_estimate(
    raw_estimate: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Apply a first-order low-pass filter to the raw GNSS estimate.
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

    IMU is used for short-term propagation, GNSS for long-term correction.
    During GNSS dropouts the correction is skipped, so the estimate runs on
    IMU dead-reckoning alone and is pulled back once GNSS returns.
    """
    dt = float(config["simulation"]["dt"])
    alpha = float(config["filters"]["complementary"]["alpha"])
    init_yaw = np.deg2rad(float(config["vehicle"]["initial_yaw_deg"]))

    n = len(target_time)
    gnss_valid = raw_gnss.get("valid", np.ones(n, dtype=bool))

    x = np.zeros(n)
    y = np.zeros(n)
    vx = np.zeros(n)
    vy = np.zeros(n)
    psi = np.zeros(n)

    x[0] = raw_gnss["x"][0]
    y[0] = raw_gnss["y"][0]
    vx[0] = raw_gnss["vx"][0]
    vy[0] = raw_gnss["vy"][0]
    psi[0] = init_yaw

    for k in range(1, n):
        # IMU propagation
        vx_pred = vx[k - 1] + calibrated_imu["ax"][k] * dt
        vy_pred = vy[k - 1] + calibrated_imu["ay"][k] * dt
        x_pred = x[k - 1] + vx_pred * dt
        y_pred = y[k - 1] + vy_pred * dt
        psi_pred = psi[k - 1] + calibrated_imu["gyro_z"][k] * dt

        if gnss_valid[k]:
            # GNSS correction
            x[k] = alpha * x_pred + (1.0 - alpha) * raw_gnss["x"][k]
            y[k] = alpha * y_pred + (1.0 - alpha) * raw_gnss["y"][k]
            vx[k] = alpha * vx_pred + (1.0 - alpha) * raw_gnss["vx"][k]
            vy[k] = alpha * vy_pred + (1.0 - alpha) * raw_gnss["vy"][k]
            psi[k] = psi_pred  # GNSS has no direct heading measurement
        else:
            # GNSS dropout: pure IMU dead-reckoning
            x[k] = x_pred
            y[k] = y_pred
            vx[k] = vx_pred
            vy[k] = vy_pred
            psi[k] = psi_pred

    return {
        "t": target_time,
        "x": x,
        "y": y,
        "vx": vx,
        "vy": vy,
        "psi": psi,
    }


def ekf_estimate(
    target_time: np.ndarray,
    calibrated_imu: dict[str, np.ndarray],
    gnss: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Extended Kalman Filter fusing IMU (prediction) and GNSS (update).

    State vector: [x, y, vx, vy, psi].

    Prediction uses the calibrated IMU as control input:
        x   += vx * dt
        y   += vy * dt
        vx  += ax * dt
        vy  += ay * dt
        psi += gyro_z * dt

    Update uses GNSS position/velocity measurements [x, y, vx, vy] at the
    GNSS sample times, and is skipped whenever the GNSS sample is invalid
    (dropout). Heading psi is not directly observed by GNSS, so it is held
    by the gyro integration (and stays accurate because the gyro bias is
    removed during calibration).

    Note: because the IMU accelerations are modelled in the global frame,
    the state-transition Jacobian F is constant, so this EKF reduces to a
    linear Kalman filter here. The EKF structure (explicit F/H, predict/
    update) is kept so a body-frame IMU model could be plugged in directly.
    """
    dt = float(config["simulation"]["dt"])
    ekf_cfg = config["filters"]["ekf"]
    gnss_cfg = config["gnss"]
    init_yaw = np.deg2rad(float(config["vehicle"]["initial_yaw_deg"]))

    n = len(target_time)

    # State-transition Jacobian (constant for the global-frame model)
    F = np.eye(5)
    F[0, 2] = dt
    F[1, 3] = dt

    # Process noise
    q_pos = float(ekf_cfg["process_noise_pos"])
    q_vel = float(ekf_cfg["process_noise_vel"])
    q_yaw = float(ekf_cfg["process_noise_yaw"])
    Q = np.diag([q_pos**2, q_pos**2, q_vel**2, q_vel**2, q_yaw**2])

    # Measurement model: GNSS observes [x, y, vx, vy]
    H = np.zeros((4, 5))
    H[0, 0] = 1.0
    H[1, 1] = 1.0
    H[2, 2] = 1.0
    H[3, 3] = 1.0

    pos_std = float(gnss_cfg["position_noise_std"])
    vel_std = float(gnss_cfg["velocity_noise_std"])
    R = np.diag([pos_std**2, pos_std**2, vel_std**2, vel_std**2])

    gate_threshold = float(ekf_cfg.get("gate_threshold", np.inf))

    # Map main-time index -> GNSS sample position
    gnss_indices = gnss["indices"]
    gnss_valid = gnss["valid"]
    meas_map = {int(idx): i for i, idx in enumerate(gnss_indices)}

    # Initial state from the first valid GNSS sample
    first_valid = int(np.argmax(gnss_valid))
    state = np.array(
        [
            gnss["x"][first_valid],
            gnss["y"][first_valid],
            gnss["vx"][first_valid],
            gnss["vy"][first_valid],
            init_yaw,
        ]
    )

    P = np.diag(
        [
            float(ekf_cfg["init_pos_var"]),
            float(ekf_cfg["init_pos_var"]),
            float(ekf_cfg["init_vel_var"]),
            float(ekf_cfg["init_vel_var"]),
            float(ekf_cfg["init_yaw_var"]),
        ]
    )

    x_out = np.zeros(n)
    y_out = np.zeros(n)
    vx_out = np.zeros(n)
    vy_out = np.zeros(n)
    psi_out = np.zeros(n)

    x_out[0] = state[0]
    y_out[0] = state[1]
    vx_out[0] = state[2]
    vy_out[0] = state[3]
    psi_out[0] = state[4]

    for k in range(1, n):
        ax = calibrated_imu["ax"][k]
        ay = calibrated_imu["ay"][k]
        gyro = calibrated_imu["gyro_z"][k]

        # --- Predict ---
        state = np.array(
            [
                state[0] + state[2] * dt,
                state[1] + state[3] * dt,
                state[2] + ax * dt,
                state[3] + ay * dt,
                state[4] + gyro * dt,
            ]
        )
        P = F @ P @ F.T + Q

        # --- Update (only at valid GNSS sample times) ---
        if k in meas_map and gnss_valid[meas_map[k]]:
            i = meas_map[k]
            z = np.array([gnss["x"][i], gnss["y"][i], gnss["vx"][i], gnss["vy"][i]])
            y_res = z - H @ state
            S = H @ P @ H.T + R
            S_inv = np.linalg.inv(S)

            # Innovation gating: reject outliers (e.g. the position-jump
            # fault) whose normalised squared residual is too large.
            mahalanobis = float(y_res @ S_inv @ y_res)
            if mahalanobis <= gate_threshold:
                K = P @ H.T @ S_inv
                state = state + K @ y_res
                P = (np.eye(5) - K @ H) @ P

        x_out[k] = state[0]
        y_out[k] = state[1]
        vx_out[k] = state[2]
        vy_out[k] = state[3]
        psi_out[k] = state[4]

    return {
        "t": target_time,
        "x": x_out,
        "y": y_out,
        "vx": vx_out,
        "vy": vy_out,
        "psi": psi_out,
    }


def _heading_from_velocity(vx: np.ndarray, vy: np.ndarray) -> np.ndarray:
    """
    Compute heading from velocity, holding the last reliable value while the
    vehicle is (nearly) stopped, where atan2(vy, vx) would be pure noise.
    """
    speed = np.hypot(vx, vy)
    psi = np.arctan2(vy, vx)
    reliable = speed > _MIN_SPEED_FOR_HEADING

    if not np.any(reliable):
        return np.unwrap(psi)

    # Forward-fill heading from the last reliable sample.
    last = psi[np.argmax(reliable)]  # first reliable value
    for k in range(len(psi)):
        if reliable[k]:
            last = psi[k]
        else:
            psi[k] = last

    return np.unwrap(psi)


def _low_pass(data: np.ndarray, alpha: float) -> np.ndarray:
    """
    First-order recursive low-pass filter.
    """
    filtered = np.zeros_like(data)
    filtered[0] = data[0]

    for k in range(1, len(data)):
        filtered[k] = alpha * data[k] + (1.0 - alpha) * filtered[k - 1]

    return filtered
