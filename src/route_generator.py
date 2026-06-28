from typing import Any

import numpy as np


def kmh_to_ms(speed_kmh: float) -> float:
    """Convert speed from km/h to m/s."""
    return speed_kmh * 1000.0 / 3600.0


def generate_true_motion(config: dict[str, Any]) -> dict[str, np.ndarray]:
    """
    Generate ground-truth vehicle motion from route configuration.

    Returns:
        Dictionary containing t, x, y, v, vx, vy, ax, ay, psi, psi_dot.
    """
    sim_cfg = config["simulation"]
    vehicle_cfg = config["vehicle"]
    route_cfg = config["route"]

    total_time = float(sim_cfg["total_time"])
    dt = float(sim_cfg["dt"])

    t = np.arange(0.0, total_time + dt, dt)
    n = len(t)

    x = np.zeros(n)
    y = np.zeros(n)
    v = np.zeros(n)
    psi = np.zeros(n)

    x[0] = float(vehicle_cfg["initial_x"])
    y[0] = float(vehicle_cfg["initial_y"])
    v[0] = kmh_to_ms(float(vehicle_cfg["initial_speed_kmh"]))
    psi[0] = np.deg2rad(float(vehicle_cfg["initial_yaw_deg"]))

    turn_sign = float(vehicle_cfg.get("turn_direction_sign", -1.0))

    for segment in route_cfg:
        start = float(segment["start"])
        end = float(segment["end"])
        duration = end - start

        speed_start = kmh_to_ms(float(segment["speed_start_kmh"]))
        speed_end = kmh_to_ms(float(segment["speed_end_kmh"]))

        yaw_change = np.deg2rad(float(segment["yaw_change_deg"])) * turn_sign

        mask = (t >= start) & (t <= end)
        tau = t[mask] - start

        if duration <= 0:
            raise ValueError(f"Invalid route segment duration: {segment}")

        # Linear speed transition
        v[mask] = speed_start + (speed_end - speed_start) * (tau / duration)

        # Yaw is integrated segment by segment later.
        # Here we create yaw-rate profile for this segment.
        yaw_rate = yaw_change / duration

        if "psi_dot_profile" not in locals():
            psi_dot_profile = np.zeros(n)

        psi_dot_profile[mask] = yaw_rate

    # Integrate yaw
    for k in range(n - 1):
        psi[k + 1] = psi[k] + psi_dot_profile[k] * dt

    vx = v * np.cos(psi)
    vy = v * np.sin(psi)

    # Integrate position
    for k in range(n - 1):
        x[k + 1] = x[k] + vx[k] * dt
        y[k + 1] = y[k] + vy[k] * dt

    ax = np.gradient(vx, dt)
    ay = np.gradient(vy, dt)
    psi_dot = np.gradient(psi, dt)

    return {
        "t": t,
        "x": x,
        "y": y,
        "v": v,
        "vx": vx,
        "vy": vy,
        "ax": ax,
        "ay": ay,
        "psi": psi,
        "psi_dot": psi_dot,
    }


def print_motion_checkpoints(true_motion: dict[str, np.ndarray]) -> None:
    """
    Print route checkpoint values for quick validation.
    """
    t = true_motion["t"]
    v = true_motion["v"]
    psi = true_motion["psi"]

    checkpoints = [10, 20, 260, 830, 1000]

    print("\nMotion checkpoints:")
    for cp in checkpoints:
        idx = int(np.argmin(np.abs(t - cp)))
        speed = v[idx]
        yaw_deg = np.rad2deg(psi[idx])
        print(f"t={cp:>4}s | speed={speed:>8.3f} m/s | yaw={yaw_deg:>8.3f} deg")