from copy import deepcopy
from typing import Any

import numpy as np


def apply_gnss_delay(
    gnss: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Apply GNSS measurement delay as a sample shift.
    """
    delayed_gnss = deepcopy(gnss)

    gnss_frequency = float(config["gnss"]["frequency"])
    delay_sec = float(config["gnss"]["delay_sec"])

    delay_steps = int(round(delay_sec * gnss_frequency))

    if delay_steps <= 0:
        return delayed_gnss

    for key in ["x", "y", "vx", "vy"]:
        data = delayed_gnss[key].copy()
        shifted = np.roll(data, delay_steps)
        shifted[:delay_steps] = data[0]
        delayed_gnss[key] = shifted

    delayed_gnss["delay_steps"] = np.array([delay_steps])

    return delayed_gnss


def apply_gnss_faults(
    gnss: dict[str, np.ndarray],
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    """
    Apply configured GNSS fault scenarios.
    """
    faulty_gnss = deepcopy(gnss)

    t = faulty_gnss["t"]
    valid = faulty_gnss["valid"].copy()
    faults_cfg = config["faults"]

    if faults_cfg["dropout"]["enabled"]:
        _apply_dropout(valid, faults_cfg["dropout"], t)

    if faults_cfg["position_jump"]["enabled"]:
        _apply_position_jump(faulty_gnss, faults_cfg["position_jump"], t)

    if faults_cfg["freeze"]["enabled"]:
        _apply_freeze(faulty_gnss, faults_cfg["freeze"], t)

    faulty_gnss["valid"] = valid

    return faulty_gnss


def _apply_dropout(
    valid: np.ndarray,
    dropout_cfg: dict[str, Any],
    t: np.ndarray,
) -> None:
    start = float(dropout_cfg["start"])
    end = float(dropout_cfg["end"])

    mask = (t >= start) & (t <= end)
    valid[mask] = False


def _apply_position_jump(
    gnss: dict[str, np.ndarray],
    jump_cfg: dict[str, Any],
    t: np.ndarray,
) -> None:
    start = float(jump_cfg["start"])
    end = float(jump_cfg["end"])
    error_x = float(jump_cfg["error_x"])
    error_y = float(jump_cfg["error_y"])

    mask = (t >= start) & (t <= end)

    gnss["x"][mask] += error_x
    gnss["y"][mask] += error_y


def _apply_freeze(
    gnss: dict[str, np.ndarray],
    freeze_cfg: dict[str, Any],
    t: np.ndarray,
) -> None:
    start = float(freeze_cfg["start"])
    end = float(freeze_cfg["end"])

    mask = (t >= start) & (t <= end)
    freeze_indices = np.where(mask)[0]

    if len(freeze_indices) == 0:
        return

    first_idx = freeze_indices[0]

    for key in ["x", "y", "vx", "vy"]:
        gnss[key][mask] = gnss[key][first_idx]