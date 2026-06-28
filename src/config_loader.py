from pathlib import Path
from typing import Any

import yaml


def load_config(config_path: str | Path = "config.yaml") -> dict[str, Any]:
    """
    Load YAML configuration file.

    Args:
        config_path: Path to the YAML config file.

    Returns:
        Configuration dictionary.

    Raises:
        FileNotFoundError: If config file does not exist.
        ValueError: If YAML file is empty or invalid.
    """
    config_path = Path(config_path)

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    if config is None:
        raise ValueError(f"Config file is empty: {config_path}")

    validate_config(config)

    return config


def validate_config(config: dict[str, Any]) -> None:
    """
    Validate required config sections and basic parameter consistency.
    """
    required_sections = [
        "simulation",
        "vehicle",
        "route",
        "imu",
        "gnss",
        "faults",
        "filters",
        "output",
        "plotting",
    ]

    for section in required_sections:
        if section not in config:
            raise ValueError(f"Missing required config section: {section}")

    simulation = config["simulation"]

    total_time = simulation.get("total_time")
    main_frequency = simulation.get("main_frequency")
    dt = simulation.get("dt")

    if total_time is None or total_time <= 0:
        raise ValueError("simulation.total_time must be greater than 0")

    if main_frequency is None or main_frequency <= 0:
        raise ValueError("simulation.main_frequency must be greater than 0")

    if dt is None or dt <= 0:
        raise ValueError("simulation.dt must be greater than 0")

    expected_dt = 1.0 / main_frequency
    if abs(dt - expected_dt) > 1e-9:
        raise ValueError(
            f"simulation.dt and simulation.main_frequency are inconsistent. "
            f"Expected dt={expected_dt}, got dt={dt}"
        )

    route = config["route"]

    if not isinstance(route, list) or len(route) == 0:
        raise ValueError("route must be a non-empty list")

    previous_end = 0.0

    for index, segment in enumerate(route):
        required_segment_keys = [
            "start",
            "end",
            "speed_start_kmh",
            "speed_end_kmh",
            "yaw_change_deg",
        ]

        for key in required_segment_keys:
            if key not in segment:
                raise ValueError(f"Missing route[{index}].{key}")

        start = segment["start"]
        end = segment["end"]

        if end <= start:
            raise ValueError(f"route[{index}] end must be greater than start")

        if index == 0 and start != 0:
            raise ValueError("First route segment must start at 0")

        if abs(start - previous_end) > 1e-9:
            raise ValueError(
                f"Route segments must be continuous. "
                f"route[{index}] starts at {start}, previous ended at {previous_end}"
            )

        previous_end = end

    if abs(previous_end - total_time) > 1e-9:
        raise ValueError(
            f"Last route segment must end at simulation.total_time. "
            f"Last end={previous_end}, total_time={total_time}"
        )

    imu_freq = config["imu"].get("frequency")
    gnss_freq = config["gnss"].get("frequency")

    if imu_freq is None or imu_freq <= 0:
        raise ValueError("imu.frequency must be greater than 0")

    if gnss_freq is None or gnss_freq <= 0:
        raise ValueError("gnss.frequency must be greater than 0")

    if imu_freq != main_frequency:
        raise ValueError(
            "This implementation expects imu.frequency to be equal to "
            "simulation.main_frequency"
        )

    if main_frequency % gnss_freq != 0:
        raise ValueError(
            "simulation.main_frequency must be divisible by gnss.frequency"
        )