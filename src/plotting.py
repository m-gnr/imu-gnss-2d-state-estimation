from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


def create_all_plots(
    true_motion: dict[str, np.ndarray],
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
    gnss: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    config: dict[str, Any],
) -> None:
    """
    Create and save all required project plots.
    """
    plot_dir = Path(config["output"]["plot_dir"])
    plot_dir.mkdir(parents=True, exist_ok=True)

    dpi = int(config["plotting"]["dpi"])

    plot_trajectory(true_motion, gnss, estimates, plot_dir, dpi)
    plot_position_errors(true_motion, estimates, plot_dir, dpi)
    plot_velocity_comparison(true_motion, estimates, plot_dir, dpi)
    plot_yaw_comparison(true_motion, estimates, plot_dir, dpi)
    plot_imu_acceleration(true_motion, imu, calibrated_imu, plot_dir, dpi)
    plot_gyro(true_motion, imu, calibrated_imu, plot_dir, dpi)
    plot_gnss_fault_regions(gnss, config, plot_dir, dpi)


def plot_trajectory(
    true_motion: dict[str, np.ndarray],
    gnss: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    plot_dir: Path,
    dpi: int,
) -> None:
    plt.figure(figsize=(9, 7))

    plt.plot(true_motion["x"], true_motion["y"], label="Ground Truth", linewidth=2)

    valid = gnss["valid"]
    plt.scatter(
        gnss["x"][valid],
        gnss["y"][valid],
        s=4,
        label="GNSS Valid",
        alpha=0.5,
    )

    invalid = ~valid
    if np.any(invalid):
        plt.scatter(
            gnss["x"][invalid],
            gnss["y"][invalid],
            s=10,
            label="GNSS Invalid",
            alpha=0.7,
        )

    for name, estimate in estimates.items():
        plt.plot(estimate["x"], estimate["y"], label=name, linewidth=1)

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("2D Trajectory Comparison")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "trajectory_comparison.png", dpi=dpi)
    plt.close()


def plot_position_errors(
    true_motion: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    plt.figure(figsize=(10, 5))
    for name, estimate in estimates.items():
        error = estimate["x"] - true_motion["x"]
        plt.plot(t, error, label=f"{name} x error")

    plt.xlabel("Time [s]")
    plt.ylabel("x Error [m]")
    plt.title("Position Error in x Axis")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "x_position_error.png", dpi=dpi)
    plt.close()

    plt.figure(figsize=(10, 5))
    for name, estimate in estimates.items():
        error = estimate["y"] - true_motion["y"]
        plt.plot(t, error, label=f"{name} y error")

    plt.xlabel("Time [s]")
    plt.ylabel("y Error [m]")
    plt.title("Position Error in y Axis")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "y_position_error.png", dpi=dpi)
    plt.close()

    plt.figure(figsize=(10, 5))
    for name, estimate in estimates.items():
        error = np.sqrt(
            (estimate["x"] - true_motion["x"]) ** 2
            + (estimate["y"] - true_motion["y"]) ** 2
        )
        plt.plot(t, error, label=f"{name} position error")

    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("2D Position Error Magnitude")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "position_error_magnitude.png", dpi=dpi)
    plt.close()


def plot_velocity_comparison(
    true_motion: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    true_speed = np.sqrt(true_motion["vx"] ** 2 + true_motion["vy"] ** 2)

    plt.figure(figsize=(10, 5))
    plt.plot(t, true_speed, label="Ground Truth", linewidth=2)

    for name, estimate in estimates.items():
        speed = np.sqrt(estimate["vx"] ** 2 + estimate["vy"] ** 2)
        plt.plot(t, speed, label=name, linewidth=1)

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed Comparison")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "speed_comparison.png", dpi=dpi)
    plt.close()


def plot_yaw_comparison(
    true_motion: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    plt.figure(figsize=(10, 5))
    plt.plot(t, np.rad2deg(true_motion["psi"]), label="Ground Truth", linewidth=2)

    for name, estimate in estimates.items():
        plt.plot(t, np.rad2deg(estimate["psi"]), label=name, linewidth=1)

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [deg]")
    plt.title("Yaw / Heading Comparison")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "yaw_comparison.png", dpi=dpi)
    plt.close()


def plot_imu_acceleration(
    true_motion: dict[str, np.ndarray],
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    plt.figure(figsize=(10, 5))
    plt.plot(t, true_motion["ax"], label="True ax", linewidth=2)
    plt.plot(t, imu["ax"], label="Raw IMU ax", alpha=0.7)
    plt.plot(t, calibrated_imu["ax"], label="Calibrated IMU ax", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.title("IMU ax Calibration")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "imu_ax_calibration.png", dpi=dpi)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(t, true_motion["ay"], label="True ay", linewidth=2)
    plt.plot(t, imu["ay"], label="Raw IMU ay", alpha=0.7)
    plt.plot(t, calibrated_imu["ay"], label="Calibrated IMU ay", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s²]")
    plt.title("IMU ay Calibration")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "imu_ay_calibration.png", dpi=dpi)
    plt.close()


def plot_gyro(
    true_motion: dict[str, np.ndarray],
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    plt.figure(figsize=(10, 5))
    plt.plot(t, true_motion["psi_dot"], label="True yaw rate", linewidth=2)
    plt.plot(t, imu["gyro_z"], label="Raw IMU gyro z", alpha=0.7)
    plt.plot(t, calibrated_imu["gyro_z"], label="Calibrated IMU gyro z", alpha=0.7)

    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Rate [rad/s]")
    plt.title("Gyroscope Calibration")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "gyro_calibration.png", dpi=dpi)
    plt.close()


def plot_gnss_fault_regions(
    gnss: dict[str, np.ndarray],
    config: dict[str, Any],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = gnss["t"]

    plt.figure(figsize=(10, 5))
    plt.plot(t, gnss["x"], label="GNSS x")
    plt.plot(t, gnss["y"], label="GNSS y")

    faults = config["faults"]

    if faults["dropout"]["enabled"]:
        plt.axvspan(
            faults["dropout"]["start"],
            faults["dropout"]["end"],
            alpha=0.2,
            label="Dropout",
        )

    if faults["position_jump"]["enabled"]:
        plt.axvspan(
            faults["position_jump"]["start"],
            faults["position_jump"]["end"],
            alpha=0.2,
            label="Position Jump",
        )

    if faults["freeze"]["enabled"]:
        plt.axvspan(
            faults["freeze"]["start"],
            faults["freeze"]["end"],
            alpha=0.2,
            label="Freeze",
        )

    plt.xlabel("Time [s]")
    plt.ylabel("GNSS Position [m]")
    plt.title("GNSS Fault Scenarios")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "gnss_fault_scenarios.png", dpi=dpi)
    plt.close()