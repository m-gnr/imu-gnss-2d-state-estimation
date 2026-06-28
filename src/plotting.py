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
    plot_velocity_components(true_motion, gnss, estimates, plot_dir, dpi)
    plot_yaw_comparison(true_motion, estimates, plot_dir, dpi)
    plot_imu_acceleration(true_motion, imu, calibrated_imu, plot_dir, dpi)
    plot_gyro(true_motion, imu, calibrated_imu, plot_dir, dpi)
    plot_gnss_fault_regions(gnss, config, plot_dir, dpi)

    # KF/EKF covariance and 3-sigma consistency plots (PDF Table 9).
    if "EKF" in estimates and "cov" in estimates["EKF"]:
        plot_ekf_covariance(true_motion, estimates["EKF"], plot_dir, dpi)
        plot_ekf_error_3sigma(true_motion, estimates["EKF"], plot_dir, dpi)


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

    # symlog: linear within +/-10 m, logarithmic beyond, so the small
    # nominal errors stay readable while the large fault spikes still show.
    plt.yscale("symlog", linthresh=10)
    plt.xlabel("Time [s]")
    plt.ylabel("x Error [m] (symlog)")
    plt.title("Position Error in x Axis")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_dir / "x_position_error.png", dpi=dpi)
    plt.close()

    plt.figure(figsize=(10, 5))
    for name, estimate in estimates.items():
        error = estimate["y"] - true_motion["y"]
        plt.plot(t, error, label=f"{name} y error")

    plt.yscale("symlog", linthresh=10)
    plt.xlabel("Time [s]")
    plt.ylabel("y Error [m] (symlog)")
    plt.title("Position Error in y Axis")
    plt.grid(True, which="both")
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
        # Small floor so the log scale stays well-defined.
        plt.plot(t, np.maximum(error, 1e-2), label=f"{name} position error")

    # Log scale spans the fault spikes (~700 m) and the nominal errors
    # (a few metres) on one readable plot.
    plt.yscale("log")
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m] (log)")
    plt.title("2D Position Error Magnitude")
    plt.grid(True, which="both")
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

    _plot_calibration_axis(
        t, true_motion["ax"], imu["ax"], calibrated_imu["ax"],
        "ax", "Acceleration [m/s²]",
        plot_dir / "imu_ax_calibration.png", dpi,
    )
    _plot_calibration_axis(
        t, true_motion["ay"], imu["ay"], calibrated_imu["ay"],
        "ay", "Acceleration [m/s²]",
        plot_dir / "imu_ay_calibration.png", dpi,
    )


def _moving_average(data: np.ndarray, window: int) -> np.ndarray:
    """Centred moving average to reveal the slowly-varying (bias) component."""
    if window < 2:
        return data
    kernel = np.ones(window) / window
    return np.convolve(data, kernel, mode="same")


def _plot_calibration_axis(
    t: np.ndarray,
    true_signal: np.ndarray,
    raw_signal: np.ndarray,
    calibrated_signal: np.ndarray,
    label: str,
    ylabel: str,
    out_path: Path,
    dpi: int,
) -> None:
    """
    Plot a calibration comparison. The raw noisy samples are drawn faintly in
    the background, while moving-average overlays make the systematic bias
    (and its removal) clearly visible against the noise.
    """
    win = 200  # 4 s at 50 Hz

    fig, axes = plt.subplots(1, 2, figsize=(13, 5), gridspec_kw={"width_ratios": [2, 1]})

    ax0 = axes[0]
    ax0.plot(t, raw_signal, color="tab:orange", alpha=0.18, linewidth=0.6)
    ax0.plot(t, calibrated_signal, color="tab:green", alpha=0.18, linewidth=0.6)
    ax0.plot(t, true_signal, color="tab:blue", linewidth=1.8, label=f"True {label}")
    ax0.plot(t, _moving_average(raw_signal, win), color="tab:orange",
             linewidth=1.8, label=f"Raw {label} (moving avg)")
    ax0.plot(t, _moving_average(calibrated_signal, win), color="tab:green",
             linewidth=1.8, label=f"Calibrated {label} (moving avg)")
    ax0.set_xlabel("Time [s]")
    ax0.set_ylabel(ylabel)
    ax0.set_title(f"IMU {label} Calibration")
    ax0.grid(True)
    ax0.legend(fontsize=8)

    # Zoom on a constant-velocity window where true input ~ 0, so the bias
    # offset between raw and calibrated is directly visible.
    z0, z1 = 60.0, 90.0
    mask = (t >= z0) & (t <= z1)
    ax1 = axes[1]
    ax1.plot(t[mask], raw_signal[mask], color="tab:orange", alpha=0.3, linewidth=0.7)
    ax1.plot(t[mask], calibrated_signal[mask], color="tab:green", alpha=0.3, linewidth=0.7)
    ax1.axhline(0.0, color="tab:blue", linewidth=1.5, label="True ≈ 0")
    ax1.plot(t[mask], _moving_average(raw_signal, win)[mask], color="tab:orange",
             linewidth=2.0, label="Raw mean (= bias)")
    ax1.plot(t[mask], _moving_average(calibrated_signal, win)[mask], color="tab:green",
             linewidth=2.0, label="Calibrated mean ≈ 0")
    ax1.set_xlabel("Time [s]")
    ax1.set_title(f"Zoom {int(z0)}–{int(z1)} s (constant speed)")
    ax1.grid(True)
    ax1.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def plot_gyro(
    true_motion: dict[str, np.ndarray],
    imu: dict[str, np.ndarray],
    calibrated_imu: dict[str, np.ndarray],
    plot_dir: Path,
    dpi: int,
) -> None:
    t = true_motion["t"]

    _plot_calibration_axis(
        t, true_motion["psi_dot"], imu["gyro_z"], calibrated_imu["gyro_z"],
        "gyro_z", "Yaw Rate [rad/s]",
        plot_dir / "gyro_calibration.png", dpi,
    )


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


def plot_velocity_components(
    true_motion: dict[str, np.ndarray],
    gnss: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
    plot_dir: Path,
    dpi: int,
) -> None:
    """Velocity components vx, vy: ground truth, GNSS and all filters."""
    t = true_motion["t"]
    valid = gnss["valid"]

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for ax, comp in zip(axes, ("vx", "vy")):
        ax.plot(t, true_motion[comp], label="Ground Truth", linewidth=2, color="k")
        ax.scatter(gnss["t"][valid], gnss[comp][valid], s=4, alpha=0.3,
                   label="GNSS", color="tab:blue")
        for name, est in estimates.items():
            ax.plot(t, est[comp], linewidth=1, label=name)
        ax.set_ylabel(f"{comp} [m/s]")
        ax.grid(True)
    axes[0].set_title("Velocity Components: Ground Truth, GNSS and Filters")
    axes[0].legend(fontsize=8, ncol=2)
    axes[1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(plot_dir / "velocity_components.png", dpi=dpi)
    plt.close(fig)


def plot_ekf_covariance(
    true_motion: dict[str, np.ndarray],
    ekf: dict[str, np.ndarray],
    plot_dir: Path,
    dpi: int,
) -> None:
    """EKF state uncertainty (1-sigma) over time from the covariance diagonal."""
    t = true_motion["t"]
    cov = ekf["cov"]                      # variances, shape (5, n)
    sigma = np.sqrt(np.maximum(cov, 0.0))

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(t, sigma[0], label="σx")
    axes[0].plot(t, sigma[1], label="σy")
    axes[0].set_ylabel("Position σ [m]")
    axes[1].plot(t, sigma[2], label="σvx")
    axes[1].plot(t, sigma[3], label="σvy")
    axes[1].set_ylabel("Velocity σ [m/s]")
    axes[2].plot(t, np.rad2deg(sigma[4]), label="σψ", color="tab:green")
    axes[2].set_ylabel("Yaw σ [deg]")
    axes[2].set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True)
        ax.legend(fontsize=9)
    axes[0].set_title("EKF State Covariance (1σ) over Time")
    fig.tight_layout()
    fig.savefig(plot_dir / "ekf_covariance.png", dpi=dpi)
    plt.close(fig)


def plot_ekf_error_3sigma(
    true_motion: dict[str, np.ndarray],
    ekf: dict[str, np.ndarray],
    plot_dir: Path,
    dpi: int,
) -> None:
    """EKF estimation error with +/-3 sigma consistency bounds."""
    t = true_motion["t"]
    sigma = np.sqrt(np.maximum(ekf["cov"], 0.0))

    err_x = ekf["x"] - true_motion["x"]
    err_y = ekf["y"] - true_motion["y"]
    dpsi = ekf["psi"] - true_motion["psi"]
    err_psi = np.rad2deg(np.arctan2(np.sin(dpsi), np.cos(dpsi)))

    panels = [
        ("x error [m]", err_x, 3.0 * sigma[0]),
        ("y error [m]", err_y, 3.0 * sigma[1]),
        ("yaw error [deg]", err_psi, 3.0 * np.rad2deg(sigma[4])),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, (label, err, three_sigma) in zip(axes, panels):
        ax.fill_between(t, -three_sigma, three_sigma, color="tab:orange",
                        alpha=0.25, label="±3σ")
        ax.plot(t, err, color="tab:blue", linewidth=0.8, label="error")
        ax.axhline(0.0, color="k", linewidth=0.6)
        ax.set_ylabel(label)
        ax.grid(True)
        ax.legend(fontsize=9, loc="upper right")
    axes[0].set_title("EKF Estimation Error with ±3σ Bounds")
    axes[2].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(plot_dir / "ekf_error_3sigma.png", dpi=dpi)
    plt.close(fig)