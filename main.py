from pathlib import Path

import numpy as np

from src.config_loader import load_config
from src.route_generator import generate_true_motion, print_motion_checkpoints
from src.sensor_models import generate_imu, generate_gnss
from src.faults import apply_gnss_delay, apply_gnss_faults
from src.calibration import calibrate_imu, compute_calibration_error_stats
from src.filters import (
    raw_gnss_estimate,
    low_pass_filter_estimate,
    complementary_filter,
    ekf_estimate,
)
from src.metrics import compute_rmse_table
from src.plotting import create_all_plots


def ensure_output_dirs(config: dict) -> None:
    Path(config["output"]["plot_dir"]).mkdir(parents=True, exist_ok=True)
    Path(config["output"]["data_dir"]).mkdir(parents=True, exist_ok=True)


def export_timeseries(
    config: dict,
    true_motion: dict,
    gnss: dict,
    estimates: dict,
) -> Path:
    """
    Export the full time series the GUI needs into a single .npz file.

    The GUI reads this file only; it never imports the simulation pipeline.
    """
    out_path = Path(
        config["output"].get("timeseries_file", "outputs/data/timeseries.npz")
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)

    methods = list(estimates.keys())
    est_x = np.vstack([estimates[m]["x"] for m in methods])
    est_y = np.vstack([estimates[m]["y"] for m in methods])
    est_psi = np.vstack([estimates[m]["psi"] for m in methods])
    est_vx = np.vstack([estimates[m]["vx"] for m in methods])
    est_vy = np.vstack([estimates[m]["vy"] for m in methods])

    np.savez_compressed(
        out_path,
        t=true_motion["t"],
        true_x=true_motion["x"],
        true_y=true_motion["y"],
        true_psi=true_motion["psi"],
        true_v=true_motion["v"],
        methods=np.array(methods),
        est_x=est_x,
        est_y=est_y,
        est_psi=est_psi,
        est_vx=est_vx,
        est_vy=est_vy,
        gnss_t=gnss["t"],
        gnss_x=gnss["x"],
        gnss_y=gnss["y"],
        gnss_valid=gnss["valid"],
    )
    return out_path


def main() -> None:
    config = load_config("config.yaml")
    ensure_output_dirs(config)

    if "random_seed" in config["simulation"]:
        np.random.seed(config["simulation"]["random_seed"])

    print("[1/7] Generating ground-truth motion...")
    true_motion = generate_true_motion(config)
    print_motion_checkpoints(true_motion)

    print("[2/7] Generating sensor measurements...")
    imu = generate_imu(true_motion, config)
    gnss = generate_gnss(true_motion, config)

    print("[3/7] Applying GNSS delay and faults...")
    gnss = apply_gnss_delay(gnss, config)
    gnss = apply_gnss_faults(gnss, config)

    print("[4/7] Calibrating IMU...")
    calibrated_imu = calibrate_imu(imu, config)
    calibration_stats = compute_calibration_error_stats(imu, calibrated_imu)

    print("[5/7] Running filters...")
    raw_estimate = raw_gnss_estimate(gnss, true_motion["t"])
    lpf_estimate = low_pass_filter_estimate(raw_estimate, config)
    comp_estimate = complementary_filter(
        true_motion["t"],
        calibrated_imu,
        raw_estimate,
        config,
    )
    ekf_est = ekf_estimate(
        true_motion["t"],
        calibrated_imu,
        gnss,
        config,
    )

    estimates = {
        "Raw GNSS": raw_estimate,
        "Low-pass Filter": lpf_estimate,
        "Complementary Filter": comp_estimate,
        "EKF": ekf_est,
    }

    print("[6/7] Computing metrics...")
    rmse_table = compute_rmse_table(true_motion, estimates)

    print("\nCalibration RMSE:")
    for key, value in calibration_stats.items():
        print(f"{key}: {value:.6f}")

    print("\nFilter RMSE table:")
    print(rmse_table.to_string(index=False))

    rmse_csv_path = Path(config["output"]["data_dir"]) / "rmse_table.csv"
    rmse_table.to_csv(rmse_csv_path, index=False)

    print("[7/8] Creating plots...")
    create_all_plots(true_motion, imu, calibrated_imu, gnss, estimates, config)

    print("[8/8] Exporting time series for the GUI...")
    timeseries_path = export_timeseries(config, true_motion, gnss, estimates)

    print("\nDone.")
    print(f"Plots saved to: {config['output']['plot_dir']}")
    print(f"RMSE table saved to: {rmse_csv_path}")
    print(f"Time series saved to: {timeseries_path}")


if __name__ == "__main__":
    main()