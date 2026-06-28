from pathlib import Path

import numpy as np

from src.config_loader import load_config
from src.route_generator import generate_true_motion, print_motion_checkpoints
from src.sensor_models import generate_imu, generate_gnss
from src.faults import apply_gnss_delay, apply_gnss_faults
from src.calibration import calibrate_imu, compute_calibration_error_stats
from src.filters import raw_gnss_estimate, low_pass_filter_estimate, complementary_filter
from src.metrics import compute_rmse_table
from src.plotting import create_all_plots


def ensure_output_dirs(config: dict) -> None:
    Path(config["output"]["plot_dir"]).mkdir(parents=True, exist_ok=True)
    Path(config["output"]["data_dir"]).mkdir(parents=True, exist_ok=True)


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

    estimates = {
        "Raw GNSS": raw_estimate,
        "Low-pass Filter": lpf_estimate,
        "Complementary Filter": comp_estimate,
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

    print("[7/7] Creating plots...")
    create_all_plots(true_motion, imu, calibrated_imu, gnss, estimates, config)

    print("\nDone.")
    print(f"Plots saved to: {config['output']['plot_dir']}")
    print(f"RMSE table saved to: {rmse_csv_path}")


if __name__ == "__main__":
    main()