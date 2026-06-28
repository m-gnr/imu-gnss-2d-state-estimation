import numpy as np
import pandas as pd


def compute_rmse_table(
    true_motion: dict[str, np.ndarray],
    estimates: dict[str, dict[str, np.ndarray]],
) -> pd.DataFrame:
    """
    Compute RMSE values for each estimation method.
    """
    rows = []

    for method_name, estimate in estimates.items():
        x_rmse = _rmse(true_motion["x"], estimate["x"])
        y_rmse = _rmse(true_motion["y"], estimate["y"])
        vx_rmse = _rmse(true_motion["vx"], estimate["vx"])
        vy_rmse = _rmse(true_motion["vy"], estimate["vy"])

        position_rmse = np.sqrt(x_rmse**2 + y_rmse**2)
        velocity_rmse = np.sqrt(vx_rmse**2 + vy_rmse**2)

        rows.append(
            {
                "method": method_name,
                "x_rmse_m": x_rmse,
                "y_rmse_m": y_rmse,
                "position_rmse_m": position_rmse,
                "vx_rmse_mps": vx_rmse,
                "vy_rmse_mps": vy_rmse,
                "velocity_rmse_mps": velocity_rmse,
            }
        )

    return pd.DataFrame(rows)


def _rmse(reference: np.ndarray, estimate: np.ndarray) -> float:
    return float(np.sqrt(np.mean((reference - estimate) ** 2)))