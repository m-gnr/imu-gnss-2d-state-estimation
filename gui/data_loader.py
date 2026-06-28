"""Load and serve the exported simulation time series for the GUI."""

from pathlib import Path

import numpy as np


class SimData:
    """
    Holds the full time series and provides interpolated lookups at an
    arbitrary simulation time (for smooth playback between samples).
    """

    def __init__(self, npz_path: str | Path):
        npz_path = Path(npz_path)
        if not npz_path.exists():
            raise FileNotFoundError(
                f"Time-series file not found: {npz_path}\n"
                f"Run the pipeline first:  python main.py"
            )

        d = np.load(npz_path, allow_pickle=True)

        self.t = d["t"].astype(float)
        self.n = len(self.t)
        self.t0 = float(self.t[0])
        self.t1 = float(self.t[-1])
        self.dt = float(self.t[1] - self.t[0])

        self.true_x = d["true_x"]
        self.true_y = d["true_y"]
        self.true_psi = d["true_psi"]
        self.true_v = d["true_v"]

        self.methods = [str(m) for m in d["methods"]]
        self.est_x = d["est_x"]            # shape (m, n)
        self.est_y = d["est_y"]
        self.est_psi = d["est_psi"]
        self.est_vx = d["est_vx"]
        self.est_vy = d["est_vy"]

        self.gnss_t = d["gnss_t"].astype(float)
        self.gnss_x = d["gnss_x"]
        self.gnss_y = d["gnss_y"]
        self.gnss_valid = d["gnss_valid"].astype(bool)

        # Precomputed per-method 2D position error (for the strip chart).
        self.pos_error = np.hypot(
            self.est_x - self.true_x, self.est_y - self.true_y
        )
        self.est_speed = np.hypot(self.est_vx, self.est_vy)

    def method_index(self, name: str) -> int:
        return self.methods.index(name)

    def frac_to_index(self, sim_t: float) -> float:
        """Continuous fractional index for a given simulation time."""
        f = (sim_t - self.t0) / self.dt
        return float(np.clip(f, 0.0, self.n - 1))

    def true_state(self, sim_t: float) -> dict:
        """Interpolated ground-truth state at sim_t."""
        f = self.frac_to_index(sim_t)
        i = int(f)
        j = min(i + 1, self.n - 1)
        a = f - i
        return {
            "x": _lerp(self.true_x[i], self.true_x[j], a),
            "y": _lerp(self.true_y[i], self.true_y[j], a),
            "psi": _lerp_angle(self.true_psi[i], self.true_psi[j], a),
            "v": _lerp(self.true_v[i], self.true_v[j], a),
        }

    def est_state(self, method_idx: int, sim_t: float) -> dict:
        """Interpolated estimate state for one method at sim_t."""
        f = self.frac_to_index(sim_t)
        i = int(f)
        j = min(i + 1, self.n - 1)
        a = f - i
        return {
            "x": _lerp(self.est_x[method_idx, i], self.est_x[method_idx, j], a),
            "y": _lerp(self.est_y[method_idx, i], self.est_y[method_idx, j], a),
            "psi": _lerp_angle(
                self.est_psi[method_idx, i], self.est_psi[method_idx, j], a
            ),
        }

    def pos_error_at(self, method_idx: int, sim_t: float) -> float:
        f = self.frac_to_index(sim_t)
        i = int(round(f))
        return float(self.pos_error[method_idx, i])


def _lerp(a: float, b: float, t: float) -> float:
    return float(a + (b - a) * t)


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Interpolate angles along the shortest arc."""
    d = (b - a + np.pi) % (2 * np.pi) - np.pi
    return float(a + d * t)
