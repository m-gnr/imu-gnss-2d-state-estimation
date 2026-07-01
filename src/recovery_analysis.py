"""
t=505s (GNSS freeze end) recovery analysis.

Freeze fault (500-505s, config.yaml faults.freeze) overlaps with a
100->50 km/h braking maneuver (500-510s). This script quantifies, per
method, how badly position error spikes right as the freeze ends and how
long it takes to settle back down, using the already-exported
outputs/data/timeseries.npz (no re-simulation).
"""

from pathlib import Path

import numpy as np
import pandas as pd

FREEZE_END = 505.0
NORMAL_LEVEL_MULT = 1.5
# Error must stay under the normal level for this long (settling time style
# criterion) before we call it "recovered" -- otherwise a single lucky sample
# right after the freeze (or a later noise spike) would count as recovery.
HOLD_SECONDS = 5.0


def load_timeseries(path: str | Path = "outputs/data/timeseries.npz") -> dict:
    data = np.load(path, allow_pickle=True)
    return {k: data[k] for k in data.files}


def compute_recovery_table(
    data: dict,
    freeze_end: float = FREEZE_END,
    normal_level_mult: float = NORMAL_LEVEL_MULT,
    hold_seconds: float = HOLD_SECONDS,
) -> pd.DataFrame:
    t = data["t"]
    methods = list(data["methods"])
    dt = t[1] - t[0]
    hold_samples = max(1, int(round(hold_seconds / dt)))

    # Pre-freeze baseline window: starts at 420s (after the 400-401s position
    # jump has fully decayed out of the filters) and ends right before freeze.
    pre_mask = (t >= 420.0) & (t < 500.0)

    rows = []
    for i, name in enumerate(methods):
        err = np.sqrt(
            (data["est_x"][i] - data["true_x"]) ** 2
            + (data["est_y"][i] - data["true_y"]) ** 2
        )

        baseline = err[pre_mask].mean()
        normal_level = normal_level_mult * baseline

        end_idx = int(np.searchsorted(t, freeze_end))
        instant_error = err[end_idx]

        post_t = t[end_idx:]
        post_err = err[end_idx:]

        # Recovery time: first index after which the error stays under
        # normal_level for at least `hold_seconds` in a row.
        below = post_err <= normal_level
        recovery_idx = None
        n = len(post_err)
        for idx in range(n - hold_samples + 1):
            if below[idx : idx + hold_samples].all():
                recovery_idx = idx
                break
        if recovery_idx is None:
            recovery_time = float("nan")
            max_error_during_recovery = post_err.max()
        else:
            recovery_time = post_t[recovery_idx] - freeze_end
            max_error_during_recovery = post_err[: recovery_idx + 1].max()

        rows.append(
            {
                "Yöntem": name,
                "Anlık Hata @505s [m]": instant_error,
                "Toparlanma Süresi [s]": recovery_time,
                "Toparlanma Sırasında Maks. Hata [m]": max_error_during_recovery,
                "Freeze Öncesi Ort. Hata [m]": baseline,
            }
        )

    return pd.DataFrame(rows)


def main() -> None:
    data = load_timeseries()
    table = compute_recovery_table(data)
    pd.set_option("display.float_format", lambda x: f"{x:.3f}")
    print("\nt=505s (GNSS freeze bitişi) Toparlanma Analizi")
    print(f"Normal seviye eşiği: freeze öncesi ortalama hatanın {NORMAL_LEVEL_MULT}x katı\n")
    print(table.to_string(index=False))


if __name__ == "__main__":
    main()
