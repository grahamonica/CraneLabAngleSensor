"""Causal (trailing-window) features for snag detection.

Input: a DataFrame with columns ``t`` (seconds), ``x``, ``y`` (cable position in
inches in the sensor plane, from any of the localization methods). Every
feature at row i uses only rows <= i, so the same function works offline on a
recorded log and online on a rolling buffer (take the last row).

Angle from vertical is atan2(r, cable height), a monotonic function of ``r``,
so it is not a separate feature: tree models are invariant to it.
"""
import numpy as np
import pandas as pd

SAMPLE_RATE_HZ = 10  # CYCLE_PERIOD = 0.10 s in firmware/circuitpython/code.py
SHORT_WINDOW = 10    # 1 s: catches the fast onset of a snag
LONG_WINDOW = 50     # 5 s: baseline the cable has drifted away from

FEATURE_NAMES = [
    "r",                # distance from the calibrated center (in)
    "speed",            # |velocity| (in/s)
    "accel",            # |change in velocity| / dt (in/s^2)
    "r_dev_short",      # r minus its 1 s trailing mean
    "r_dev_long",       # r minus its 5 s trailing mean
    "r_std_short",      # 1 s trailing std of r (sway vs. pinned)
    "r_range_short",    # 1 s trailing max - min of r
    "speed_max_short",  # 1 s trailing max speed
]


def build_features(df, short=SHORT_WINDOW, long=LONG_WINDOW):
    t = df["t"].to_numpy(float)
    x = df["x"].to_numpy(float)
    y = df["y"].to_numpy(float)

    dt = np.diff(t, prepend=t[0])
    default_dt = float(np.median(dt[1:])) if len(dt) > 1 else 1.0 / SAMPLE_RATE_HZ
    dt = np.where(dt > 0, dt, default_dt)

    r = np.hypot(x, y)
    vx = np.diff(x, prepend=x[0]) / dt
    vy = np.diff(y, prepend=y[0]) / dt
    speed = np.hypot(vx, vy)
    accel = np.hypot(np.diff(vx, prepend=vx[0]), np.diff(vy, prepend=vy[0])) / dt

    r_s = pd.Series(r)
    roll_short = r_s.rolling(short, min_periods=1)
    roll_long = r_s.rolling(long, min_periods=1)

    out = pd.DataFrame(
        {
            "r": r,
            "speed": speed,
            "accel": accel,
            "r_dev_short": r - roll_short.mean().to_numpy(),
            "r_dev_long": r - roll_long.mean().to_numpy(),
            "r_std_short": roll_short.std().fillna(0.0).to_numpy(),
            "r_range_short": (roll_short.max() - roll_short.min()).to_numpy(),
            "speed_max_short": pd.Series(speed).rolling(short, min_periods=1).max().to_numpy(),
        },
        index=df.index,
    )
    return out[FEATURE_NAMES]
