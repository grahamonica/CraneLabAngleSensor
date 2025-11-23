"""Synthetic cable-position log for developing the snag detector before a
labelled field recording exists (same role the old sensordata.py simulator
played for localization). Replace with real logs once you have them.

Normal motion: pendulum sway at the cable's natural frequency with a slowly
wandering amplitude, slow drift of the center, and localization noise.
A snag: the cable is pulled quickly to an off-center position, held there with
almost no sway (it is pinned), then released into a larger swing that decays.
Label is 1 from onset until release and 0 otherwise. The snap-back swing is
deliberately labelled 0 so the model learns that a big, fast swing on its own
is not a snag.

Units match the rest of the repo: inches, seconds, 10 Hz.
"""
import numpy as np
import pandas as pd

CABLE_HEIGHT_M = 816.11 / 1000
PENDULUM_HZ = np.sqrt(9.81 / CABLE_HEIGHT_M) / (2 * np.pi)  # ~0.55 Hz


def simulate(duration_s=1200.0, rate_hz=10.0, mean_gap_s=60.0, noise_in=0.1, seed=0):
    rng = np.random.default_rng(seed)
    n = int(duration_s * rate_hz)
    dt = 1.0 / rate_hz
    t = np.arange(n) * dt

    # Normal sway, 0.15 to 0.45 in amplitude, plus a slow random-walk drift.
    amp = 0.3 * (1 + 0.5 * np.sin(2 * np.pi * t / 300 + rng.uniform(0, 2 * np.pi)))
    x = amp * np.sin(2 * np.pi * PENDULUM_HZ * t + rng.uniform(0, 2 * np.pi))
    y = amp * np.sin(2 * np.pi * PENDULUM_HZ * t + rng.uniform(0, 2 * np.pi))
    drift = np.cumsum(rng.normal(0, 0.002, size=(n, 2)), axis=0)
    x += drift[:, 0]
    y += drift[:, 1]

    snag = np.zeros(n, dtype=int)
    i = int(rng.exponential(mean_gap_s) * rate_hz)
    while i < n - int(10 * rate_hz):
        rise = int(rng.uniform(0.2, 0.6) * rate_hz)
        hold = int(rng.uniform(1.0, 4.0) * rate_hz)
        mag, ang = rng.uniform(1.5, 5.0), rng.uniform(0, 2 * np.pi)
        ox, oy = mag * np.cos(ang), mag * np.sin(ang)

        # Onset: ramp to the offset on top of the existing sway.
        ramp = np.linspace(0, 1, rise)
        x[i : i + rise] += ox * ramp
        y[i : i + rise] += oy * ramp

        # Hold: pinned at the offset, sway replaced by small jitter.
        h0, h1 = i + rise, i + rise + hold
        x[h0:h1] = ox + drift[h0:h1, 0] + rng.normal(0, 0.03, h1 - h0)
        y[h0:h1] = oy + drift[h0:h1, 1] + rng.normal(0, 0.03, h1 - h0)
        snag[i:h1] = 1

        # Release: decaying swing back toward center (labelled normal).
        r0, r1 = h1, min(h1 + int(6.0 * rate_hz), n)
        tt = np.arange(r1 - r0) * dt
        env = np.exp(-tt / 2.0) * np.cos(2 * np.pi * PENDULUM_HZ * tt)
        x[r0:r1] += ox * env
        y[r0:r1] += oy * env

        i = r1 + int(rng.exponential(mean_gap_s) * rate_hz)

    x += rng.normal(0, noise_in, n)
    y += rng.normal(0, noise_in, n)
    return pd.DataFrame({"t": t, "x": x, "y": y, "snag": snag})


if __name__ == "__main__":
    df = simulate()
    print(df.describe())
    print(f"snag rows: {int(df['snag'].sum())} of {len(df)}")
