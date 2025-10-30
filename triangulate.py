# triangulate.py
import math
import numpy as np

def _solve(sensor_positions, angles_rad):
    # Line normal for bearing theta: n = (-sinθ, cosθ)
    A = []
    b = []
    for (x0, y0), th in zip(sensor_positions, angles_rad):
        n = (-math.sin(th), math.cos(th))
        A.append(n)
        b.append(n[0]*x0 + n[1]*y0)
    A = np.asarray(A, float)     # (m,2)
    b = np.asarray(b, float)     # (m,)
    # Least-squares solution to A x ≈ b
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = A @ x - b
    rss = float(resid @ resid)
    return (float(x[0]), float(x[1])), rss

def robust_triangulation(sensor_positions, angles):
    # Hypothesis A: 0° along +x, CCW
    angA = [math.radians(a) for a in angles]
    xA, rssA = _solve(sensor_positions, angA)
    print(f"Triangulation Hyp A: Pos=({xA[0]:.3f}, {xA[1]:.3f}), RSS={rssA:.3f}")

    # Hypothesis B: 0° along +y, CCW  ≡ A - 90°
    angB = [math.radians(a - 90.0) for a in angles]
    xB, rssB = _solve(sensor_positions, angB)
    print(f"Triangulation Hyp B: Pos=({xB[0]:.3f}, {xB[1]:.3f}), RSS={rssB:.3f}")

    return xA if rssA <= rssB else xB
