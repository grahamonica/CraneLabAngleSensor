import math
import numpy as np

def triangulation_residual(point, sensor_positions, angles):
    """Returns residuals: perpendicular distances from point to each sensor ray."""
    px, py = point
    residuals = []
    for (sx, sy), theta in zip(sensor_positions, angles):
        dx = math.cos(theta)
        dy = math.sin(theta)
        # Vector from sensor to point
        vx = px - sx
        vy = py - sy
        # Perpendicular distance from point to ray (cross product / norm)
        perp_dist = abs(vx * dy - vy * dx)
        residuals.append(perp_dist)
    return np.array(residuals)

def robust_triangulation(sensor_positions, angles, max_iter=10, tol=1e-6):
    # Initialize with geometric center
    center = np.mean(sensor_positions, axis=0)
    weights = np.ones(len(sensor_positions))

    for _ in range(max_iter):
        residuals = triangulation_residual(center, sensor_positions, angles)
        weights = 1 / np.clip(residuals, 1e-3, None)

        # Weighted least squares update
        A = []
        b = []
        for (sx, sy), theta, w in zip(sensor_positions, angles, weights):
            dx = math.cos(theta)
            dy = math.sin(theta)
            # Line is: (x - sx)*dy - (y - sy)*dx = 0  → linear form: dy*x - dx*y = dy*sx - dx*sy
            A.append([dy * w, -dx * w])
            b.append((dy * sx - dx * sy) * w)
        A = np.array(A)
        b = np.array(b).reshape(-1, 1)

        new_center, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        new_center = new_center.flatten()

        if np.linalg.norm(new_center - center) < tol:
            break
        center = new_center

    return center
