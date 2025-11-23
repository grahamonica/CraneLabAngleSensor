import numpy as np
import math


def estimate_from_individual_projections(sensor_readings, sensor_positions):
    """
    Estimate position from sensor readings where angles are in degrees.
    """
    projections = []
    for i in range(4):
        dist, angle_degrees = sensor_readings[i]
        sx, sy = sensor_positions[i]

        px = sx + (dist) * math.cos(math.radians(angle_degrees))
        py = sy + (dist) * math.sin(math.radians(angle_degrees))
        print(
            f"Sensor {i + 1}: Position=({sx:.2f}, {sy:.2f}), Distance={dist:.2f}, Angle={angle_degrees:.2f}° => Projection=({px:.2f}, {py:.2f})"
        )
        projections.append([px, py])

    return robust_least_squares(np.array(projections))


def robust_least_squares(points, max_iter=10, tol=1e-6):
    # Start with equal weights
    weights = np.ones(len(points))
    prev_center = np.mean(points, axis=0)

    for _ in range(max_iter):
        # Weighted centroid
        weighted_sum = np.sum(points * weights[:, None], axis=0)
        new_center = weighted_sum / np.sum(weights)

        # Compute residuals (Euclidean distances to new center)
        residuals = np.linalg.norm(points - new_center, axis=1)

        # Update weights: inverse of residual, clipped to avoid division by zero
        weights = 1 / np.clip(residuals, 1e-3, None)

        # Check convergence
        if np.linalg.norm(new_center - prev_center) < tol:
            break
        prev_center = new_center

    return new_center
