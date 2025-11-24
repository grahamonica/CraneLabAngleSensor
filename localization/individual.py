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

    projections = np.array(projections)
    faulty = find_faulty_sensors(projections)
    for i in faulty:
        print(f"WARNING: Sensor {i + 1} disagrees with the other sensors, excluding it")
    good = [i for i in range(len(projections)) if i not in faulty]
    if len(good) < 2:
        good = list(range(len(projections)))  # can't tell who is wrong, use all
    return robust_least_squares(projections[good])


FAULT_THRESHOLD_IN = 2.5  # healthy sensors stay within ~1.9 in of each other in the recorded data


def find_faulty_sensors(points, threshold=FAULT_THRESHOLD_IN):
    """Each sensor gives its own estimate of the cable position, so the
    others can check it: flag a sensor whose point is far from the median of
    the other sensors' points."""
    faulty = []
    for i in range(len(points)):
        others = np.delete(points, i, axis=0)
        if np.linalg.norm(points[i] - np.median(others, axis=0)) > threshold:
            faulty.append(i)
    return faulty


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
