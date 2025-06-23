import numpy as np

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
