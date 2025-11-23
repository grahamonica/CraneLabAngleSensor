import numpy as np
from scipy.optimize import minimize


def trilaterate_least_squares(sensor_positions, distances):
    def objective(point):
        x, y = point
        error = 0
        for (sx, sy), distance in zip(sensor_positions, distances):
            calculated_dist = np.sqrt((x - sx) ** 2 + (y - sy) ** 2)
            error += (calculated_dist - distance) ** 2
        return error

    # Initial guess: center
    initial_guess = np.mean(sensor_positions, axis=0)

    # Optimize to find the point that best fits all distance measurements
    result = minimize(objective, initial_guess, method="Nelder-Mead")

    return (result.x[0], result.x[1])
