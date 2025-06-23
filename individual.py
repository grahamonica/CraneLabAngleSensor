import numpy as np
import math
from leastsquares import robust_least_squares

def estimate_from_individual_projections(sensor_readings, sensor_positions):
    projections = []
    for i in range(4):
        dist, angle = sensor_readings[i]
        sx, sy = sensor_positions[i]

        px = sx + dist * math.cos(angle)
        py = sy + dist * math.sin(angle)
        projections.append([px, py])

    return robust_least_squares(np.array(projections))