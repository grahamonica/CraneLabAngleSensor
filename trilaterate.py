import numpy as np
def trilaterate_least_squares(sensor_positions, distances):
    A = []
    b = []
    
    x0, y0 = sensor_positions[0]
    r0 = distances[0]

    for i in range(1, len(sensor_positions)):
        xi, yi = sensor_positions[i]
        ri = distances[i]

        A.append([2 * (xi - x0), 2 * (yi - y0)])
        b.append(
            r0**2 - ri**2 - x0**2 + xi**2 - y0**2 + yi**2
        )

    A = np.array(A)
    b = np.array(b).reshape(-1, 1)

    # Solve (AᵀA)x = Aᵀb
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return x.flatten()
