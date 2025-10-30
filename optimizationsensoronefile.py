# analyze_lidar.py
import math
import csv, statistics, subprocess
from collections import defaultdict
import trilaterate, triangulate, individual

cableheight = 816.11/25.4
# Geometry (inches)
SQUARESIDE = 500/25.4
SENSOR_POS = [
    ( SQUARESIDE/2,  SQUARESIDE/2),   # 1
    (-SQUARESIDE/2,  SQUARESIDE/2),   # 2
    (-SQUARESIDE/2, -SQUARESIDE/2),   # 3
    ( SQUARESIDE/2, -SQUARESIDE/2),   # 4
]
CABLECENTER = None

def run_formatter():
    print("Converting Excel → CSV...")
    subprocess.run(["python", "dataformatter.py"], check=True)

def load_tests(csv_path="formatted_sensor_data.csv"):
    tests = defaultdict(list)
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            t = int(r["Test"])
            tests[t].append((float(r["Distance (inches)"]), float(r["Angle (degrees)"])))
    return [v for _, v in sorted(tests.items()) if len(v) == 4]

def hypot2(a, b):
    dx, dy = a[0]-b[0], a[1]-b[1]
    return (dx*dx + dy*dy) ** 0.5

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
        py =sy + (dist ) * math.sin(math.radians(angle_degrees))
        print(f"Sensor {i+1}: Position=({sx:.2f}, {sy:.2f}), Distance={dist:.2f}, Angle={angle_degrees:.2f}° => Projection=({px:.2f}, {py:.2f})")
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

import numpy as np
from scipy.optimize import minimize

def trilaterate_least_squares(sensor_positions, distances):

    def objective(point):
        x, y = point
        error = 0
        for (sx, sy), distance in zip(sensor_positions, distances):
            calculated_dist = np.sqrt((x - sx)**2 + (y - sy)**2)
            error += (calculated_dist - distance)**2
        return error

    # Initial guess: center
    initial_guess = np.mean(sensor_positions, axis=0)
    
    # Optimize to find the point that best fits all distance measurements
    result = minimize(objective, initial_guess, method='Nelder-Mead')
    
    return (result.x[0], result.x[1])


def main(distancesarray, anglesarray):

    d1 = []; d2 = []; d3 = []

    readings = [[distancesarray[j], anglesarray[j]] for j in range(4)]

    distances = distancesarray
    angles    = anglesarray
    p1 = individual.estimate_from_individual_projections(readings, SENSOR_POS)
    p2 = trilaterate.trilaterate_least_squares(SENSOR_POS, distances)
    p3 = triangulate.robust_triangulation(SENSOR_POS, angles)

    print(f"\nTest {i}:")
    for name, p, bucket in [
        ("Individual Projections", p1, d1),
        ("Trilateration",         p2, d2),
        ("Triangulation",         p3, d3),
    ]:
        print(f"  {name:<30}: Position = ({p[0]:.3f}, {p[1]:.3f}) inches")
        if CABLECENTER is not None:
            d = hypot2(p, CABLECENTER); bucket.append(d)
            print(f"    Distance from target = {d:.3f} inches")

    def print_stats(label, arr):
        if not arr: return
        if len(arr) == 1:
            print(f"{label:<40} Avg = {arr[0]:.3f} in, Std Dev = n/a")
        else:
            print(f"{label:<40} Avg = {statistics.mean(arr):.3f} in, Std Dev = {statistics.stdev(arr):.3f} in")

    print("\n--- Summary Statistics ---")
    print_stats("Individual projections (least squares):", d1)
    print_stats("Trilateration (distance-only):",         d2)
    print_stats("Triangulation (angle-only):",            d3)
    #angled1 is the angle from vertical of the cable
    angled1 = [math.degrees(math.atan2(d, cableheight)) for d in d1]
    print_stats("Individual projections (angle from vertical):", angled1)
    angled2 = [math.degrees(math.atan2(d, cableheight)) for d in d2]
    print_stats("Trilateration (angle from vertical):", angled2)
    angled3 = [math.degrees(math.atan2(d, cableheight)) for d in d3]
    print_stats("Triangulation (angle from vertical):", angled3)
if __name__ == "__main__":
    main()
