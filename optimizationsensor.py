# analyze_lidar.py
import math
import csv
import statistics
import subprocess
from collections import defaultdict
import trilaterate
import triangulate
import individual

cableheight = 816.11 / 25.4
# Geometry (inches)
SQUARESIDE = 500 / 25.4
SENSOR_POS = [
    (SQUARESIDE / 2, SQUARESIDE / 2),  # 1
    (-SQUARESIDE / 2, SQUARESIDE / 2),  # 2
    (-SQUARESIDE / 2, -SQUARESIDE / 2),  # 3
    (SQUARESIDE / 2, -SQUARESIDE / 2),  # 4
]
CABLECENTER = (0.0, 0.0)  # set to None to skip distance-to-target


def run_formatter():
    print("Converting Excel → CSV...")
    subprocess.run(["python", "dataformatter.py"], check=True)


def load_tests(csv_path="formatted_sensor_data.csv"):
    tests = defaultdict(list)
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            t = int(r["Test"])
            tests[t].append(
                (float(r["Distance (inches)"]), float(r["Angle (degrees)"]))
            )
    return [v for _, v in sorted(tests.items()) if len(v) == 4]


def hypot2(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5


def main():
    run_formatter()
    all_tests = load_tests()
    print(f"Loaded {len(all_tests)} complete test sets")

    d1 = []
    d2 = []
    d3 = []

    for i, readings in enumerate(all_tests):
        distances = [d for d, _ in readings]
        angles = [a for _, a in readings]

        p1 = individual.estimate_from_individual_projections(readings, SENSOR_POS)
        p2 = trilaterate.trilaterate_least_squares(SENSOR_POS, distances)
        p3 = triangulate.robust_triangulation(SENSOR_POS, angles)

        print(f"\nTest {i}:")
        for name, p, bucket in [
            ("Individual Projections", p1, d1),
            ("Trilateration", p2, d2),
            ("Triangulation", p3, d3),
        ]:
            print(f"  {name:<30}: Position = ({p[0]:.3f}, {p[1]:.3f}) inches")
            if CABLECENTER is not None:
                d = hypot2(p, CABLECENTER)
                bucket.append(d)
                print(f"    Distance from target = {d:.3f} inches")

    def print_stats(label, arr):
        if not arr:
            return
        if len(arr) == 1:
            print(f"{label:<40} Avg = {arr[0]:.3f} in, Std Dev = n/a")
        else:
            print(
                f"{label:<40} Avg = {statistics.mean(arr):.3f} in, Std Dev = {statistics.stdev(arr):.3f} in"
            )

    print("\n--- Summary Statistics ---")
    print_stats("Individual projections (least squares):", d1)
    print_stats("Trilateration (distance-only):", d2)
    print_stats("Triangulation (angle-only):", d3)
    # angled1 is the angle from vertical of the cable
    angled1 = [math.degrees(math.atan2(d, cableheight)) for d in d1]
    print_stats("Individual projections (angle from vertical):", angled1)
    angled2 = [math.degrees(math.atan2(d, cableheight)) for d in d2]
    print_stats("Trilateration (angle from vertical):", angled2)
    angled3 = [math.degrees(math.atan2(d, cableheight)) for d in d3]
    print_stats("Triangulation (angle from vertical):", angled3)


if __name__ == "__main__":
    main()
