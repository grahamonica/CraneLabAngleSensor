# Importing the simulated sensor data
import sensordata
import math
import statistics
import numpy as np
import trilaterate
import triangulate
import leastsquares
import individual

# Inputs for the sensor data generation in inches
squareside = 24  # inches between sensors on the sides of a square
cablecenter = (11.5, 12.5)  # center of the simulated cable
cablerad = .25  # radius of the cable
distancestddev = 1 # standard deviation of the distance noise
anglestddev = math.pi / 100 # standard deviation of the angle noise in radians

hundredtests, sensor_positions = sensordata.generate_sensor_data(squareside, cablecenter, cablerad, distancestddev, anglestddev)

all_d1 = []
all_d2 = []
all_d3 = []

for i in range (100):
    # First we are going to use each sensor to identify the location individually
    sensor1pos = hundredtests[4 * i]
    sensor2pos = hundredtests[4 * i + 1]
    sensor3pos = hundredtests[4 * i + 2]
    sensor4pos = hundredtests[4 * i + 3]

    sensor_readings = [
        hundredtests[4 * i],
        hundredtests[4 * i + 1],
        hundredtests[4 * i + 2],
        hundredtests[4 * i + 3]
    ]

    estimated_center1 = individual.estimate_from_individual_projections(sensor_readings, sensor_positions)

    # Now we are going to use combos of 3 sensors to identify the location with trilateration
    distances = [sensor1pos[0], sensor2pos[0], sensor3pos[0], sensor4pos[0]]
    estimated_center2 = trilaterate.trilaterate_least_squares(sensor_positions, distances)

    # same thing but with triangulation
    angles = [sensor1pos[1], sensor2pos[1], sensor3pos[1], sensor4pos[1]]
    estimated_center3 = triangulate.robust_triangulation(sensor_positions, angles)

    # Print the results
    def distance(p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

    d1 = distance(estimated_center1, cablecenter)
    d2 = distance(estimated_center2, cablecenter)
    d3 = distance(estimated_center3, cablecenter)
    all_d1.append(d1)
    all_d2.append(d2)
    all_d3.append(d3)

    print(f"Test {i}:")
    print(f"  From direct sensor projections (least squares):    Distance = {d1:.3f} inches")
    print(f"  From trilateration (distance only):               Distance = {d2:.3f} inches")
    print(f"  From triangulation (angle only):                  Distance = {d3:.3f} inches")

print("\n--- Summary Over 100 Tests ---")
def print_stats(label, data):
    avg = statistics.mean(data)
    std = statistics.stdev(data)
    print(f"{label:<40} Avg = {avg:.3f} in, Std Dev = {std:.3f} in")

print_stats("Individual projections (least squares):", all_d1)
print_stats("Trilateration (distance-only):", all_d2)
print_stats("Triangulation (angle-only):", all_d3)

