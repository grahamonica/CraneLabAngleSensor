# Importing the simulated sensor data
import sensordata
import math
import statistics
import numpy as np
import trilaterate
import triangulate
import leastsquares
import individual
import dataformatter

# Inputs for the sensor data generation in inches
squareside = 20  # inches between sensors on the sides of a square
cablecenter = (10, 10)  # center of the simulated cable
cablerad = 21/25.4  # radius of the cable in inches (21mm converted to inches)
distancestddev = 0.0787402 * 2.5# standard deviation of the distance noise (this is 2mm -- now 5mm in inches)
anglestddev = math.pi / 180 # range of accuracy for the yaw angle in radians (this is 1 degree)

# Flag to switch between simulated and formatted data
use_formatted_data = True  # Set to True to use dataformatter.py output

if use_formatted_data:
    # Load formatted data from dataformatter.py
    formatted_data_file = 'formatted_sensor_data.txt'
    with open(formatted_data_file, 'r') as f:
        hundredtests = eval(f.read())
    
    # Define sensor positions manually (in inches)
    sensor_positions = [
        (0, 0),  # lower left
        (0, squareside),  # upper left
        (squareside, 0),  # lower right
        (squareside, squareside)  # upper right
    ]
else:
    # Use simulated data from sensordata.py
    hundredtests, sensor_positions = sensordata.generate_sensor_data(squareside, cablecenter, cablerad, distancestddev, anglestddev)

all_d1 = []
all_d2 = []
all_d3 = []

for i in range (hundredtests.__len__() // 4):
    # First we are going to use each sensor to identify the location individually
    sensor1pos = hundredtests[4 * i]
    sensor2pos = hundredtests[4 * i + 1]
    sensor3pos = hundredtests[4 * i + 2]
    sensor4pos = hundredtests[4 * i + 3]

    sensor_readings = [sensor1pos, sensor2pos, sensor3pos, sensor4pos]
    
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

