# Importing the simulated sensor data
import sensordata
import math
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

for i in range (100):
    # First we are going to use each sensor to identify the location individually
    sensor1pos = hundredtests[4 * i]
    sensor2pos = hundredtests[4 * i + 1]
    sensor3pos = hundredtests[4 * i + 2]
    sensor4pos = hundredtests[4 * i + 3]

    cable1x = sensor1pos[0] * math.cos(sensor1pos[1])
    cable1y = sensor1pos[0] * math.sin(sensor1pos[1])

    cable2x = sensor2pos[0] * math.cos(sensor2pos[1])
    cable2y = sensor2pos[0] * math.sin(sensor2pos[1])

    cable3x = sensor3pos[0] * math.cos(sensor3pos[1])
    cable3y = sensor3pos[0] * math.sin(sensor3pos[1])

    cable4x = sensor4pos[0] * math.cos(sensor4pos[1])
    cable4y = sensor4pos[0] * math.sin(sensor4pos[1])

    estimated_center1 = leastsquares.robust_least_squares(np.array([
        [cable1x, cable1y],
        [cable2x, cable2y],
        [cable3x, cable3y],
        [cable4x, cable4y]
    ]))

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

    print(f"Test {i}:")
    print(f"  From direct sensor projections (least squares):    Distance = {d1:.3f} inches")
    print(f"  From trilateration (distance only):               Distance = {d2:.3f} inches")
    print(f"  From triangulation (angle only):                  Distance = {d3:.3f} inches")

