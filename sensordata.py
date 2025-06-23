import math
import random

def generate_sensor_data(squareside, cablecenter, cablerad, distancestddev, anglestddev):

    # Sensor positions
    sensor_positions = [
        (0, 0),  # lower left
        (0, squareside),  # upper left
        (squareside, 0),  # lower right
        (squareside, squareside),  # upper right
    ]

    # Theoretical measurements for each sensor
    sensors = []
    for x, y in sensor_positions:
        dx = cablecenter[0] - x
        dy = cablecenter[1] - y
        distance = (dx**2 + dy**2)**0.5 - cablerad
        angle = math.atan2(dy, dx)
        sensors.append([distance, angle])

    hundredtests = []

    for i in range(100000):
        test = []
        for sensor in sensors:
            normaldistance = random.gauss(0, distancestddev)
            normalangle = random.gauss(0, anglestddev)
            test.append([sensor[0] + normaldistance, sensor[1] + normalangle])
        hundredtests.append(test)
    
        # Flatten tests into one list of 400 [dist, angle] entries
    flat_tests = [pair for test in hundredtests for pair in test]
    return flat_tests, sensor_positions
