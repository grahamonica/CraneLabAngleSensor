# Crane Lab Angle Sensor

This project implements various methods for estimating the position of a cable using sensor data from four sensors arranged in a square configuration. The project compares three different approaches to position estimation:

1. Individual Sensor Projections
2. Trilateration (using distance measurements)
3. Triangulation (using angle measurements)

## Project Structure

- `sensordata.py` - Generates simulated sensor data with configurable parameters
- `individual.py` - Implements position estimation using individual sensor projections and least squares
- `trilaterate.py` - Implements trilateration using distance measurements
- `triangulate.py` - Implements triangulation using angle measurements
- `leastsquares.py` - Contains robust least squares implementation used by other modules
- `optimizationsensor.py` - Main script that runs comparisons between different methods

## Configuration

The simulation can be configured with the following parameters (found in `optimizationsensor.py`):

- Square side length: 24 inches (distance between sensors)
- Cable center: (11.5, 12.5) inches
- Cable radius: 0.25 inches
- Distance standard deviation: ~2mm (0.0787402 * 2.5 inches)
- Angle standard deviation: 1 degree (π/180 radians)

## Methods

### Individual Projections
Uses direct projections from each sensor and combines them using robust least squares to estimate the position.

### Trilateration
Uses only distance measurements from the sensors and implements a least squares solution to estimate position.

### Triangulation
Uses only angle measurements and implements robust triangulation with weighted least squares.

## Usage

Run the optimization sensor script to compare the three methods:

```bash
python optimizationsensor.py
```

The script will run 100,000 tests and output:
- Individual test results showing the distance error for each method
- Summary statistics including average error and standard deviation for each approach

## Implementation Details

- All implementations use numpy for efficient matrix operations
- Robust estimation techniques are used to handle noise in measurements
- The project includes simulated noise in both distance and angle measurements
- Error calculations are performed using Euclidean distance from true center
