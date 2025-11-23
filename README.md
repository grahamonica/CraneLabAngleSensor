# Crane Lab Angle Sensor

Four LiDAR sensors mounted in a square around a crane cable measure the cable's
position in the sensor plane, which gives the cable's angle from vertical. This
repo holds the sensor firmware, host-side code that compares three ways of
localizing the cable from the four readings, and an XGBoost snag detector that
runs on the localized positions.

## Layout

```
firmware/
  LidarCommunicationFinal/   Teensy sketch, one per LiDAR: parses the LiDAR packet
                             stream and sends the closest point once per revolution
  Lidar_Receiver_Final/      Teensy sketch: reads four sensors, calibrates, trilaterates
  circuitpython/code.py      CircuitPython receiver (4 UARTs) running all three
                             localization methods live; uses ulab's numpy only
localization/
  individual.py              per-sensor projections + robust least squares
  trilaterate.py             distance-only, Nelder-Mead least squares
  triangulate.py             angle-only, linear least squares over two axis conventions
  dataformatter.py           data/LiDAR_Data_Collection.xlsx -> data/formatted_sensor_data.csv
  optimizationsensor.py      runs the three methods on the CSV and prints error stats
data/                        recorded spreadsheet ("Object In Center") and the CSV built from it
snag_detection/
  features.py                trailing-window features from a (t, x, y) position log
  simulate.py                synthetic log with labelled snags, for development
  train.py                   trains the XGBoost classifier; models/ is gitignored
```

## Setup

```bash
python3 -m pip install -r requirements.txt
brew install libomp        # macOS only: xgboost needs the OpenMP runtime
```

The firmware needs the Arduino IDE (Teensy sketches, `BasicLinearAlgebra`
library for the receiver) or CircuitPython with `ulab` for `code.py`.

## Localization

Geometry lives at the top of `localization/optimizationsensor.py` and
`firmware/circuitpython/code.py`: sensors at the corners of a 500 mm square
centered on the origin, cable height 816.11 mm. Angles are in degrees,
distances in inches after conversion.

Three methods are compared:

- Individual projections: each sensor's (distance, angle) is projected to a point, then a robust weighted least-squares centroid is taken.
- Trilateration: distances only, least-squares fit of a point to the four range circles.
- Triangulation: angles only, intersection of the four bearing lines, solved under both 0°-along-x and 0°-along-y conventions and keeping the lower residual.

```bash
python3 localization/optimizationsensor.py
```

This regenerates the CSV from the spreadsheet, prints each method's position
for every test set, and finishes with the mean and standard deviation of the
distance from the known center plus the equivalent angle from vertical.

## Snag detection

A snag is the cable being caught and pulled off-center quickly, then held
there. The detector works on the position log the localization code produces
(one row per 10 Hz cycle: `t`, `x`, `y`) and flags each row as snag or not.

- `features.py` builds causal features from a trailing window: radial offset `r`, speed, acceleration, deviation of `r` from its 1 s and 5 s trailing means, and 1 s rolling std, range and max speed. Causal means the same code works on a live rolling buffer.
- `train.py` fits an `XGBClassifier` with `scale_pos_weight` for the rare positive class and evaluates on a chronological hold-out (the last 25% of the log), which is the honest split for time-series data.
- `simulate.py` generates a labelled log until real snag recordings exist.

```bash
python3 snag_detection/train.py --synthetic           # develop against simulated data
python3 snag_detection/train.py data/my_log.csv       # columns: t, x, y, snag (0/1)
```

To label a real recording, log the positions during a test, mark the rows from
snag onset to release as 1, and pass the CSV to `train.py`. Loading and using
the saved model is shown in the docstring of `train.py`.

## Firmware notes

Each LiDAR Teensy sends a 6-byte packet per revolution: `float` angle in
degrees followed by `uint16` distance in mm (`struct` format `<fH`), at
115200 baud. The LiDAR itself talks to the Teensy at 921600 baud.
