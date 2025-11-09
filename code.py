import board
import math
from ulab import numpy as np
import busio
import digitalio
import struct
import time

# === UART setup (4 hardware ports) ===
uart1 = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.01)
uart2 = busio.UART(board.TX2, board.RX2, baudrate=115200, timeout=0.01)
uart3 = busio.UART(board.TX3, board.RX3, baudrate=115200, timeout=0.01)
uart4 = busio.UART(board.TX4, board.RX4, baudrate=115200, timeout=0.01)

# === LED setup ===
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
led.value = True

cableheight = 816.11 / 25.4
# Geometry (inches)
SQUARESIDE = 500 / 25.4
SENSOR_POS = [
    (SQUARESIDE / 2, SQUARESIDE / 2),  # 1
    (-SQUARESIDE / 2, SQUARESIDE / 2),  # 2
    (-SQUARESIDE / 2, -SQUARESIDE / 2),  # 3
    (SQUARESIDE / 2, -SQUARESIDE / 2),  # 4
]
CABLECENTER = None
OFFSETS_DEG = [180.0, 180.0, 0.0, 0.0]  # sensors 1–2: +180°, 3–4: +0°
RADIUS_MM = 4.5
MM2IN = 1 / 25.4

# CABLECENTER = (0.0, 0.0) 

PACKET_FORMAT = "<fH"
PACKET_SIZE = 6

angles = np.zeros(4)
distances = np.zeros(4)


# ==== Functions ====
def read_uart(uart, label):
    """Buffer bytes and extract full 6-byte packets."""
    data = uart.read(64)
    if data:
        buffers[label].extend(data)
    buf = buffers[label]
    while len(buf) >= PACKET_SIZE:
        packet = bytes(buf[:PACKET_SIZE])
        buf[:] = buf[PACKET_SIZE:]
        try:
            angle, distance = struct.unpack(PACKET_FORMAT, packet)
            if 0 <= angle < 360 and 0 < distance < 10000:
                latest[label] = (angle, distance)
                updated[label] = True
                uart_number = int(label[-1])
                uart_data[uart_number]["angle"] = angle
                uart_data[uart_number]["distance"] = distance
                for i in range(4):
                    if uart_data[i+1]["angle"] is not None:
                        angles[i] = uart_data[i+1]["angle"]
                    if uart_data[i+1]["distance"] is not None:
                        distances[i] = uart_data[i+1]["distance"]
        except Exception:
            pass  # skip malformed packets

def print_cycle():
    """Print one synchronized set in UART1→4 order with timestamp."""
    now = time.monotonic() - start_time
    print(
        f"[{now:7.3f}s] "
        f"[UART1] {latest['UART1'][0]:6.2f}° {latest['UART1'][1]:4d} mm | "
        f"[UART2] {latest['UART2'][0]:6.2f}° {latest['UART2'][1]:4d} mm | "
        f"[UART3] {latest['UART3'][0]:6.2f}° {latest['UART3'][1]:4d} mm | "
        f"[UART4] {latest['UART4'][0]:6.2f}° {latest['UART4'][1]:4d} mm"
    )
    led.value = False
    time.sleep(0.01)
    led.value = True

def hypot2(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5

def _solve(sensor_positions, angles_rad):
    normals = []
    bs = []
    for (x0, y0), th in zip(sensor_positions, angles_rad):
        nx = -math.sin(th)
        ny =  math.cos(th)
        b = nx * x0 + ny * y0
        normals.append((nx, ny))
        bs.append(b)

    Sxx = Sxy = Syx = Syy = 0.0
    Sxb = Syb = 0.0
    for (nx, ny), b in zip(normals, bs):
        Sxx += nx * nx
        Sxy += nx * ny
        Syx += ny * nx
        Syy += ny * ny
        Sxb += nx * b
        Syb += ny * b

    det = Sxx * Syy - Sxy * Syx
    if abs(det) < 1e-9:
        return (0.0, 0.0), 1e9

    x = ( Syy * Sxb - Sxy * Syb) / det
    y = (-Syx * Sxb + Sxx * Syb) / det

    rss = 0.0
    for (nx, ny), b in zip(normals, bs):
        r = nx * x + ny * y - b
        rss += r * r

    return (x, y), rss

def minimize(func, x0, step=10.0, tol=1e-4, max_iter=200, **kwargs):
    x, y = float(x0[0]), float(x0[1])
    f = func((x, y))

    for _ in range(max_iter):
        improved = False
        # Try small moves in ±x, ±y
        for dx, dy in ((step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)):
            x2 = x + dx
            y2 = y + dy
            f2 = func((x2, y2))
            if f2 < f:
                x, y, f = x2, y2, f2
                improved = True
                break
        if not improved:
            step *= 0.5
            if step < tol:
                break

    class Result:
        pass
    res = Result()
    res.x = (x, y)
    res.fun = f
    return res

def trilaterate_least_squares(sensor_positions, distances): 
    def objective(point): 
        x, y = point 
        error = 0 
        for (sx, sy), distance in zip(sensor_positions, distances): 
            calculated_dist = np.sqrt((x - sx) ** 2 + (y - sy) ** 2) 
            error += (calculated_dist - distance) ** 2 
        return error 
    
    initial_guess = np.mean(np.array(sensor_positions), axis=0)
    initial_guess = (float(initial_guess[0]), float(initial_guess[1]))
    # Optimize to find the point that best fits all distance measurements 
    result = minimize(objective, initial_guess, method="Nelder-Mead") 
    return (result.x[0], result.x[1])

def robust_triangulation(sensor_positions, angles):
    # Hypothesis A: 0° along +x, CCW
    angA = [math.radians(a) for a in angles]
    xA, rssA = _solve(sensor_positions, angA)
    print(f"Triangulation Hyp A: Pos=({xA[0]:.3f}, {xA[1]:.3f}), RSS={rssA:.3f}")

    # Hypothesis B: 0° along +y, CCW  ≡ A - 90°
    angB = [math.radians(a - 90.0) for a in angles]
    xB, rssB = _solve(sensor_positions, angB)
    print(f"Triangulation Hyp B: Pos=({xB[0]:.3f}, {xB[1]:.3f}), RSS={rssB:.3f}")

    return (-1)*xA if rssA <= rssB else xB


def estimate_from_individual_projections(sensor_readings, sensor_positions):
    """
    Estimate position from sensor readings where angles are in degrees.
    """
    projections = []
    for i in range(4):
        dist, angle_degrees = sensor_readings[i]
        sx, sy = sensor_positions[i]

        px = sx + (dist) * math.cos(math.radians(angle_degrees))
        py = sy + (dist) * math.sin(math.radians(angle_degrees))
        print(
            f"Sensor {i + 1}: Position=({sx:.2f}, {sy:.2f}), Distance={dist:.2f}, Angle={angle_degrees:.2f}° => Projection=({px:.2f}, {py:.2f})"
        )
        projections.append([px, py])

    return robust_least_squares(np.array(projections))


def robust_least_squares(points, max_iter=10, tol=1e-6):
    # Start with equal weights
    weights = np.ones(len(points))
    prev_center = np.mean(points, axis=0)

    for _ in range(max_iter):
        # Weighted centroid
        # Weighted centroid
        sx = sy = sw = 0.0
        for (x, y), w in zip(points, weights):
            sx += x * w
            sy += y * w
            sw += w

        # weighted_sum = (sx, sy)  we dont use this line anymore but I am being consissten for now
        new_center = np.array([sx / sw, sy / sw])
        # Compute residuals (Euclidean distances to new center)
        residuals = np.linalg.norm(points - new_center, axis=1)

        # Update weights: inverse of residual, clipped to avoid division by zero
        weights = 1 / np.maximum(residuals, 1e-3)

        # Check convergence
        if np.linalg.norm(new_center - prev_center) < tol:
            break
        prev_center = new_center

    return new_center

def main(distancesarray, anglesarray):
    d1 = []
    d2 = []
    d3 = []

    readings = [
        [
            (distancesarray[j] + RADIUS_MM) * MM2IN,
            (360 - anglesarray[j] + OFFSETS_DEG[j]) % 360,
        ]
        for j in range(4)
    ]

    distances = distancesarray
    angles = anglesarray
    p1 = estimate_from_individual_projections(readings, SENSOR_POS)
    p2 = trilaterate_least_squares(SENSOR_POS, distances)
    p3 = robust_triangulation(SENSOR_POS, angles)

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
                f"{label:<40} Avg = {np.mean(arr):.3f} in, Std Dev = {np.stdev(arr):.3f} in" #swapped statistics for np
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


# ==== End of Functions ====
 
# === Buffers and state ===
buffers = {name: bytearray() for name in ["UART1","UART2","UART3","UART4"]}
uarts = {
    "UART1": uart1,
    "UART2": uart2,
    "UART3": uart3,
    "UART4": uart4,
}

uart_data = {
    1: {"angle": None, "distance": None},
    2: {"angle": None, "distance": None},
    3: {"angle": None, "distance": None},
    4: {"angle": None, "distance": None},
}

latest = {name: None for name in uarts}      # (angle, distance)
updated = {name: False for name in uarts}    # whether new data arrived this cycle
last_cycle = time.monotonic()
start_time = last_cycle

CYCLE_PERIOD = 0.10  # 10 Hz output (seconds per group)

while True:
    for label, uart in uarts.items():
        read_uart(uart, label)
    now = time.monotonic()
    all_ready = all(updated.values())
    if all_ready or (now - last_cycle) >= CYCLE_PERIOD:
        if all(v is not None for v in latest.values()):
            #print(angles)
            #print(distances)
            main(distances, angles)
        for k in updated:
            updated[k] = False
        last_cycle = now



