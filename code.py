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

# import math

# def minimize(func, x0, method="Nelder-Mead",
#              maxiter=200, xatol=1e-8, fatol=1e-8):
#     # Convert x0 to float tuple
#     x = float(x0[0])
#     y = float(x0[1])

#     # Parameters (from Nelder & Mead)
#     alpha = 1.0
#     gamma = 2.0
#     rho   = 0.5
#     sigma = 0.5

#     # Initial simplex
#     # SciPy uses: x0, x0 + scale * ei (for each i)
#     scale = 0.05
#     x1 = (x + scale * (abs(x) + 1.0), y)
#     x2 = (x, y + scale * (abs(y) + 1.0))
#     simplex = [(x, y), x1, x2]
#     fs = [func(p) for p in simplex]

#     for iteration in range(maxiter):
#         # Order
#         simplex, fs = zip(*sorted(zip(simplex, fs), key=lambda t: t[1]))
#         simplex = list(simplex)
#         fs = list(fs)

#         best = simplex[0]
#         worst = simplex[-1]
#         second_worst = simplex[-2]
#         f_best = fs[0]
#         f_worst = fs[-1]
#         f_second = fs[-2]

#         # Check termination
#         # in SciPy: np.max(np.abs(simplex[1:] - best)) <= xatol
#         max_diff = 0.0
#         for p in simplex[1:]:
#             dx = abs(p[0] - best[0])
#             dy = abs(p[1] - best[1])
#             if dx > max_diff:
#                 max_diff = dx
#             if dy > max_diff:
#                 max_diff = dy
#         if max_diff <= xatol and (f_worst - f_best) <= fatol:
#             break

#         # Centroid excluding worst
#         cx = cy = 0.0
#         for p in simplex[:-1]:
#             cx += p[0]
#             cy += p[1]
#         cx /= 2.0
#         cy /= 2.0

#         # Reflection
#         xr = (cx + alpha * (cx - worst[0]), cy + alpha * (cy - worst[1]))
#         f_r = func(xr)

#         if f_r < f_best:
#             # Expansion
#             xe = (cx + gamma * (xr[0] - cx), cy + gamma * (xr[1] - cy))
#             f_e = func(xe)
#             if f_e < f_r:
#                 simplex[-1] = xe
#                 fs[-1] = f_e
#             else:
#                 simplex[-1] = xr
#                 fs[-1] = f_r

#         elif f_r < f_second:
#             simplex[-1] = xr
#             fs[-1] = f_r

#         else:
#             # Contraction
#             if f_r < f_worst:
#                 # outside contraction
#                 xc = (cx + rho * (xr[0] - cx), cy + rho * (xr[1] - cy))
#             else:
#                 # inside contraction
#                 xc = (cx + rho * (worst[0] - cx), cy + rho * (worst[1] - cy))
#             f_c = func(xc)
#             if f_c < f_worst:
#                 simplex[-1] = xc
#                 fs[-1] = f_c
#             else:
#                 # Shrink
#                 for i in range(1, len(simplex)):
#                     simplex[i] = (best[0] + sigma * (simplex[i][0] - best[0]),
#                                   best[1] + sigma * (simplex[i][1] - best[1]))
#                     fs[i] = func(simplex[i])

#     # Return a simple result object
#     class Result:
#         pass
#     res = Result()
#     res.x = simplex[0]
#     res.fun = fs[0]
#     return res

# def trilaterate_least_squares(sensor_positions, distances): 
#     def objective(point): 
#         x, y = point 
#         error = 0 
#         for (sx, sy), distance in zip(sensor_positions, distances): 
#             calculated_dist = np.sqrt((x - sx) ** 2 + (y - sy) ** 2) 
#             error += (calculated_dist - distance) ** 2 
#         return error 
    
#     initial_guess = np.mean(np.array(sensor_positions), axis=0)
#     initial_guess = (float(initial_guess[0]), float(initial_guess[1]))
#     # Optimize to find the point that best fits all distance measurements 
#     result = minimize(objective, initial_guess, method="Nelder-Mead") 
#     return (result.x[0], result.x[1])

def trilaterate_least_squares(sensor_positions, distances):
    """
    Closed-form multilateration least-squares using the same algebra
    as the Teensy doMultilateration() code.

    sensor_positions: list of (sx, sy)
    distances:        list of d_i (same units as sx, sy)

    Returns (x, y) estimate. If the normal equation matrix is singular,
    returns (0.0, 0.0).
    """
    distances = distances / 25.4
    n = len(sensor_positions)
    if n < 3:
        # Not enough sensors for 2D trilateration
        return (0.0, 0.0)

    # We’re building A (n×3) with rows [1, -2x_i, -2y_i]
    # and b (n×1) with entries d_i^2 - x_i^2 - y_i^2
    #
    # Then we compute:
    #   AtA = A^T A  (3×3)
    #   Atb = A^T b  (3×1)
    # and solve:
    #   (AtA) * P = Atb, where P = [c, X, Y]^T and (X, Y) is the position.
    #
    # We'll accumulate AtA and Atb directly in scalar form.

    # Components of symmetric 3×3 matrix AtA
    A00 = A01 = A02 = A11 = A12 = A22 = A20 = A21 = A22 = A30 = A31 = A32 = 0.0
    # Components of 3×1 vector Atb
    b0 = b1 = b2 = 0.0

    for (sx, sy), d in zip(sensor_positions, distances):
        d2 = d * d
        b_i = d2 - sx * sx - sy * sy

        r0 = 1.0
        r1 = -2.0 * sx
        r2 = -2.0 * sy

        # AtA = sum over i of (row_i^T * row_i)
        A00 = r0
        A00 += r0 * r0          # (0,0)
        A01 += r0 * r1          # (0,1)
        A02 += r0 * r2          # (0,2)
        A11 += r1 * r1          # (1,1)
        A12 += r1 * r2          # (1,2)
        A22 += r2 * r2          # (2,2)

        # Atb = sum over i of row_i^T * b_i
        b0 += r0 * b_i
        b1 += r1 * b_i
        b2 += r2 * b_i

    # Fill symmetric parts
    A10 = A01
    A20 = A02
    A21 = A12

    # Determinant of AtA (3×3) – matches your C++ code
    det = (
        A00 * (A11 * A22 - A12 * A21)
        - A01 * (A10 * A22 - A12 * A20)
        + A02 * (A10 * A21 - A11 * A20)
    )

    if abs(det) < 1e-6:
        # Singular / ill-conditioned geometry
        return (0.0, 0.0)

    # Manual inverse of 3×3 (same as Teensy code)
    inv00 =  (A11 * A22 - A12 * A21) / det
    inv01 = -(A01 * A22 - A02 * A21) / det
    inv02 =  (A01 * A12 - A02 * A11) / det

    inv10 = -(A10 * A22 - A12 * A20) / det
    inv11 =  (A00 * A22 - A02 * A20) / det
    inv12 = -(A00 * A12 - A02 * A10) / det

    inv20 =  (A10 * A21 - A11 * A20) / det
    inv21 = -(A00 * A21 - A01 * A20) / det
    inv22 =  (A00 * A11 - A01 * A10) / det

    # P = inv(AtA) * Atb, P = [c, X, Y]^T
    P0 = inv00 * b0 + inv01 * b1 + inv02 * b2
    P1 = inv10 * b0 + inv11 * b1 + inv12 * b2
    P2 = inv20 * b0 + inv21 * b1 + inv22 * b2

    # We only care about X and Y, like outX = P(1), outY = P(2)
    x = P1
    y = P2
    return (x, y)

def robust_triangulation(sensor_positions, angles):
    # Hypothesis A: 0° along +x, CCW
    angA = [math.radians(a) for a in angles]
    xA, rssA = _solve(sensor_positions, angA)
    print(f"Triangulation Hyp A: Pos=({xA[0]:.3f}, {xA[1]:.3f}), RSS={rssA:.3f}")

    # Hypothesis B: 0° along +y, CCW  ≡ A - 90°
    angB = [math.radians(a - 90.0) for a in angles]
    xB, rssB = _solve(sensor_positions, angB)
    print(f"Triangulation Hyp B: Pos=({xB[0]:.3f}, {xB[1]:.3f}), RSS={rssB:.3f}")

    return (-xA[0], -xA[1]) if rssA <= rssB else (-xB[0], -xB[1])


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
        
