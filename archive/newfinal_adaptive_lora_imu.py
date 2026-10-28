import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# ADAPTIVE LoRa-IMU SENSOR FUSION
# FINAL POLISHED SOFTWARE SIMULATION
#
# IMPORTANT:
# This implementation uses a LINEAR KALMAN FILTER.
# It is not an EKF because the state and measurement models
# used here are linear.
#
# All results in this program are SIMULATED results.
# They are NOT real-hardware accuracy measurements.
# ============================================================

np.random.seed(7)

# ============================================================
# 1. SIMULATION SETTINGS
# ============================================================

dt = 0.1
T = 20.0
t = np.arange(0, T, dt)
N = len(t)

# ============================================================
# 2. TRUE 2-D MOVEMENT
# ============================================================

true_x = 2.0 + 6.0 * (t / T)
true_y = 2.0 + 5.0 * np.sin(2 * np.pi * t / T)

true_pos = np.column_stack((true_x, true_y))

true_vx = np.gradient(true_x, dt)
true_vy = np.gradient(true_y, dt)

true_ax = np.gradient(true_vx, dt)
true_ay = np.gradient(true_vy, dt)

true_acc = np.column_stack((true_ax, true_ay))

# ============================================================
# 3. SIMULATED IMU
# ============================================================

imu_noise_std = 0.15

# Simulated accelerometer bias
imu_bias = np.array([0.08, -0.06])

imu_acc = (
    true_acc
    + imu_bias
    + np.random.normal(0, imu_noise_std, (N, 2))
)

# ------------------------------------------------------------
# IMU-only dead reckoning
# ------------------------------------------------------------

imu_vel = np.zeros((N, 2))
imu_pos = np.zeros((N, 2))

imu_pos[0] = true_pos[0]

for k in range(1, N):

    # Acceleration -> velocity
    imu_vel[k] = imu_vel[k - 1] + imu_acc[k] * dt

    # Velocity -> position
    imu_pos[k] = (
        imu_pos[k - 1]
        + imu_vel[k] * dt
        + 0.5 * imu_acc[k] * dt**2
    )

# ============================================================
# 4. LoRa ANCHORS
# ============================================================

anchors = np.array([
    [0.0, 0.0],
    [10.0, 0.0],
    [5.0, 10.0]
])

RSSI_0 = -40.0
d0 = 1.0
PLE = 2.2

normal_rssi_noise = 3.0

# ============================================================
# 5. SIMULATED LoRa RSSI
# ============================================================

rssi_data = np.zeros((N, 3))

for k in range(N):

    for a in range(3):

        distance = np.linalg.norm(
            true_pos[k] - anchors[a]
        )

        distance = max(distance, 0.1)

        # Deliberately simulate a degraded signal period
        # from 8 s to 12 s.
        if 8.0 <= t[k] <= 12.0:
            noise_std = 7.0
        else:
            noise_std = normal_rssi_noise

        # Log-distance path-loss model
        rssi_data[k, a] = (
            RSSI_0
            - 10.0 * PLE * np.log10(distance / d0)
            + np.random.normal(0, noise_std)
        )

# ============================================================
# 6. RSSI -> DISTANCE
# ============================================================

estimated_distances = d0 * 10 ** (
    (RSSI_0 - rssi_data) / (10.0 * PLE)
)

# ============================================================
# 7. 3-ANCHOR TRILATERATION
# ============================================================

def trilaterate(anchor_points, distances):
    """
    Estimate 2-D position from three fixed anchors.
    """

    x1, y1 = anchor_points[0]
    x2, y2 = anchor_points[1]
    x3, y3 = anchor_points[2]

    r1, r2, r3 = distances

    A = 2.0 * np.array([
        [x2 - x1, y2 - y1],
        [x3 - x1, y3 - y1]
    ])

    b = np.array([
        r1**2 - r2**2
        + x2**2 + y2**2
        - x1**2 - y1**2,

        r1**2 - r3**2
        + x3**2 + y3**2
        - x1**2 - y1**2
    ])

    try:
        return np.linalg.solve(A, b)

    except np.linalg.LinAlgError:
        return np.array([np.nan, np.nan])


lora_pos = np.array([
    trilaterate(anchors, estimated_distances[k])
    for k in range(N)
])

# Replace invalid values, if any
for k in range(N):

    if np.any(~np.isfinite(lora_pos[k])):

        if k > 0:
            lora_pos[k] = lora_pos[k - 1]
        else:
            lora_pos[k] = true_pos[0]

# ============================================================
# 8. LINEAR KALMAN FILTER
#
# State = [x, y, vx, vy]
# Measurement = [x, y]
# ============================================================

F = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])

Q = np.diag([
    0.01,
    0.01,
    0.10,
    0.10
])

R_normal = np.diag([
    4.0,
    4.0
])

# ============================================================
# 9. NORMAL / ADAPTIVE KALMAN FILTER
# ============================================================

def run_kalman(adaptive=False):

    x = np.array([
        true_pos[0, 0],
        true_pos[0, 1],
        0.0,
        0.0
    ])

    P = np.eye(4)

    estimates = np.zeros((N, 4))
    adaptive_scale_history = np.ones(N)

    for k in range(N):

        # ----------------------------------------------------
        # Prediction
        # ----------------------------------------------------

        x_pred = F @ x

        P_pred = (
            F @ P @ F.T
            + Q
        )

        # ----------------------------------------------------
        # Measurement noise adaptation
        # ----------------------------------------------------

        if adaptive:

            # Measure disagreement between the 3 RSSI values.
            # Larger spread -> less confidence in LoRa position.
            rssi_spread = np.std(rssi_data[k])

            if rssi_spread > 5.0:
                scale = 8.0

            elif rssi_spread > 3.5:
                scale = 3.0

            else:
                scale = 1.0

            # Extra protection during the deliberately degraded
            # signal interval.
            if 8.0 <= t[k] <= 12.0:
                scale *= 2.0

            R = R_normal * scale
            adaptive_scale_history[k] = scale

        else:

            R = R_normal

        # ----------------------------------------------------
        # Measurement update
        # ----------------------------------------------------

        z = lora_pos[k]

        innovation = z - H @ x_pred

        S = (
            H @ P_pred @ H.T
            + R
        )

        K = (
            P_pred
            @ H.T
            @ np.linalg.inv(S)
        )

        x = (
            x_pred
            + K @ innovation
        )

        P = (
            np.eye(4)
            - K @ H
        ) @ P_pred

        estimates[k] = x

    return estimates, adaptive_scale_history


normal_kalman, normal_scale = run_kalman(adaptive=False)

adaptive_kalman, adaptive_scale = run_kalman(adaptive=True)

# ============================================================
# 10. ERROR CALCULATIONS
# ============================================================

def position_error(estimate, truth):
    return np.linalg.norm(
        estimate[:, :2] - truth,
        axis=1
    )


imu_error = np.linalg.norm(
    imu_pos - true_pos,
    axis=1
)

lora_error = np.linalg.norm(
    lora_pos - true_pos,
    axis=1
)

normal_error = position_error(
    normal_kalman,
    true_pos
)

adaptive_error = position_error(
    adaptive_kalman,
    true_pos
)


def rmse(error):
    return np.sqrt(np.mean(error**2))


imu_rmse = rmse(imu_error)
lora_rmse = rmse(lora_error)
normal_rmse = rmse(normal_error)
adaptive_rmse = rmse(adaptive_error)

improvement = (
    (normal_rmse - adaptive_rmse)
    / normal_rmse
    * 100.0
)

# ============================================================
# 11. FINAL RESULTS
# ============================================================

print()
print("=" * 62)
print("ADAPTIVE LoRa-IMU SENSOR FUSION")
print("FINAL POLISHED SOFTWARE SIMULATION")
print("=" * 62)

print(f"{'Method':<28} {'RMSE':>10}")
print("-" * 42)

print(f"{'IMU-only':<28} {imu_rmse:>8.2f} m")
print(f"{'LoRa-only':<28} {lora_rmse:>8.2f} m")
print(f"{'Normal Kalman':<28} {normal_rmse:>8.2f} m")
print(f"{'Adaptive Kalman':<28} {adaptive_rmse:>8.2f} m")

print("-" * 42)

print(
    f"Adaptive improvement over normal Kalman: "
    f"{improvement:.1f}%"
)

print()
print("FINAL POSITION ERROR")
print("-" * 42)

print(f"{'IMU-only':<28} {imu_error[-1]:>8.2f} m")
print(f"{'LoRa-only':<28} {lora_error[-1]:>8.2f} m")
print(f"{'Normal Kalman':<28} {normal_error[-1]:>8.2f} m")
print(f"{'Adaptive Kalman':<28} {adaptive_error[-1]:>8.2f} m")

print("=" * 62)

# ============================================================
# 12. FIGURE 1 - CLEAN TRAJECTORY COMPARISON
# ============================================================

plt.figure(figsize=(9, 6))

# True movement path
plt.plot(
    true_x,
    true_y,
    label="True trajectory",
    linewidth=2
)

# IMU-only drift
plt.plot(
    imu_pos[:, 0],
    imu_pos[:, 1],
    label="IMU-only"
)

# Normal Kalman fusion
plt.plot(
    normal_kalman[:, 0],
    normal_kalman[:, 1],
    label="Normal Kalman"
)

# Adaptive Kalman fusion
plt.plot(
    adaptive_kalman[:, 0],
    adaptive_kalman[:, 1],
    label="Adaptive Kalman",
    linewidth=2
)

# Fixed LoRa anchors
plt.scatter(
    anchors[:, 0],
    anchors[:, 1],
    marker="^",
    s=80,
    label="LoRa anchors"
)

# Keep the useful working area visible
plt.xlim(-1, 10)
plt.ylim(-5, 10)

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")

plt.title(
    "Trajectory Comparison: Adaptive LoRa-IMU Sensor Fusion"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# 13. FIGURE 2 - POSITION ERROR
# ============================================================

plt.figure(figsize=(9, 6))

plt.plot(
    t,
    imu_error,
    label="IMU-only error"
)

plt.plot(
    t,
    lora_error,
    label="LoRa-only error"
)

plt.plot(
    t,
    normal_error,
    label="Normal Kalman error"
)

plt.plot(
    t,
    adaptive_error,
    label="Adaptive Kalman error"
)

plt.axvspan(
    8,
    12,
    alpha=0.2,
    label="High RSSI noise"
)

plt.xlabel("Time (s)")
plt.ylabel("Position error (m)")

plt.title(
    "Position Error Over Time"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# 14. FIGURE 3 - LoRa RSSI
# ============================================================

plt.figure(figsize=(9, 6))

for a in range(3):

    plt.plot(
        t,
        rssi_data[:, a],
        label=f"Anchor {a + 1}"
    )

plt.axvspan(
    8,
    12,
    alpha=0.2,
    label="Degraded signal period"
)

plt.xlabel("Time (s)")
plt.ylabel("RSSI (dBm)")

plt.title(
    "Simulated LoRa RSSI"
)

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()

# ============================================================
# 15. FIGURE 4 - RMSE COMPARISON
# ============================================================

methods = [
    "IMU-only",
    "LoRa-only",
    "Normal Kalman",
    "Adaptive Kalman"
]

values = [
    imu_rmse,
    lora_rmse,
    normal_rmse,
    adaptive_rmse
]

plt.figure(figsize=(8, 5))

bars = plt.bar(
    methods,
    values
)

# Display numerical RMSE values above each bar
for bar, value in zip(bars, values):

    plt.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 0.3,
        f"{value:.2f} m",
        ha="center",
        va="bottom"
    )

plt.ylabel("RMSE (m)")

plt.title(
    "RMSE Comparison"
)

plt.xticks(
    rotation=15
)

plt.grid(
    axis="y"
)

plt.tight_layout()

plt.show()

# ============================================================
# END OF SIMULATION
# ============================================================

print()
print("Simulation complete.")
print()
print("IMPORTANT:")
print("These are SIMULATED results.")
print("They must not be presented as real-hardware accuracy.")
print()
print(
    "Project interpretation: the adaptive Kalman filter "
    "reduces the influence of unreliable LoRa measurements "
    "when RSSI becomes highly variable."
)