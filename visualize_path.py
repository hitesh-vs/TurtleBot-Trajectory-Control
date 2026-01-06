import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance

# ---------------- Load CSVs ----------------
odom_csv = "odom_log.csv"
waypoints_csv = "waypoints_time.csv"

odom = pd.read_csv(odom_csv)
wps = pd.read_csv(waypoints_csv)

# ---------------- Extract data ----------------
odom_x = odom["x"]
odom_y = odom["y"]

wp_x = wps["x"]
wp_y = wps["y"]

# ---------------- Calculate Tracking Errors ----------------
# For each odometry point, find the closest waypoint
errors = []

for i in range(len(odom)):
    odom_point = np.array([odom_x.iloc[i], odom_y.iloc[i]])
    
    # Calculate distance to all waypoints
    distances = []
    for j in range(len(wps)):
        wp_point = np.array([wp_x.iloc[j], wp_y.iloc[j]])
        dist = np.linalg.norm(odom_point - wp_point)
        distances.append(dist)
    
    # Get minimum distance (closest waypoint)
    min_distance = min(distances)
    errors.append(min_distance)

errors = np.array(errors)

# Calculate metrics
ate = np.mean(errors)  # Average Tracking Error
rmse = np.sqrt(np.mean(errors**2))  # Root Mean Square Error

print("=" * 50)
print("TRACKING ERROR METRICS")
print("=" * 50)
print(f"ATE (Average Tracking Error):  {ate:.4f} m")
print(f"RMSE (Root Mean Square Error): {rmse:.4f} m")
print(f"Max Error:                     {np.max(errors):.4f} m")
print(f"Min Error:                     {np.min(errors):.4f} m")
print("=" * 50)

# ---------------- Plot ----------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# Left plot: Trajectory comparison
ax1.plot(
    wp_x, wp_y,
    'g--',
    linewidth=2,
    label="Desired trajectory (waypoints)"
)

ax1.plot(
    odom_x, odom_y,
    'r',
    linewidth=2,
    label="Actual trajectory (odometry)"
)

# Start & end markers
ax1.scatter(wp_x.iloc[0], wp_y.iloc[0], c='green', marker='o', s=80, label="Start")
ax1.scatter(wp_x.iloc[-1], wp_y.iloc[-1], c='green', marker='x', s=80, label="Goal")

ax1.axis("equal")
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_title(f"Trajectory Tracking\nATE: {ate:.4f} m | RMSE: {rmse:.4f} m")
ax1.grid(True)
ax1.legend()

# Right plot: Error over time
ax2.plot(errors, 'b-', linewidth=2)
ax2.axhline(y=ate, color='r', linestyle='--', label=f'ATE: {ate:.4f} m')
ax2.axhline(y=rmse, color='orange', linestyle='--', label=f'RMSE: {rmse:.4f} m')
ax2.set_xlabel("Odometry Sample Index")
ax2.set_ylabel("Tracking Error [m]")
ax2.set_title("Tracking Error Over Time")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()