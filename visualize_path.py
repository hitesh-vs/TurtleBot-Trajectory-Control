import pandas as pd
import matplotlib.pyplot as plt

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

# ---------------- Plot ----------------
plt.figure(figsize=(8, 8))

plt.plot(
    wp_x, wp_y,
    'g--',
    linewidth=2,
    label="Desired trajectory (waypoints)"
)

plt.plot(
    odom_x, odom_y,
    'r',
    linewidth=2,
    label="Actual trajectory (odometry)"
)

# Start & end markers
plt.scatter(wp_x.iloc[0], wp_y.iloc[0], c='green', marker='o', s=80, label="Start")
plt.scatter(wp_x.iloc[-1], wp_y.iloc[-1], c='green', marker='x', s=80, label="Goal")

plt.axis("equal")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Trajectory Tracking: Desired vs Actual")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.show()
