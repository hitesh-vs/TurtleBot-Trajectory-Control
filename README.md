# Path Smoothing and Trajectory Following for a Differential Drive Robot

This project implements **path smoothing**, **trajectory generation**, and **trajectory tracking control** for a differential drive robot using **ROS 2** and **Gazebo**.

## Features

- Waypoint-based path input (CSV)
- Path smoothing and trajectory generation
- Pure Pursuit Controller for trajectory tracking
- Gazebo-based simulation
- Odometry logging for performance evaluation
- Trajectory visualization and comparison

## Prerequisites

Ensure the following dependencies are installed:

- **ROS 2** (compatible distribution for your OS)
- **Gazebo**
- **Python 3**

## Repository Setup

### Clone the repository
```bash
git clone https://github.com/hitesh-vs/TurtleBot-Trajectory-Control.git
cd TurtleBot-Trajectory-Control
```

### Build the workspace
```bash
colcon build
```

Upon successful build, all four ROS 2 packages should compile.

### Source the workspace
```bash
source install/setup.bash
```
## Running the Simulation

### 1. Define Waypoints

Enter the desired waypoints in CSV format inside:
```bash
waypoint.csv
```

Example format:
```bash
x,y
0.0,0.0
1.0,0.5
2.0,1.0
```

### 2. Launch Gazebo Environment

Open a new terminal and run:
```bash
./scripts/setup_simulation.sh
```

This launches the Gazebo environment with the differential drive robot.

Click Play in the Gazebo GUI to start the simulation.

### 3. (Optional) Log Odometry Data

To record the robot’s actual trajectory for later analysis, run in a new terminal:
```bash
ros2 run odom_logger odom_logger_node
```

This will save odometry data to a CSV file.

### 4. Run Planning and Control Stack

In another terminal, run:
```bash
./scripts/run_simulation.sh
```

This starts:

Path smoothing

Trajectory generation

Pure Pursuit Controller

### Visualizing Controller Performance

After the controller finishes and odometry data has been logged:
```bash
python3 visualize_path.py
```

This script compares:

Desired trajectory (waypoints / smoothed path)

Actual trajectory (odometry data)

## Notes

Ensure all scripts are executable:
```bash
chmod +x scripts/*.sh
```

Gazebo must be running before launching the control stack.
