#!/usr/bin/env bash
set -e

source install/setup.bash

echo "Starting planning stack..."

# Start planners in their own process groups
setsid ros2 run spline_path_planner spline_planner_node &
SPLINE_PID=$!

setsid ros2 run curvature_velocity_planner curvature_velocity_node &
VELOCITY_PID=$!

echo "Spline + Velocity planners running..."
sleep 3

echo "Stopping planners..."

# Kill entire process groups
kill -SIGINT -$SPLINE_PID -$VELOCITY_PID 2>/dev/null || true

# Wait cleanly
wait $SPLINE_PID 2>/dev/null || true
wait $VELOCITY_PID 2>/dev/null || true

# ---------------- Process 3 ----------------
echo "Starting Pure Pursuit controller (Ctrl+C to stop)"
exec ros2 run pure_pursuit_controller pure_pursuit_node
