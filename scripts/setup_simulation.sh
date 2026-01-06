#!/usr/bin/env bash
set -e
echo "Starting Gazebo..."
gz sim urdf/diff_drive_robot.sdf &
GZ_PID=$!

echo "Starting ROS-GZ bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  --ros-args -r /model/vehicle_blue/odometry:=/odom &
BRIDGE_PID=$!

cleanup() {
  echo "Shutting down simulation..."
  kill -INT $BRIDGE_PID 2>/dev/null || true
  kill -INT $GZ_PID 2>/dev/null || true
  wait
}

trap cleanup SIGINT SIGTERM

wait
