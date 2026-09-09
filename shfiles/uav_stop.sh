#!/usr/bin/env bash
set -euo pipefail

EGO_WS=~/Ego_Planner_v2_real

echo "[uav_stop] Sourcing ROS env..."
source "$EGO_WS/devel/setup.bash" || true

echo "[uav_stop] Stopping rosbag record..."
pkill -INT -f "rosbag record" || true
sleep 2
pkill -TERM -f "rosbag record" || true

echo "[uav_stop] Stopping planner/control roslaunch..."
pkill -INT -f "multi_run_in_exp.launch" || true
sleep 2

echo "[uav_stop] Sending LAND command..."
rostopic pub -1 /px4ctrl_1/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2" || true
sleep 20

echo "[uav_stop] Stopping VINS/MAVROS/RealSense if running..."
pkill -INT -f "multi_node.launch" || true
pkill -INT -f "fast_drone_250.launch" || true
pkill -INT -f "px4.launch" || true
pkill -INT -f "rs_camera.launch" || true

echo "[uav_stop] Done."
