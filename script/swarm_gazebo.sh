#!/bin/sh
set -eu

# Start PX4 SITL/Gazebo separately first:
#   roslaunch px4 multi_vehicle.launch
roslaunch ego_planner dac_sfc_three_drone_gazebo.launch "$@"
