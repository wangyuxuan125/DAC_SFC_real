#!/bin/sh
set -eu

rostopic pub -1 /iris_0/takeoff_land \
  quadrotor_msgs/TakeoffLand \
  "{takeoff_land_cmd: 1}"
