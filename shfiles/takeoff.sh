#!/bin/sh
set -eu

rostopic pub -1 /px4ctrl_2/takeoff_land \
  quadrotor_msgs/TakeoffLand \
  "{takeoff_land_cmd: 1}"
