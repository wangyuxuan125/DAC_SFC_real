#!/bin/sh
set -eu

for id in 0 1 2
do
  rostopic pub -1 "/iris_$id/takeoff_land" \
    quadrotor_msgs/TakeoffLand \
    "{takeoff_land_cmd: 2}"
done
