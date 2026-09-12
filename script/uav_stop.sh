#!/usr/bin/env bash
set -euo pipefail

SESSION="dac_sfc_compact"
DAC_WS="${HOME}/DAC_SFC_real"
DECOMP_SETUP="/home/amov/decomp_ws/devel/setup.bash"
PX4CTRL_NODE="/px4ctrl_0"

source /opt/ros/noetic/setup.bash
source "${DECOMP_SETUP}"
source "${DAC_WS}/devel/setup.bash"

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[uav_stop] tmux session '${SESSION}' was not found." >&2
  exit 1
fi

if ! rosservice info "${PX4CTRL_NODE}/set_parameters" >/dev/null 2>&1; then
  echo "[uav_stop] PX4Ctrl dynamic-reconfigure service is unavailable." >&2
  echo "[uav_stop] Refusing to stop flight processes." >&2
  exit 1
fi

echo "[uav_stop] Returning CMD_CTRL to AUTO_HOVER..."
rosrun dynamic_reconfigure dynparam set "${PX4CTRL_NODE}" \
  "{mode_bool: true, cmd_bool: false}"
sleep 1

echo "[uav_stop] Sending LAND command on /iris_0/takeoff_land..."
rostopic pub -1 /iris_0/takeoff_land \
  quadrotor_msgs/TakeoffLand \
  "{takeoff_land_cmd: 2}"

echo "[uav_stop] Waiting for landed_state=1 and armed=false..."
safe_to_stop=false
attempt=0
while [ "${attempt}" -lt 60 ]; do
  landed_state="$(
    timeout 2 rostopic echo -n1 /mavros/extended_state 2>/dev/null |
      awk '/landed_state:/ {print $2; exit}' || true
  )"
  armed_state="$(
    timeout 2 rostopic echo -n1 /mavros/state 2>/dev/null |
      awk '/armed:/ {print $2; exit}' || true
  )"

  if [ "${landed_state}" = "1" ] && [ "${armed_state}" = "False" ]; then
    safe_to_stop=true
    break
  fi

  attempt=$((attempt + 1))
  sleep 1
done

if [ "${safe_to_stop}" != "true" ]; then
  echo "[uav_stop] Landing/disarm was not confirmed." >&2
  echo "[uav_stop] Planner, PX4Ctrl, rosbag and VINS remain running." >&2
  exit 1
fi

echo "[uav_stop] Landing and disarm confirmed. Finalizing rosbag..."
tmux send-keys -t "${SESSION}:flight.2" C-c
sleep 3

echo "[uav_stop] Stopping planner and PX4Ctrl..."
tmux send-keys -t "${SESSION}:flight.0" C-c
tmux send-keys -t "${SESSION}:flight.1" C-c
sleep 2

echo "[uav_stop] Stopping VINS and its odometry monitor..."
tmux send-keys -t "${SESSION}:vins_goal.0" C-c
tmux send-keys -t "${SESSION}:vins_goal.1" C-c

echo "[uav_stop] Safe shutdown completed."
