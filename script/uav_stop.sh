#!/usr/bin/env bash
set -euo pipefail

SESSION="dac_sfc_compact"
DAC_WS="${HOME}/DAC_SFC_real"
DECOMP_SETUP="/home/amov/decomp_ws/devel/setup.bash"
PX4CTRL_NODE="/px4ctrl_2"
CMD_TOPIC="/iris_0/position_cmd"
TAKEOFF_LAND_TOPIC="/px4ctrl_2/takeoff_land"

source /opt/ros/noetic/setup.bash
source "${DECOMP_SETUP}"
source "${DAC_WS}/devel/setup.bash"

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[uav_stop] tmux session '${SESSION}' was not found." >&2
  exit 1
fi

if ! rosnode info "${PX4CTRL_NODE}" >/dev/null 2>&1; then
  echo "[uav_stop] PX4Ctrl node '${PX4CTRL_NODE}' is unavailable." >&2
  echo "[uav_stop] Refusing to stop flight processes." >&2
  exit 1
fi

# This flight-validated PX4Ctrl has no dynamic-reconfigure switch. Stop the
# trajectory source first; after cmd timeout it returns CMD_CTRL -> AUTO_HOVER.
echo "[uav_stop] Stopping planner/trajectory source..."
tmux send-keys -t "${SESSION}:flight.0" C-c
sleep 2

active_publishers="$(
  timeout 3 rostopic info "${CMD_TOPIC}" 2>/dev/null |
    awk '/^Publishers:/{capture=1; next} /^Subscribers:/{capture=0} capture && /^ \*/{print}' ||
    true
)"
if [ -n "${active_publishers}" ]; then
  echo "[uav_stop] A publisher is still active on ${CMD_TOPIC}:" >&2
  echo "${active_publishers}" >&2
  echo "[uav_stop] PX4Ctrl and VINS remain running; LAND was not sent." >&2
  exit 1
fi

echo "[uav_stop] Sending LAND command on ${TAKEOFF_LAND_TOPIC}..."
rostopic pub -1 "${TAKEOFF_LAND_TOPIC}" \
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
  echo "[uav_stop] PX4Ctrl, rosbag and VINS remain running." >&2
  exit 1
fi

echo "[uav_stop] Landing and disarm confirmed. Finalizing rosbag..."
tmux send-keys -t "${SESSION}:flight.2" C-c
sleep 3

echo "[uav_stop] Stopping PX4Ctrl..."
tmux send-keys -t "${SESSION}:flight.1" C-c
sleep 2

echo "[uav_stop] Stopping VINS and its odometry monitor..."
tmux send-keys -t "${SESSION}:vins_goal.0" C-c
tmux send-keys -t "${SESSION}:vins_goal.1" C-c

echo "[uav_stop] Safe shutdown completed."
