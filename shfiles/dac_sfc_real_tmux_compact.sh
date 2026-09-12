#!/bin/sh
set -eu

SESSION="dac_sfc_compact"
VINS_WS="${HOME}/vins"
DAC_WS="${HOME}/DAC_SFC_real"
DECOMP_SETUP="/home/amov/decomp_ws/devel/setup.bash"
TMUX_WIDTH="${DAC_TMUX_WIDTH:-180}"
TMUX_HEIGHT="${DAC_TMUX_HEIGHT:-54}"

BAG_DIR="${DAC_WS}/bags"
mkdir -p "${BAG_DIR}"
BAG_PREFIX="${BAG_DIR}/dac_sfc_$(date +%Y%m%d_%H%M%S)"

for required_path in \
  "${VINS_WS}/devel/setup.bash" \
  "${VINS_WS}/vins.sh" \
  "/opt/ros/noetic/setup.bash" \
  "${DECOMP_SETUP}" \
  "${DAC_WS}/devel/setup.bash" \
  "${DAC_WS}/script/uav_start.sh" \
  "${DAC_WS}/script/uav_stop.sh"
do
  if [ ! -e "${required_path}" ]; then
    echo "Missing required path: ${required_path}" >&2
    exit 1
  fi
done

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed or is not in PATH." >&2
  exit 1
fi

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "tmux session '${SESSION}' already exists; opening it."
  if [ -n "${TMUX:-}" ]; then
    exec tmux switch-client -t "${SESSION}"
  else
    exec tmux attach-session -t "${SESSION}"
  fi
fi

# Window 0: VINS odometry check uses half of the window. VINS and Goal share
# the other half. VINS and the check start automatically; Goal is pre-filled.
tmux new-session -d -x "${TMUX_WIDTH}" -y "${TMUX_HEIGHT}" \
  -s "${SESSION}" -n vins_goal

cleanup_incomplete_session()
{
  status=$?
  trap - 0
  if [ "${status}" -ne 0 ]; then
    echo "tmux layout creation failed; removing incomplete session '${SESSION}'." >&2
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
  fi
  exit "${status}"
}
trap cleanup_incomplete_session 0

tmux set-option -t "${SESSION}" pane-border-status top
tmux set-option -t "${SESSION}" pane-border-format '#{pane_index}: #{pane_title}'
tmux split-window -h -p 50 -t "${SESSION}:vins_goal.0"
tmux split-window -v -t "${SESSION}:vins_goal.1"

tmux select-pane -t "${SESSION}:vins_goal.0" -T vins_odom
tmux select-pane -t "${SESSION}:vins_goal.1" -T vins
tmux select-pane -t "${SESSION}:vins_goal.2" -T goal

tmux send-keys -t "${SESSION}:vins_goal.1" \
  "cd '${VINS_WS}'; source devel/setup.bash; sh vins.sh" C-m

tmux send-keys -t "${SESSION}:vins_goal.0" \
  "cd '${VINS_WS}'; source devel/setup.bash; until rostopic list >/dev/null 2>&1; do sleep 1; done; rostopic echo /vins_fusion/imu_propagate" C-m

tmux send-keys -t "${SESSION}:vins_goal.2" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; rostopic pub -1 /goal_with_id quadrotor_msgs/GoalSet '{drone_id: 0, goal: [1.0, 0.0, 1.0]}'"

# Window 1: all flight-system commands are independent and pre-filled.
tmux new-window -t "${SESSION}" -n flight
tmux split-window -h -t "${SESSION}:flight.0"
tmux split-window -v -t "${SESSION}:flight.0"
tmux split-window -v -t "${SESSION}:flight.1"
tmux split-window -v -t "${SESSION}:flight.2"
tmux split-window -v -t "${SESSION}:flight.3"
tmux select-layout -t "${SESSION}:flight" tiled

tmux select-pane -t "${SESSION}:flight.0" -T planner
tmux select-pane -t "${SESSION}:flight.1" -T px4ctrl
tmux select-pane -t "${SESSION}:flight.2" -T rosbag
tmux select-pane -t "${SESSION}:flight.3" -T takeoff
tmux select-pane -t "${SESSION}:flight.4" -T uav_start
tmux select-pane -t "${SESSION}:flight.5" -T uav_stop

tmux send-keys -t "${SESSION}:flight.0" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; roslaunch ego_planner single_run_in_exp.launch start_px4ctrl:=false dac_sfc_visualization_enabled:=true start_odom_visualization:=true dac_sfc_log_enabled:=false"

tmux send-keys -t "${SESSION}:flight.1" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; roslaunch px4ctrl multi_node.launch"

tmux send-keys -t "${SESSION}:flight.2" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; rosbag record -O '${BAG_PREFIX}' /vins_fusion/imu_propagate /drone_0_planning/trajectory /iris_0/position_cmd /goal_with_id /iris_0/takeoff_land /mavros/state /mavros/extended_state /drone_0_odom_visualization/robot /drone_0_odom_visualization/path /drone_0_ego_planner_node/dac_sfc/polyhedron_array /drone_0_ego_planner_node/a_star_list /drone_0_ego_planner_node/init_list /drone_0_ego_planner_node/optimal_list /drone_0_ego_planner_node/global_list /drone_0_ego_planner_node/failed_list /drone_0_ego_planner_node/grid_map/occupancy_inflate /tf /tf_static"

tmux send-keys -t "${SESSION}:flight.3" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; sh shfiles/takeoff.sh"

tmux send-keys -t "${SESSION}:flight.4" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; ./script/uav_start.sh"

tmux send-keys -t "${SESSION}:flight.5" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; ./script/uav_stop.sh"

tmux select-window -t "${SESSION}:vins_goal"
trap - 0
tmux attach-session -t "${SESSION}"
