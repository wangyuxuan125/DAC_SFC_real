#!/usr/bin/env bash
set -euo pipefail

SESSION="dac_sfc_real"
VINS_WS="${HOME}/vins"
DAC_WS="${HOME}/DAC_SFC_real"
DECOMP_SETUP="/home/amov/decomp_ws/devel/setup.bash"

BAG_DIR="${DAC_WS}/bags"
mkdir -p "${BAG_DIR}"
BAG_PREFIX="${BAG_DIR}/dac_sfc_$(date +%Y%m%d_%H%M%S)"

for required_path in \
  "${VINS_WS}/devel/setup.bash" \
  "${VINS_WS}/vins.sh" \
  "/opt/ros/noetic/setup.bash" \
  "${DECOMP_SETUP}" \
  "${DAC_WS}/devel/setup.bash"
do
  if [[ ! -e "${required_path}" ]]; then
    echo "Missing required path: ${required_path}" >&2
    exit 1
  fi
done

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed or is not in PATH." >&2
  exit 1
fi

tmux has-session -t "${SESSION}" 2>/dev/null && tmux kill-session -t "${SESSION}"

# Window 0: start VINS and monitor propagated odometry automatically.
tmux new-session -d -s "${SESSION}" -n vins_check
tmux set-option -t "${SESSION}" pane-border-status top
tmux set-option -t "${SESSION}" pane-border-format '#{pane_index}: #{pane_title}'
tmux split-window -h -t "${SESSION}:vins_check.0"
tmux select-pane -t "${SESSION}:vins_check.0" -T vins
tmux select-pane -t "${SESSION}:vins_check.1" -T vins_odom
tmux send-keys -t "${SESSION}:vins_check.0" \
  "cd '${VINS_WS}'; source devel/setup.bash; sh vins.sh" C-m
tmux send-keys -t "${SESSION}:vins_check.1" \
  "cd '${VINS_WS}'; source devel/setup.bash; until rostopic list >/dev/null 2>&1; do sleep 1; done; rostopic echo /vins_fusion/imu_propagate" C-m
tmux select-layout -t "${SESSION}:vins_check" even-horizontal

# Window 1: planner, PX4Ctrl and rosbag are independent panes.
# Commands are pre-filled and must be started manually with Enter.
tmux new-window -t "${SESSION}" -n core
tmux split-window -h -t "${SESSION}:core.0"
tmux split-window -v -t "${SESSION}:core.0"
tmux select-layout -t "${SESSION}:core" tiled
tmux select-pane -t "${SESSION}:core.0" -T planner
tmux select-pane -t "${SESSION}:core.1" -T px4ctrl
tmux select-pane -t "${SESSION}:core.2" -T rosbag

tmux send-keys -t "${SESSION}:core.0" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; roslaunch ego_planner single_run_in_exp.launch start_px4ctrl:=false dac_sfc_visualization_enabled:=true start_odom_visualization:=true dac_sfc_log_enabled:=false"

tmux send-keys -t "${SESSION}:core.1" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; roslaunch px4ctrl multi_node.launch"

tmux send-keys -t "${SESSION}:core.2" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; rosbag record -O '${BAG_PREFIX}' /vins_fusion/imu_propagate /drone_0_planning/trajectory /iris_0/position_cmd /goal_with_id /iris_0/takeoff_land /mavros/state /mavros/extended_state /drone_0_odom_visualization/robot /drone_0_odom_visualization/path /drone_0_ego_planner_node/dac_sfc/polyhedron_array /drone_0_ego_planner_node/a_star_list /drone_0_ego_planner_node/init_list /drone_0_ego_planner_node/optimal_list /drone_0_ego_planner_node/global_list /drone_0_ego_planner_node/failed_list /drone_0_ego_planner_node/grid_map/occupancy_inflate /tf /tf_static"

# Window 2: flight operations. State monitoring starts automatically; the
# takeoff, goal and landing commands are pre-filled for deliberate execution.
tmux new-window -t "${SESSION}" -n flight_ops
tmux split-window -h -t "${SESSION}:flight_ops.0"
tmux split-window -v -t "${SESSION}:flight_ops.0"
tmux split-window -v -t "${SESSION}:flight_ops.1"
tmux select-layout -t "${SESSION}:flight_ops" tiled
tmux select-pane -t "${SESSION}:flight_ops.0" -T mavros_state
tmux select-pane -t "${SESSION}:flight_ops.1" -T takeoff
tmux select-pane -t "${SESSION}:flight_ops.2" -T goal
tmux select-pane -t "${SESSION}:flight_ops.3" -T land

tmux send-keys -t "${SESSION}:flight_ops.0" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; until rostopic list >/dev/null 2>&1; do sleep 1; done; rostopic echo /mavros/state" C-m

tmux send-keys -t "${SESSION}:flight_ops.1" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; sh shfiles/takeoff.sh"

tmux send-keys -t "${SESSION}:flight_ops.2" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; rostopic pub -1 /goal_with_id quadrotor_msgs/GoalSet '{drone_id: 0, goal: [1.0, 0.0, 1.0]}'"

tmux send-keys -t "${SESSION}:flight_ops.3" \
  "cd '${DAC_WS}'; source /opt/ros/noetic/setup.bash; source '${DECOMP_SETUP}'; source '${DAC_WS}/devel/setup.bash'; sh shfiles/land.sh"

tmux select-window -t "${SESSION}:vins_check"
tmux attach-session -t "${SESSION}"
