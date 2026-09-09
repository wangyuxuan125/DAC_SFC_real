#!/usr/bin/env bash
set -e

SESSION=uav_real
VINS_WS=~/vins
EGO_WS=~/Ego_Planner_v2_real

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux new-session -d -s "$SESSION" -n main

# 创建 8 个 pane
tmux split-window -h  -t "$SESSION:main"
tmux split-window -v  -t "$SESSION:main.0"
tmux split-window -v  -t "$SESSION:main.1"
tmux split-window -v  -t "$SESSION:main.1"
tmux split-window -v  -t "$SESSION:main.3"
tmux split-window -h  -t "$SESSION:main.3"
tmux split-window -v  -t "$SESSION:main.5"
tmux split-window -h  -t "$SESSION:main.5"
# 现在 panes 约为 0~6

# Pane 0：启动 VINS（阻塞运行没关系）
tmux send-keys -t "$SESSION:main.0" \
  "cd $VINS_WS; source devel/setup.bash; ./vins.sh" C-m
sleep 20
# Pane 1：监控 vins 输出（按你要求 rostopic echo）
# 注意：需要在能看到 ROS_MASTER 的环境下 source；通常 source 任一工作空间即可
tmux send-keys -t "$SESSION:main.1" \
  "cd $EGO_WS; source devel/setup.bash; rostopic echo /vins_fusion/imu_propagate" C-m

# 其余 pane：预填后续命令，但不执行（给你明确“未启动”）
tmux send-keys -t "$SESSION:main.2" \
  "cd $EGO_WS; source devel/setup.bash; roslaunch ego_planner multi_run_in_exp.launch"
tmux send-keys -t "$SESSION:main.3" \
  "cd $EGO_WS; source devel/setup.bash; roslaunch px4ctrl multi_node.launch"
tmux send-keys -t "$SESSION:main.4" \
  "source $EGO_WS/devel/setup.bash; rosbag record -O ~/bags/multi_drone_$(date +%Y%m%d_%H%M%S).bag \
/iris_0/drone_0_ego_planner_node/optimal_list \
/iris_0/drone_0_ego_planner_node/init_list \
/iris_0/drone_0_odom_visualization/path \
/iris_0/drone_0_ego_planner_node/grid_map/occupancy_inflate \
/iris_0/drone_0_odom_visualization/robot \
/camera/color/image_raw"
tmux send-keys -t "$SESSION:main.5" \
  "cd $EGO_WS; source devel/setup.bash; ./shfiles/takeoff.sh"
tmux send-keys -t "$SESSION:main.6" \
  "cd $EGO_WS; source devel/setup.bash; ./shfiles/uav_start.sh"
tmux send-keys -t "$SESSION:main.7" \
  "cd $EGO_WS; source devel/setup.bash; ./shfiles/uav_stop.sh"
tmux attach -t "$SESSION"
