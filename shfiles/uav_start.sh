#!/usr/bin/env bash
set -e

SESSION=uav_real

# 确保 session 存在
tmux has-session -t "$SESSION" 2>/dev/null || { echo "tmux session $SESSION not found"; exit 1; }

# 依次在对应 pane 执行后续启动（这里假设 pane 编号与上一个脚本一致）
tmux send-keys -t "$SESSION:main.2" C-m   # ego_planner multi_run_in_exp.launch
sleep 2
tmux send-keys -t "$SESSION:main.3" C-m   # px4ctrl multi_node.launch
sleep 2
tmux send-keys -t "$SESSION:main.4" C-m   # rosbag record

