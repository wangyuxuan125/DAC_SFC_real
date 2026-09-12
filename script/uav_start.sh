#!/usr/bin/env bash
set -euo pipefail

SESSION="dac_sfc_compact"
WINDOW="${SESSION}:flight"

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "[uav_start] tmux session '${SESSION}' was not found." >&2
  echo "[uav_start] Run: sh shfiles/dac_sfc_real_tmux_compact.sh" >&2
  exit 1
fi

for pane in 0 1 2; do
  pane_command="$(tmux display-message -p -t "${WINDOW}.${pane}" '#{pane_current_command}')"
  case "${pane_command}" in
    bash|dash|sh|zsh)
      ;;
    *)
      echo "[uav_start] Refusing to reuse pane ${pane}; it is already running '${pane_command}'." >&2
      exit 1
      ;;
  esac
done

echo "[uav_start] Starting DAC-SFC planner..."
tmux send-keys -t "${WINDOW}.0" C-m
sleep 2

echo "[uav_start] Starting PX4Ctrl..."
tmux send-keys -t "${WINDOW}.1" C-m
sleep 2

echo "[uav_start] Starting rosbag recording..."
tmux send-keys -t "${WINDOW}.2" C-m

echo "[uav_start] Planner, PX4Ctrl and rosbag commands were started."
