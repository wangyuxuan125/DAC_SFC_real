source /opt/ros/noetic/setup.bash
source /home/wyx/DAC_SFC_real/devel/setup.bash

PX4_FIRMWARE_ROOT=/home/wyx/PX4_Firmware
PX4_SITL_BUILD=/home/wyx/PX4_Firmware/build/px4_sitl_default

source "$PX4_FIRMWARE_ROOT/Tools/setup_gazebo.bash" \
    "$PX4_FIRMWARE_ROOT" \
    "$PX4_SITL_BUILD"

export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:$PX4_FIRMWARE_ROOT:$PX4_FIRMWARE_ROOT/Tools/sitl_gazebo"
rospack profile

roslaunch px4 multi_vehicle.launch



cd /home/wyx/DAC_SFC_real
source devel/setup.bash
bash script/ego_gazebo.sh


rostopic pub -1 /goal_with_id quadrotor_msgs/GoalSet "{drone_id: 0, goal: [-10.0, -15.0, 1.0]}"


固定使用的完整启动命令
source /opt/ros/noetic/setup.bash
source /home/wyx/ICRA2027/DecompROS/devel/setup.bash
source ~/DAC_SFC_real/devel/setup.bash

mkdir -p /tmp/dac_sfc_deployment/runs

DAC_LOG=/tmp/dac_sfc_deployment/runs/console_$(date +%Y%m%d_%H%M%S).log
echo "$DAC_LOG" > /tmp/dac_sfc_deployment/latest_console_path.txt

roslaunch ego_planner single_run_in_gazebo.launch \
  drone_id:=0 \
  flight_type:=1 \
  dac_sfc_shadow_only:=false \
  dac_sfc_fallback_to_ego:=true \
  dac_sfc_log_directory:=/tmp/dac_sfc_deployment \
  planning_horizon:=4.0 \
  max_vel:=1.0 \
  max_acc:=2.0 2>&1 |
tee "$DAC_LOG"

启动时应出现：

[DAC-SFC] Session CSV: /tmp/dac_sfc_deployment/runs/dac_sfc_...csv

查看当前文件：

cat /tmp/dac_sfc_deployment/latest_csv_path.txt
cat /tmp/dac_sfc_deployment/latest_console_path.txt

每次运行后的固定分析命令

汇总脚本现在不传文件名就会自动分析最新 CSV：

cd ~/DAC_SFC_real

python3 src/main_ws/src/planner/dac_sfc/tools/summarize_deployment.py

打印本次失败：

DAC_CSV=$(cat /tmp/dac_sfc_deployment/latest_csv_path.txt)

awk -F, '
NR>1 && $5==0 {
  printf "run=%s failure=%s obstacles=%s faces=%s violation=%s total_ms=%s\n",
         $2,$9,$24,$26,$34,$23
}' "$DAC_CSV"

打印本次关键日志：

DAC_LOG=$(cat /tmp/dac_sfc_deployment/latest_console_path.txt)

grep -E \
'Active-Witness corridor rejected|active_voxel_support_gap|corridor_violation|Safety check failed' \
"$DAC_LOG" |
tail -n 100
































roslaunch px4 multi_vehicle.launch 
source devel/setup.bash 
sh script/swarm_gazebo.sh 

roslaunch px4 single_vehicle.launch 
source devel/setup.bash 
sh script/ego_gazebo.sh 

抓包验证通讯情况
sudo tcpdump -i wlan0 udp port 8081 -nn
rostopic echo /others_odom

地面机：
rostopic list | grep goal_from_rviz 
能看到 /drone_0/goal_from_rviz、/drone_1/goal_from_rviz；
rostopic list | grep goal_user2brig 
能看到 /goal_user2brig；
在 RViz 中点击 /drone_0/goal_from_rviz 后
地面机上：
rostopic echo /goal_user2brig
有消息，并且 drone_id =  ；
在某架无人机上：
rostopic echo /goal_with_id
能看到同样的目标，说明 bridge 通路畅通。

地图共享的坐标转换
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world drone_0_map

rosbag

rosbag record -O multi_drone.bag /iris_0/drone_0_ego_planner_node/optimal_list /iris_0/drone_0_ego_planner_node/init_list /iris_0/drone_0_odom_visualization/path /iris_0/drone_0_ego_planner_node/grid_map/occupancy_inflate /iris_0/drone_0_odom_visualization/robot /camera/color/image_raw

DAC-SFC 部署验证（默认 shadow 模式，不接管控制）见：

`docs/dac_sfc_deployment_validation.md`
