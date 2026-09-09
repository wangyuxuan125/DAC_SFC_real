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
