#roslaunch px4 fast_racing.launch & sleep 20;
roslaunch ego_planner multi_run_in_gazebo.launch & sleep 10;
roslaunch px4ctrl multi_node.launch & sleep 10;
rosrun rqt_reconfigure rqt_reconfigure & sleep 10;
roslaunch ego_planner rviz.launch
