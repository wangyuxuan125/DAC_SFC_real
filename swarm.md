roslaunch px4 multi_vehicle.launch

roslaunch ego_planner dac_sfc_three_drone_gazebo.launch   start_px4ctrl:=true   start_rviz:=false   start_swarm_bridge:=true   start_assign_goals:=false   dac_sfc_shadow_only:=true   dac_sfc_fallback_to_ego:=false   px4ctrl_hover_percentage:=0.5   px4ctrl_odom_velocity_in_body:=true


roslaunch ego_planner rviz.launch


source /opt/ros/noetic/setup.bash
source ~/DAC_SFC_real/devel/setup.bash

python3 - <<'PY'
import time
import rospy
from quadrotor_msgs.msg import GoalSet

goals = [
    (0, [-8.0,  -1.5, 1.0]),
    (1, [-8.0,  3.0, 1.0]),
    (2, [-8.0, 7.5, 1.0]),
]

rospy.init_node("publish_three_world_goals", anonymous=True)
pub = rospy.Publisher("/goal_with_id", GoalSet, queue_size=10)

deadline = time.monotonic() + 15.0
while not rospy.is_shutdown() and pub.get_num_connections() < 3:
    if time.monotonic() >= deadline:
        raise RuntimeError(
            "Only {} /goal_with_id subscribers".format(
                pub.get_num_connections()))
    time.sleep(0.1)

print("Connected subscribers:", pub.get_num_connections())

for drone_id, goal in goals:
    msg = GoalSet()
    msg.drone_id = drone_id
    msg.goal = goal
    pub.publish(msg)
    print("Published drone_{} goal={}".format(drone_id, goal))
    time.sleep(0.1)

time.sleep(2.0)
PY

rostopic pub -1 /iris_0/takeoff_land   quadrotor_msgs/TakeoffLand   "{takeoff_land_cmd: 1}"
rostopic pub -1 /iris_1/takeoff_land   quadrotor_msgs/TakeoffLand   "{takeoff_land_cmd: 1}"
rostopic pub -1 /iris_2/takeoff_land   quadrotor_msgs/TakeoffLand   "{takeoff_land_cmd: 1}"

