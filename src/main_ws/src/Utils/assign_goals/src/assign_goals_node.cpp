#include <Eigen/Eigen>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <uav_utils/geometry_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

// ros::Subscriber trajs_sub_;
ros::Publisher goals_pub_, new_goals_arrow_pub_;
ros::Time last_publish_time_;
bool need_clear_;

int drone_id;
int   goal_repeat_num;
double goal_repeat_dt;

// struct Selected_t
// {
//   int drone_id;
//   Eigen::Vector3d p;
// };
// vector<Selected_t> drones_;

void displayArrowList(const vector<Eigen::Vector3d> &start, const vector<Eigen::Vector3d> &end, const double scale, const int id, const int32_t action)
{
  if (start.size() != end.size())
  {
    ROS_ERROR("start.size() != end.size(), return");
    return;
  }

  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "world";
  arrow.header.stamp = ros::Time::now();
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = action;

  // geometry_msgs::Point start, end;
  // arrow.points

  arrow.color.r = 0;
  arrow.color.g = 0;
  arrow.color.b = 0;
  arrow.color.a = 1.0;
  arrow.scale.x = scale;
  arrow.scale.y = 4 * scale;
  arrow.scale.z = 4 * scale;

  for (int i = 0; i < int(start.size()); i++)
  {
    geometry_msgs::Point st, ed;
    st.x = start[i](0);
    st.y = start[i](1);
    st.z = start[i](2);
    ed.x = end[i](0);
    ed.y = end[i](1);
    ed.z = end[i](2);
    arrow.points.clear();
    arrow.points.push_back(st);
    arrow.points.push_back(ed);
    arrow.id = i + id;

    array.markers.push_back(arrow);
  }

  new_goals_arrow_pub_.publish(array);
}

// void selected_drones_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//   static ros::Time last_select_time = ros::Time(0);
//   ros::Time t_now = ros::Time::now();
//   if ((t_now - last_select_time).toSec() > 2)
//   {
//     drones_.clear();
//   }
//   Selected_t drone;
//   drone.drone_id = atoi(msg->header.frame_id.substr(6, 10).c_str());
//   drone.p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
//   drones_.push_back(drone);

//   cout.precision(3);
//   cout << "received drone " << drone.drone_id << " at " << drone.p.transpose() << ", total:" << drones_.size() << endl;

//   last_select_time = t_now;
// }

void user_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  Eigen::Vector3d goal(msg->pose.position.x,
                       msg->pose.position.y,
                       msg->pose.position.z);

  std::vector<Eigen::Vector3d> starts(1), ends(1);
  starts[0] << goal(0), goal(1), 0.0;   
  ends[0]   << goal(0), goal(1), goal(2); 
  displayArrowList(starts, ends, 0.05, 0, visualization_msgs::Marker::ADD);

  last_publish_time_ = ros::Time::now();
  need_clear_ = true;

  quadrotor_msgs::GoalSet goal_msg;
  goal_msg.drone_id = drone_id;
  goal_msg.goal[0] = goal(0);
  goal_msg.goal[1] = goal(1);
  goal_msg.goal[2] = goal(2);

  int n = std::max(goal_repeat_num, 1);      
  for (int i = 0; i < n; ++i)
  {
    goals_pub_.publish(goal_msg);
    ROS_INFO("[assign_goals_node] send goal(%d/%d) to drone %d: (%.2f, %.2f, %.2f)",
             i + 1, n, drone_id, goal(0), goal(1), goal(2));

    if (i < n - 1 && goal_repeat_dt > 1e-6)
    {
      ros::Duration(goal_repeat_dt).sleep();
    }
  }
}


// void user_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//   Eigen::Vector3d center = Eigen::Vector3d::Zero();
//   for (size_t i = 0; i < drones_.size(); ++i)
//   {
//     center += drones_[i].p;
//   }
//   center /= drones_.size();

//   Eigen::Vector3d user_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//   Eigen::Vector3d movment = user_goal - center;

//   vector<Eigen::Vector3d> each_one_starts(drones_.size()), each_one_goals(drones_.size());
//   for (size_t i = 0; i < drones_.size(); ++i)
//   {
//     each_one_starts[i] = drones_[i].p;
//     each_one_goals[i] = drones_[i].p + movment;
//     cout.precision(3);
//     cout << "drone " << drones_[i].drone_id << ", start=" << drones_[i].p.transpose() << ", end=" << each_one_goals[i].transpose() << endl;
//   }

//   displayArrowList(each_one_starts, each_one_goals, 0.05, 0, visualization_msgs::Marker::ADD);
//   last_publish_time_ = ros::Time::now();
//   need_clear_ = true;

//   for (size_t i = 0; i < drones_.size(); ++i)
//   {
//     quadrotor_msgs::GoalSet goal_msg;
//     goal_msg.drone_id = drones_[i].drone_id;
//     goal_msg.goal[0] = each_one_goals[i](0);
//     goal_msg.goal[1] = each_one_goals[i](1);
//     goal_msg.goal[2] = each_one_goals[i](2);
//     goals_pub_.publish(goal_msg);
//     ros::Duration(0.01).sleep();
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "assign_goals");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  srand(floor(ros::Time::now().toSec() * 10));

  pnh.param("drone_id", drone_id, -1);
  if (drone_id < 0)
  {
    ROS_ERROR("[assign_goals_node] invalid ~drone_id, please set it in launch!");
    return -1;
  }
  ROS_INFO("[assign_goals_node] control drone_id = %d", drone_id);

  pnh.param("goal_repeat_num", goal_repeat_num, 10);      
  pnh.param("goal_repeat_dt",  goal_repeat_dt,  0.01);  
  ROS_INFO("[assign_goals_node] goal_repeat_num = %d, goal_repeat_dt = %.3f",
           goal_repeat_num, goal_repeat_dt);

  ros::Subscriber user_goal_sub =
      pnh.subscribe<geometry_msgs::PoseStamped>("goal_from_rviz", 10, user_goal_cb);

  goals_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_user2brig", 10);
  new_goals_arrow_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/new_goals_arrow", 10);

  ROS_INFO("[assign_goals_node] Start running.");
  while (ros::ok())
  {
    if (need_clear_ && (ros::Time::now() - last_publish_time_).toSec() > 2)
    {
      need_clear_ = false;
      std::vector<Eigen::Vector3d> blank(1);
      blank[0] = Eigen::Vector3d::Zero();
      displayArrowList(blank, blank, 0.05, 0, visualization_msgs::Marker::DELETEALL);
      cout << "DELETEALL Arrows." << endl;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return 0;
}


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "assign_goals");
//   ros::NodeHandle nh("~");

//   srand(floor(ros::Time::now().toSec() * 10));

//   ros::Subscriber selected_drones_sub = nh.subscribe<geometry_msgs::PoseStamped>("/rviz_selected_drones", 100, selected_drones_cb);
//   ros::Subscriber user_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, user_goal_cb);

//   goals_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal_user2brig", 10);
//   new_goals_arrow_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/new_goals_arrow", 10);

//   ROS_INFO("[assign_goals_node]Start running.");
//   while (ros::ok())
//   {
//     if (need_clear_ && (ros::Time::now() - last_publish_time_).toSec() > 2)
//     {
//       need_clear_ = false;
//       std::vector<Eigen::Vector3d> blank(1);
//       blank[0] = Eigen::Vector3d::Zero();
//       displayArrowList(blank, blank, 0.05, 0, visualization_msgs::Marker::DELETEALL);
//       cout << "DELETEALL Arrows." << endl;
//     }

//     ros::Duration(0.01).sleep();
//     ros::spinOnce();
//   }

//   return 0;
// }

