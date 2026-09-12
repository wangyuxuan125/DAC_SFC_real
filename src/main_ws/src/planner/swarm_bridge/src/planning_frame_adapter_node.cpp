#include <ros/ros.h>

#include <Eigen/Core>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <string>

namespace
{
double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

template <typename VectorLike>
void rotateZ(VectorLike &vector, const double cosine, const double sine)
{
  const double x = vector.x;
  const double y = vector.y;
  vector.x = cosine * x - sine * y;
  vector.y = sine * x + cosine * y;
}
}  // namespace

class PlanningFrameAdapter
{
public:
  explicit PlanningFrameAdapter(ros::NodeHandle &node)
      : node_(node)
  {
    node_.param("offset_x", offset_x_, 0.0);
    node_.param("offset_y", offset_y_, 0.0);
    node_.param("offset_z", offset_z_, 0.0);
    node_.param("yaw_offset", yaw_offset_, 0.0);
    node_.param("world_frame", world_frame_, std::string("world"));
    node_.param("child_frame", child_frame_, std::string("base_link"));

    cosine_ = std::cos(yaw_offset_);
    sine_ = std::sin(yaw_offset_);

    world_odom_pub_ =
        node_.advertise<nav_msgs::Odometry>("world_odom", 20);
    local_cmd_pub_ =
        node_.advertise<quadrotor_msgs::PositionCommand>("local_cmd", 50);

    local_odom_sub_ = node_.subscribe(
        "local_odom", 20, &PlanningFrameAdapter::localOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    world_cmd_sub_ = node_.subscribe(
        "world_cmd", 50, &PlanningFrameAdapter::worldCommandCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("[planning_frame_adapter] local -> " << world_frame_
                    << " offset=[" << offset_x_ << ", " << offset_y_ << ", "
                    << offset_z_ << "] yaw_offset=" << yaw_offset_);
  }

private:
  void rotateCovariance(boost::array<double, 36> &covariance,
                        const bool inverse) const
  {
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using RowMajorMatrix6d =
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;

    const Eigen::Map<const RowMajorMatrix6d> source_map(covariance.data());
    const Matrix6d source = source_map;
    const double sine = inverse ? -sine_ : sine_;

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = cosine_;
    rotation(0, 1) = -sine;
    rotation(1, 0) = sine;
    rotation(1, 1) = cosine_;

    Matrix6d transform = Matrix6d::Zero();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 3>(3, 3) = rotation;

    const Matrix6d result = transform * source * transform.transpose();
    Eigen::Map<RowMajorMatrix6d>(covariance.data()) = result;
  }

  void rotateQuaternionForward(geometry_msgs::Quaternion &orientation) const
  {
    const double half_yaw = 0.5 * yaw_offset_;
    const double yaw_z = std::sin(half_yaw);
    const double yaw_w = std::cos(half_yaw);

    const double x = yaw_w * orientation.x - yaw_z * orientation.y;
    const double y = yaw_w * orientation.y + yaw_z * orientation.x;
    const double z = yaw_w * orientation.z + yaw_z * orientation.w;
    const double w = yaw_w * orientation.w - yaw_z * orientation.z;
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);

    if (norm > 1.0e-12)
    {
      orientation.x = x / norm;
      orientation.y = y / norm;
      orientation.z = z / norm;
      orientation.w = w / norm;
    }
  }

  void localOdomCallback(const nav_msgs::OdometryConstPtr &message)
  {
    nav_msgs::Odometry world = *message;

    rotateZ(world.pose.pose.position, cosine_, sine_);
    world.pose.pose.position.x += offset_x_;
    world.pose.pose.position.y += offset_y_;
    world.pose.pose.position.z += offset_z_;
    rotateQuaternionForward(world.pose.pose.orientation);

    rotateZ(world.twist.twist.linear, cosine_, sine_);
    rotateZ(world.twist.twist.angular, cosine_, sine_);
    rotateCovariance(world.pose.covariance, false);
    rotateCovariance(world.twist.covariance, false);

    world.header.frame_id = world_frame_;
    world.child_frame_id = child_frame_;
    world_odom_pub_.publish(world);
  }

  void worldCommandCallback(
      const quadrotor_msgs::PositionCommandConstPtr &message)
  {
    quadrotor_msgs::PositionCommand local = *message;

    local.position.x -= offset_x_;
    local.position.y -= offset_y_;
    local.position.z -= offset_z_;
    rotateZ(local.position, cosine_, -sine_);
    rotateZ(local.velocity, cosine_, -sine_);
    rotateZ(local.acceleration, cosine_, -sine_);
    rotateZ(local.jerk, cosine_, -sine_);
    local.yaw = wrapAngle(local.yaw - yaw_offset_);
    local.header.frame_id = child_frame_;

    local_cmd_pub_.publish(local);
  }

  ros::NodeHandle node_;
  ros::Subscriber local_odom_sub_;
  ros::Subscriber world_cmd_sub_;
  ros::Publisher world_odom_pub_;
  ros::Publisher local_cmd_pub_;

  double offset_x_;
  double offset_y_;
  double offset_z_;
  double yaw_offset_;
  double cosine_;
  double sine_;
  std::string world_frame_;
  std::string child_frame_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_frame_adapter");
  ros::NodeHandle private_node("~");
  PlanningFrameAdapter adapter(private_node);
  ros::spin();
  return 0;
}
