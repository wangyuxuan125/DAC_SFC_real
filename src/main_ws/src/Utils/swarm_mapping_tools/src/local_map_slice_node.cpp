#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class LocalMapSliceNode
{
public:
  LocalMapSliceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    // 从参数服务器读取参数（带默认值）
    pnh.param("max_radius",   max_radius_,   10.0);   // 截取半径 [m]
    pnh.param("voxel_size",   voxel_size_,   0.3);    // 体素格大小 [m]
    pnh.param("z_min",        z_min_,       -1.0);    // 最低高度
    pnh.param("z_max",        z_max_,        5.0);    // 最高高度
    pnh.param("use_odom_center", use_odom_center_, true); // 是否使用里程计做截取中心

    std::string odom_topic;
    pnh.param<std::string>("odom_topic", odom_topic, std::string("/vins_fusion/imu_propagate"));

    // 订阅源点云：~input_cloud
    cloud_sub_ = pnh.subscribe("input_cloud", 1, &LocalMapSliceNode::cloudCallback, this);

    // 订阅里程计
    odom_sub_  = nh.subscribe(odom_topic, 1, &LocalMapSliceNode::odomCallback, this);

    // 发布局部点云：~local_swarm_cloud
    cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("local_swarm_cloud", 1);

    has_odom_ = false;

    ROS_INFO("[local_map_slice_node] max_radius = %.2f, voxel_size = %.2f, z_range = [%.2f, %.2f]",
             max_radius_, voxel_size_, z_min_, z_max_);
    ROS_INFO("[local_map_slice_node] subscribe cloud on ~input_cloud, odom on %s",
             odom_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    // 记录当前无人机位置，用于截取中心
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_z_ = msg->pose.pose.position.z;
    has_odom_ = true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (use_odom_center_ && !has_odom_)
    {
      ROS_WARN_THROTTLE(2.0, "[local_map_slice_node] no odom yet, skip cloud");
      return;
    }

    // 1. ROS 点云 -> PCL 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud_in);

    // 2. 先根据半径 / 高度做一次过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_filtered->reserve(cloud_in->size());

    double cx = use_odom_center_ ? odom_x_ : 0.0;
    double cy = use_odom_center_ ? odom_y_ : 0.0;
    double r2 = max_radius_ * max_radius_;

    for (const auto& pt : cloud_in->points)
    {
      double dx = pt.x - cx;
      double dy = pt.y - cy;
      double dist2 = dx * dx + dy * dy;

      if (dist2 > r2)  continue;
      if (pt.z < z_min_ || pt.z > z_max_) continue;

      cloud_filtered->points.push_back(pt);
    }

    cloud_filtered->width  = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = false;

    if (cloud_filtered->empty())
    {
      ROS_DEBUG("[local_map_slice_node] filtered cloud empty, skip publishing");
      return;
    }

    // 3. 体素滤波降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.filter(*cloud_down);

    // 4. PCL -> ROS 点云，发布
    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud_down, cloud_out);

    // 保持原 frame_id，时间戳用当前时间或者沿用原始
    cloud_out.header.frame_id = msg->header.frame_id;
    cloud_out.header.stamp    = ros::Time::now();

    cloud_pub_.publish(cloud_out);
  }

  // 成员变量
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  cloud_pub_;

  double max_radius_;
  double voxel_size_;
  double z_min_, z_max_;
  bool   use_odom_center_;
  bool   has_odom_;
  double odom_x_, odom_y_, odom_z_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_map_slice_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  LocalMapSliceNode node(nh, pnh);

  ros::spin();
  return 0;
}
