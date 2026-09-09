#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class GlobalMapAssembler
{
public:
  GlobalMapAssembler(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    // 参数
    pnh.param("resolution", resolution_, 0.3);      // 栅格分辨率 [m]
    pnh.param("width",      width_,      400);      // 栅格宽度（格数）
    pnh.param("height",     height_,     400);      // 栅格高度（格数）
    pnh.param("origin_x",   origin_x_,  -60.0);     // 地图原点 (world 坐标)
    pnh.param("origin_y",   origin_y_,  -60.0);     // 地图原点 (world 坐标)
    pnh.param("z_min",      z_min_,     -1.0);      // 只考虑 [z_min, z_max] 的点
    pnh.param("z_max",      z_max_,      5.0);

    pnh.param<std::string>("frame_id", frame_id_, std::string("world"));

    // 初始化 OccupancyGrid
    grid_.info.resolution = resolution_;
    grid_.info.width      = width_;
    grid_.info.height     = height_;
    grid_.info.origin.position.x = origin_x_;
    grid_.info.origin.position.y = origin_y_;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    grid_.header.frame_id = frame_id_;
    grid_.data.assign(width_ * height_, -1);  // -1 = unknown

    // 订阅来自 bridge 的多机地图点云
    sub_ = nh.subscribe("/others_map_cloud", 10,
                        &GlobalMapAssembler::cloudCallback, this);
    // 发布总地图
    pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/swarm/global_occupancy", 1, true);

    ROS_INFO("[global_map_assembler] resolution=%.2f, size=%dx%d, origin=(%.1f, %.1f)",
             resolution_, width_, height_, origin_x_, origin_y_);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 把所有点“打”到栅格上
    for (const auto& pt : cloud.points)
    {
      if (pt.z < z_min_ || pt.z > z_max_) continue;

      int ix = static_cast<int>(std::floor((pt.x - origin_x_) / resolution_));
      int iy = static_cast<int>(std::floor((pt.y - origin_y_) / resolution_));

      if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_) continue;

      int idx = iy * width_ + ix;
      grid_.data[idx] = 100; // 占据
    }

    grid_.header.stamp = ros::Time::now();
    pub_.publish(grid_);
  }

private:
  ros::Subscriber sub_;
  ros::Publisher  pub_;

  nav_msgs::OccupancyGrid grid_;

  double resolution_;
  int    width_, height_;
  double origin_x_, origin_y_;
  double z_min_, z_max_;
  std::string frame_id_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_map_assembler_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  GlobalMapAssembler node(nh, pnh);

  ros::spin();
  return 0;
}
