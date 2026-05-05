#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace pointcloud_slam
{
class PoissonReconstruction
{
public:
  using PointT = pcl::PointXYZRGB;
  using CloudT = pcl::PointCloud<PointT>;

  PoissonReconstruction(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

private:
  ros::Subscriber sub_clusters_;
  ros::Publisher pub_mesh_marker_;

  int normal_k_search_;
  int poisson_depth_;
  int poisson_solver_divide_;
  int poisson_iso_divide_;
  double poisson_point_weight_;
  bool save_mesh_;
  std::string mesh_output_dir_;
};
}  // namespace pointcloud_slam
