#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

namespace pointcloud_slam
{
class TabletopSegmentation
{
public:
  using PointT = pcl::PointXYZRGB;
  using CloudT = pcl::PointCloud<PointT>;

  TabletopSegmentation(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  CloudT::Ptr voxelFilter(const CloudT::Ptr& cloud) const;
  CloudT::Ptr passThroughFilter(const CloudT::Ptr& cloud) const;

  bool segmentTable(
      const CloudT::Ptr& cloud,
      pcl::PointIndices::Ptr& table_inliers,
      pcl::ModelCoefficients::Ptr& table_coefficients) const;

  void extractObjects(
      const CloudT::Ptr& cloud,
      const pcl::PointIndices::Ptr& table_inliers,
      CloudT::Ptr& objects_cloud) const;

  std::vector<pcl::PointIndices> clusterObjects(const CloudT::Ptr& cloud) const;

  void publishClusters(
      const CloudT::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      const std_msgs::Header& header);

  void publishTablePlane(
      const CloudT::Ptr& cloud,
      const pcl::PointIndices::Ptr& table_inliers,
      const std_msgs::Header& header);

private:
  ros::Subscriber sub_cloud_;
  ros::Publisher pub_table_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_clusters_;
  ros::Publisher pub_markers_;

  double voxel_leaf_size_;

  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;

  double plane_distance_threshold_;
  int plane_max_iterations_;

  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
};
}  // namespace pointcloud_slam
