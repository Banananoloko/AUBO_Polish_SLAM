#include "PointCLoud_SLAM/tabletop_segmentation.hpp"

#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pointcloud_slam
{
TabletopSegmentation::TabletopSegmentation(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param("voxel_leaf_size", voxel_leaf_size_, 0.005);

  pnh.param("passthrough/x_min", x_min_, 0.1);
  pnh.param("passthrough/x_max", x_max_, 1.2);
  pnh.param("passthrough/y_min", y_min_, -0.6);
  pnh.param("passthrough/y_max", y_max_, 0.6);
  pnh.param("passthrough/z_min", z_min_, 0.0);
  pnh.param("passthrough/z_max", z_max_, 1.5);

  pnh.param("plane_segmentation/distance_threshold", plane_distance_threshold_, 0.01);
  pnh.param("plane_segmentation/max_iterations", plane_max_iterations_, 1000);

  pnh.param("clustering/tolerance", cluster_tolerance_, 0.02);
  pnh.param("clustering/min_cluster_size", min_cluster_size_, 200);
  pnh.param("clustering/max_cluster_size", max_cluster_size_, 30000);

  sub_cloud_ = nh.subscribe("/input_cloud", 1, &TabletopSegmentation::cloudCallback, this);

  pub_table_   = nh.advertise<sensor_msgs::PointCloud2>("/tabletop/table_plane", 1);
  pub_objects_ = nh.advertise<sensor_msgs::PointCloud2>("/tabletop/objects_cloud", 1);
  pub_clusters_= nh.advertise<sensor_msgs::PointCloud2>("/tabletop/clusters", 1);
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/tabletop/cluster_markers", 1);

  ROS_INFO("TabletopSegmentation node initialized.");
}

void TabletopSegmentation::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  CloudT::Ptr cloud(new CloudT);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty())
  {
    ROS_WARN_THROTTLE(2.0, "Input cloud is empty.");
    return;
  }

  auto filtered = voxelFilter(cloud);
  filtered = passThroughFilter(filtered);

  if (filtered->empty())
  {
    ROS_WARN_THROTTLE(2.0, "Filtered cloud is empty.");
    return;
  }

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr table_coefficients(new pcl::ModelCoefficients);

  if (!segmentTable(filtered, table_inliers, table_coefficients))
  {
    ROS_WARN_THROTTLE(2.0, "Failed to segment tabletop plane.");
    return;
  }

  publishTablePlane(filtered, table_inliers, msg->header);

  CloudT::Ptr objects_cloud(new CloudT);
  extractObjects(filtered, table_inliers, objects_cloud);

  if (objects_cloud->empty())
  {
    ROS_WARN_THROTTLE(2.0, "No object points found after removing table plane.");
    return;
  }

  sensor_msgs::PointCloud2 objects_msg;
  pcl::toROSMsg(*objects_cloud, objects_msg);
  objects_msg.header = msg->header;
  pub_objects_.publish(objects_msg);

  auto cluster_indices = clusterObjects(objects_cloud);
  publishClusters(objects_cloud, cluster_indices, msg->header);

  ROS_INFO_THROTTLE(1.0, "Objects cloud size: %zu, clusters: %zu",
                    objects_cloud->size(), cluster_indices.size());
}

TabletopSegmentation::CloudT::Ptr
TabletopSegmentation::voxelFilter(const CloudT::Ptr& cloud) const
{
  CloudT::Ptr out(new CloudT);
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  vg.filter(*out);
  return out;
}

TabletopSegmentation::CloudT::Ptr
TabletopSegmentation::passThroughFilter(const CloudT::Ptr& cloud) const
{
  CloudT::Ptr temp(new CloudT);
  CloudT::Ptr out(new CloudT);

  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min_, x_max_);
  pass.filter(*temp);

  pass.setInputCloud(temp);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min_, y_max_);
  pass.filter(*out);

  temp.reset(new CloudT);
  pass.setInputCloud(out);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min_, z_max_);
  pass.filter(*temp);

  return temp;
}

bool TabletopSegmentation::segmentTable(
    const CloudT::Ptr& cloud,
    pcl::PointIndices::Ptr& table_inliers,
    pcl::ModelCoefficients::Ptr& table_coefficients) const
{
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(plane_distance_threshold_);
  seg.setMaxIterations(plane_max_iterations_);
  seg.setInputCloud(cloud);
  seg.segment(*table_inliers, *table_coefficients);

  return !table_inliers->indices.empty();
}

void TabletopSegmentation::extractObjects(
    const CloudT::Ptr& cloud,
    const pcl::PointIndices::Ptr& table_inliers,
    CloudT::Ptr& objects_cloud) const
{
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.setNegative(true);
  extract.filter(*objects_cloud);
}

std::vector<pcl::PointIndices>
TabletopSegmentation::clusterObjects(const CloudT::Ptr& cloud) const
{
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  return cluster_indices;
}

void TabletopSegmentation::publishTablePlane(
    const CloudT::Ptr& cloud,
    const pcl::PointIndices::Ptr& table_inliers,
    const std_msgs::Header& header)
{
  CloudT::Ptr table_cloud(new CloudT);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.setNegative(false);
  extract.filter(*table_cloud);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*table_cloud, msg);
  msg.header = header;
  pub_table_.publish(msg);
}

void TabletopSegmentation::publishClusters(
    const CloudT::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std_msgs::Header& header)
{
  CloudT::Ptr merged(new CloudT);
  visualization_msgs::MarkerArray markers;

  int marker_id = 0;
  for (const auto& indices : cluster_indices)
  {
    PointT min_pt, max_pt;
    CloudT::Ptr cluster(new CloudT);

    for (int idx : indices.indices)
      cluster->push_back((*cloud)[idx]);

    *merged += *cluster;

    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "tabletop_clusters";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (min_pt.x + max_pt.x) * 0.5;
    marker.pose.position.y = (min_pt.y + max_pt.y) * 0.5;
    marker.pose.position.z = (min_pt.z + max_pt.z) * 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.01f, max_pt.x - min_pt.x);
    marker.scale.y = std::max(0.01f, max_pt.y - min_pt.y);
    marker.scale.z = std::max(0.01f, max_pt.z - min_pt.z);
    marker.color.a = 0.35;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.2;
    markers.markers.push_back(marker);
  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*merged, msg);
  msg.header = header;
  pub_clusters_.publish(msg);
  pub_markers_.publish(markers);
}
}  // namespace pointcloud_slam

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tabletop_segmentation_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pointcloud_slam::TabletopSegmentation node(nh, pnh);
  ros::spin();
  return 0;
}
