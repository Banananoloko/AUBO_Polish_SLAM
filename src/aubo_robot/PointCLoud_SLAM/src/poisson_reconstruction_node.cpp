#include "PointCLoud_SLAM/poisson_reconstruction.hpp"

#include <sstream>
#include <cstdlib>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/poisson.h>
#include <pcl_conversions/pcl_conversions.h>


namespace pointcloud_slam
{
PoissonReconstruction::PoissonReconstruction(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  pnh.param("normal_estimation/k_search", normal_k_search_, 20);
  pnh.param("poisson/depth", poisson_depth_, 8);
  pnh.param("poisson/solver_divide", poisson_solver_divide_, 8);
  pnh.param("poisson/iso_divide", poisson_iso_divide_, 8);
  pnh.param("poisson/point_weight", poisson_point_weight_, 4.0);
  pnh.param("save_mesh", save_mesh_, true);
  pnh.param("mesh_output_dir", mesh_output_dir_, std::string("/tmp/tabletop_meshes"));

  sub_clusters_ = nh.subscribe("/input_clusters", 1, &PoissonReconstruction::cloudCallback, this);
  pub_mesh_marker_ = nh.advertise<visualization_msgs::Marker>("/tabletop/mesh_marker", 1);

  ROS_INFO("PoissonReconstruction node initialized.");
}

void PoissonReconstruction::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  using PointNormalT = pcl::PointXYZRGBNormal;

  CloudT::Ptr cloud(new CloudT);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->size() < 100)
  {
    ROS_WARN_THROTTLE(2.0, "Cluster cloud too small for Poisson reconstruction.");
    return;
  }

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(normal_k_search_);
  ne.compute(*normals);

  pcl::PointCloud<PointNormalT>::Ptr cloud_with_normals(new pcl::PointCloud<PointNormalT>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

  pcl::Poisson<PointNormalT> poisson;
  poisson.setDepth(poisson_depth_);
  poisson.setSolverDivide(poisson_solver_divide_);
  poisson.setIsoDivide(poisson_iso_divide_);
  poisson.setPointWeight(poisson_point_weight_);
  poisson.setInputCloud(cloud_with_normals);

  pcl::PolygonMesh mesh;
  poisson.reconstruct(mesh);

  ROS_INFO("Poisson mesh reconstructed: polygons=%zu", mesh.polygons.size());

  if (save_mesh_)
  {
    try
    {
      std::string cmd = "mkdir -p " + mesh_output_dir_;
      std::system(cmd.c_str());
      std::ostringstream oss;
      oss << mesh_output_dir_ << "/mesh_" << ros::Time::now().toNSec() << ".ply";
      pcl::io::savePLYFileBinary(oss.str(), mesh);
      ROS_INFO_STREAM("Saved mesh to: " << oss.str());
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Failed to save mesh: " << e.what());
    }
  }

  visualization_msgs::Marker marker;
  marker.header = msg->header;
  marker.ns = "poisson_mesh";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.text = "Poisson mesh saved";
  pub_mesh_marker_.publish(marker);
}
}  // namespace pointcloud_slam

int main(int argc, char** argv)
{
  ros::init(argc, argv, "poisson_reconstruction_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pointcloud_slam::PoissonReconstruction node(nh, pnh);
  ros::spin();
  return 0;
}
