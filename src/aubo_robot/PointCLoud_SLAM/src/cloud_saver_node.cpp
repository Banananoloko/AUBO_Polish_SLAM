#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudSaver
{
public:
  CloudSaver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    pnh.param("output_dir", output_dir_, std::string("/tmp/clouds"));
    sub_ = nh.subscribe("/input_cloud", 1, &CloudSaver::callback, this);
    ROS_INFO("CloudSaver initialized, output_dir=%s", output_dir_.c_str());
  }

private:
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    if (cloud.empty())
      return;

    std::ostringstream oss;
    oss << output_dir_ << "/cloud_" << ros::Time::now().toNSec() << ".pcd";

    if (pcl::io::savePCDFileBinary(oss.str(), cloud) == 0)
      ROS_INFO_STREAM("Saved cloud: " << oss.str());
    else
      ROS_ERROR_STREAM("Failed to save cloud: " << oss.str());
  }

  ros::Subscriber sub_;
  std::string output_dir_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_saver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CloudSaver node(nh, pnh);
  ros::spin();
  return 0;
}
