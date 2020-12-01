#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// ICP
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// rosbag includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Path.h>

#include <pointcloud_align/misaligned.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "transformer");
  ros::NodeHandle nh;
  ros::Publisher baseline_pc_pub;
  ros::Publisher baseline_path_pub;

  baseline_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/baseline_pointcloud", 1, true);
  baseline_path_pub = nh.advertise<nav_msgs::Path>("/baseline_path", 1, true);

  std::string baseline_bag_name;
  std::string baseline_name;
  std::string csv_dir;
  std::vector<std::string> experiments_list;
  int experiments_num;

  nh.param<std::string>("/baseline_bag", baseline_bag_name, "");
  nh.param<std::string>("/baseline_name", baseline_name, "");
  nh.param<std::string>("/csv_dir", csv_dir, "");
  nh.param<std::vector<std::string>>("/experiments_list", experiments_list, std::vector<std::string>());
  
  ROS_INFO("Reading from:\n%s", baseline_bag_name.c_str());

  rosbag::Bag baseline_bag;
  
  baseline_bag.open(baseline_bag_name, rosbag::bagmode::Read);  // bag containing reference cloud and path

  // Topics to read
  std::vector<std::string> topics;
  topics.push_back(std::string("/lio_sam/mapping/map_global"));
  topics.push_back(std::string("/lio_sam/mapping/path"));

  rosbag::View baseline_view(baseline_bag, rosbag::TopicQuery(topics));

  PointCloudT::Ptr baseline_cloud(new PointCloudT);
  nav_msgs::Path baseline_path;

  // Read baseline pointcloud
  for (const auto& messageInstance : baseline_view) {
    sensor_msgs::PointCloud2ConstPtr pc_msg = messageInstance.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg != NULL) {
      pcl::fromROSMsg(*pc_msg, *baseline_cloud);
      ROS_INFO("Found basline pointcloud.");
    }

    nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
    if (path_msg != NULL) {
      baseline_path = *path_msg;
      ROS_INFO("Found baseline path.");
    } 
  }

  ROS_INFO("Publish teach data");
  baseline_path_pub.publish(baseline_path);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*baseline_cloud, pc_msg);
  baseline_pc_pub.publish(pc_msg);

  ROS_INFO("Aligning experiments");
  for(auto name : experiments_list)
  {
    misaligned* experiment = (new misaligned (nh, name, baseline_cloud));
    experiment->icp();
    experiment->write_to_csv(csv_dir);

    if(experiment->transform_pcd)
      experiment->transform_pcd_files();
  }

  std::ofstream csv_file;
  csv_file.open (csv_dir + baseline_name + ".csv");
  csv_file << "#timestamp, tx, ty, tz, qx, qy, qz, qw\n";

  for (auto & pose : baseline_path.poses) {
      csv_file <<
          pose.header.stamp << ", " <<
          pose.pose.position.x << ", " <<
          pose.pose.position.y << ", " <<
          pose.pose.position.z << ", " <<
          pose.pose.orientation.x << ", " <<
          pose.pose.orientation.y << ", " <<
          pose.pose.orientation.z << ", " <<
          pose.pose.orientation.w << std::endl;
    }
    csv_file.close();

  // Spin
  ros::spin ();
}