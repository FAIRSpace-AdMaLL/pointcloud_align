#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/Path.h>

// pcl::PointXYZI
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudAlign
{
  ros::NodeHandle& nh_;
  ros::Publisher pc_pub_;
  ros::Publisher path_pub_;
  
  rosbag::Bag bag_;

  std::string name_prefix_;
  std::vector<std::string> pcd_list_;
  std::vector<std::string> bag_cloud_list_;

  int iterations_;
  int correspondence_;
  int ransac_;

  double initial_x_;
  double initial_y_;
  double initial_rot_;

  PointCloudT::Ptr tf_cloud_;
  PointCloudT::Ptr input_cloud_;
  PointCloudT::Ptr transformed_cloud_;
  
  Eigen::Matrix4d tf_matrix_;

  nav_msgs::Path path_;

public:
  PointCloudAlign(ros::NodeHandle& nh, std::string param_prefix);

  void get_transform(const PointCloudT::Ptr& refernce_cloud);
  void get_cloud(PointCloudT::Ptr& refernce_cloud);
  void set_as_global();
  void transform_and_save();
  void transform_path(std::string write_dir);
  void transform_pointcloud_from_bag(std::string name, std::string read_topic, std::string pub_topic);
  void transform_pcd_batch(std::string name, std::string read_dir, std::string write_dir, std::string pub_topic, bool is_local);

  bool transform_pcd;
};
