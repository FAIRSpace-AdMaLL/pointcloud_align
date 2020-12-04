#pragma once

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/Path.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class misaligned
{
  ros::NodeHandle& nh_;
  ros::Publisher pc_pub_;
  ros::Publisher path_pub_;
  ros::Publisher pc_pcd_pub_;

  std::string name_;
  std::string bag_name_;
  std::string pcd_in_dir_;
  std::string pcd_out_dir_;

  int iterations_;
  int correspondence_;
  int ransac_;

  double initial_x_;
  double initial_y_;
  double initial_rot_;

  PointCloudT::Ptr baseline_cloud_;
  PointCloudT::Ptr experiment_cloud_;

  nav_msgs::Path path_;

public:
  misaligned(ros::NodeHandle& nh, std::string param_prefix, PointCloudT::Ptr baseline_cloud);

  bool icp();
  void write_to_csv(std::string dir);
  void transform_pcd_files();

  bool transform_pcd;
};
