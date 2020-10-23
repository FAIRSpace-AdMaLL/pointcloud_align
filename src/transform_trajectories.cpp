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
#include <pcl/visualization/pcl_visualizer.h>

// rosbag includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Path.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "transformer");
  ros::NodeHandle nh;
  ros::Publisher baseline_pc_pub;
  ros::Publisher experiment_pc_pub;
  ros::Publisher baseline_path_pub;
  ros::Publisher experiment_path_pub;

  baseline_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/baseline_pointcloud", 1, true);
  experiment_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/experiment_pointcloud", 1, true);
  baseline_path_pub = nh.advertise<nav_msgs::Path>("/baseline_path", 1, true);
  experiment_path_pub = nh.advertise<nav_msgs::Path>("/experiment_path", 1, true);

  std::string baseline_bag_name;
  std::string experiment_bag_name;
  int iterations;
  int correspondence;
  int ransac;
  double initial_rot;

  nh.param<std::string>("/baseline_bag", baseline_bag_name, "");
  nh.param<std::string>("/experiment_bag", experiment_bag_name, "");
  nh.param<int>("/icp_iterations", iterations, 100);
  nh.param<int>("/correspondence", correspondence, 5);
  nh.param<int>("/ransac", ransac, 0);
  nh.param<double>("/initial_rot", initial_rot, 90.0);

  ROS_INFO("Reading from:\n%s\n%s", baseline_bag_name.c_str(), experiment_bag_name.c_str());

  rosbag::Bag baseline_bag, experiment_bag;
  
  baseline_bag.open(baseline_bag_name, rosbag::bagmode::Read);  // bag containing reference cloud 
  experiment_bag.open(experiment_bag_name, rosbag::bagmode::Read);  // bag containing source cloud

  // Topics to read
  std::vector<std::string> topics;
  topics.push_back(std::string("/lio_sam/mapping/map_global"));
  topics.push_back(std::string("/lio_sam/mapping/path"));

  rosbag::View baseline_view(baseline_bag, rosbag::TopicQuery(topics));
  rosbag::View experiment_view(experiment_bag, rosbag::TopicQuery(topics));

  PointCloudT::Ptr baseline_cloud(new PointCloudT);
  PointCloudT::Ptr experiment_cloud(new PointCloudT);
  nav_msgs::Path baseline_path;
  nav_msgs::Path experiment_path;

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

  // Read experiment pointcloud and path
  for (const auto& messageInstance : experiment_view) {
    sensor_msgs::PointCloud2ConstPtr pc_msg = messageInstance.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg != NULL) {
      pcl::fromROSMsg(*pc_msg, *experiment_cloud);
      ROS_INFO("Found experiment pointcloud.");
    }

    nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
    if (path_msg != NULL) {
      experiment_path = *path_msg;
      ROS_INFO("Found experiment path.");
    } 
  }

  PointCloudT::Ptr cloud_tr (new PointCloudT);
  *cloud_tr = *experiment_cloud;


  Eigen::Matrix4d initial_rot_matrix = Eigen::Matrix4d::Identity ();
  initial_rot_matrix(0, 0) = std::cos(initial_rot);
  initial_rot_matrix(0, 1) = -std::sin(initial_rot);
  initial_rot_matrix(1, 0) = std::sin(initial_rot);
  initial_rot_matrix(1, 1) = std::cos(initial_rot);
  pcl::transformPointCloud (*cloud_tr, *experiment_cloud, initial_rot_matrix);

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations);
  icp.setMaxCorrespondenceDistance(correspondence);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(ransac);
  icp.setInputSource(experiment_cloud);
  icp.setInputTarget(baseline_cloud);
  icp.align(*experiment_cloud);

  ROS_INFO("ICP finished");

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << " : experiment_cloud -> baseline_cloud" << std::endl;
    Eigen::Matrix4d icp_rt_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (icp_rt_matrix);

    // transform the experiment cloud to the correct pose
    pcl::transformPointCloud (*cloud_tr, *experiment_cloud, icp_rt_matrix);

    // transform the experiment cloud to the correct pose
    pcl::transformPointCloud (*cloud_tr, *experiment_cloud, Eigen::Matrix4d(icp_rt_matrix*initial_rot_matrix));

    // Calculate the rot and translation
    Eigen::Matrix2d rot = icp_rt_matrix.block(0,0,2,2) * initial_rot_matrix.block(0,0,2,2);
    Eigen::Vector2d trans = icp_rt_matrix.block(0,3,2,1);

    ROS_INFO("Correcting poses");
    // transform the experiment path to the new frame
    for (auto & pose : experiment_path.poses)
    {
      Eigen::Vector2d cur_pose(pose.pose.position.x, pose.pose.position.y);
      Eigen::Vector2d corrected_pose = rot*cur_pose + trans;

      pose.pose.position.x = corrected_pose[0];
      pose.pose.position.y = corrected_pose[1];
    }

    ROS_INFO("publishing messages");
    // publish paths
    baseline_path_pub.publish(baseline_path);
    experiment_path_pub.publish(experiment_path);

    sensor_msgs::PointCloud2 pc_msg;

    pcl::toROSMsg(*baseline_cloud, pc_msg);
    baseline_pc_pub.publish(pc_msg);

    pcl::toROSMsg(*experiment_cloud, pc_msg);
    experiment_pc_pub.publish(pc_msg);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  // Spin
  ros::spin ();
}