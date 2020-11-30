#include <fstream>

// rosbag includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pointcloud_align/misaligned.hpp>

misaligned::misaligned(ros::NodeHandle & nh, std::string param_prefix, PointCloudT::Ptr baseline_cloud)
:   nh_(nh),
    name_(param_prefix),
    baseline_cloud_(baseline_cloud),
    experiment_cloud_(new PointCloudT)
{
    ROS_INFO("Aligning %s", name_.c_str());
    // create publishers
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name_ + "/pointcloud", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>(name_ + "/path", 1, true);

    nh.param<std::string>(name_ + "/experiment_bag", bag_name_, "");
    nh.param<int>(name_ + "/icp_iterations", iterations_, 100);
    nh.param<int>(name_ + "/correspondence", correspondence_, 5);
    nh.param<int>(name_ + "/ransac", ransac_, 0);
    nh.param<double>(name_ + "/initial_rot", initial_rot_, 90.0);

    // Topics to read
    std::vector<std::string> topics;
    topics.push_back(std::string("/lio_sam/mapping/map_global"));
    topics.push_back(std::string("/lio_sam/mapping/path"));

    rosbag::Bag bag;
    ROS_INFO("Opening %s", bag_name_.c_str());
    bag.open(bag_name_, rosbag::bagmode::Read);
    rosbag::View bag_view(bag, rosbag::TopicQuery(topics));

    // Read experiment pointcloud and path
    for (const auto& messageInstance : bag_view) {
        sensor_msgs::PointCloud2ConstPtr pc_msg = messageInstance.instantiate<sensor_msgs::PointCloud2>();
        if (pc_msg != NULL) {
            pcl::fromROSMsg(*pc_msg, *experiment_cloud_);
            ROS_INFO("Found %s experiment pointcloud.",  name_.c_str());
            pc_pub_.publish(pc_msg);
        }

        nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
        if (path_msg != NULL) {
            path_ = *path_msg;
            ROS_INFO("Found %s experiment path.",  name_.c_str());
        } 
    }

    path_pub_.publish(path_);
}

bool misaligned::icp()
{
   PointCloudT::Ptr cloud_tr (new PointCloudT);
  *cloud_tr = *experiment_cloud_;

  Eigen::Matrix4d initial_rot_matrix = Eigen::Matrix4d::Identity ();
  initial_rot_matrix(0, 0) = std::cos(initial_rot_);
  initial_rot_matrix(0, 1) = -std::sin(initial_rot_);
  initial_rot_matrix(1, 0) = std::sin(initial_rot_);
  initial_rot_matrix(1, 1) = std::cos(initial_rot_);
  pcl::transformPointCloud (*cloud_tr, *experiment_cloud_, initial_rot_matrix);

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations_);
  icp.setMaxCorrespondenceDistance(correspondence_);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(ransac_);
  icp.setInputSource(experiment_cloud_);
  icp.setInputTarget(baseline_cloud_);
  icp.align(*experiment_cloud_);

  ROS_INFO("%s ICP finished", name_.c_str());

  if (icp.hasConverged ())
  {
    // std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    // std::cout << " : experiment_cloud_ -> baseline_cloud_" << std::endl;
    Eigen::Matrix4d icp_rt_matrix = icp.getFinalTransformation ().cast<double>();
    // print4x4Matrix (icp_rt_matrix);

    // transform the experiment cloud to the correct pose
    pcl::transformPointCloud (*cloud_tr, *experiment_cloud_, Eigen::Matrix4d(icp_rt_matrix*initial_rot_matrix));

    // Get the rotation and translation
    Eigen::Matrix2d rot = icp_rt_matrix.block(0,0,2,2) * initial_rot_matrix.block(0,0,2,2);
    Eigen::Vector2d trans = icp_rt_matrix.block(0,3,2,1);

    ROS_INFO("Correcting poses");
    // transform the experiment path to the teach frame
    for (auto & pose : path_.poses)
    {
      Eigen::Vector2d cur_pose(pose.pose.position.x, pose.pose.position.y);
      Eigen::Vector2d corrected_pose = rot*cur_pose + trans;

      pose.pose.position.x = corrected_pose[0];
      pose.pose.position.y = corrected_pose[1];
    }

    ROS_INFO("%s publishing messages", name_.c_str());
    

    path_pub_.publish(path_);

    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*experiment_cloud_, pc_msg);
    pc_pub_.publish(pc_msg);

    return true;
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return false;
  }
}

void misaligned::write_to_csv(std::string dir)
{
    std::ofstream csv_file;
    csv_file.open (dir + name_ + ".csv");
    csv_file << "#timestamp, tx, ty, tz, qx, qy, qz, qw\n";

    for (auto & pose : path_.poses) {
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
}