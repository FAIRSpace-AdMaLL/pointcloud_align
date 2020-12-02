#include <fstream>


#include <tf/tf.h>
// rosbag includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
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
    pc_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name_ + "/transformed_pcd_cloud", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>(name_ + "/path", 1, true);

    nh.param<std::string>(name_ + "/experiment_bag", bag_name_, "");
    nh.param<bool>(name_ + "/transform_pcd", transform_pcd, false);
    nh.param<std::string>(name_ + "/pcd_in_path", pcd_in_path_, "");
    nh.param<std::string>(name_ + "/pcd_in_path", pcd_out_path_, "");
    nh.param<int>(name_ + "/icp_iterations", iterations_, 100);
    nh.param<int>(name_ + "/correspondence", correspondence_, 5);
    nh.param<int>(name_ + "/ransac", ransac_, 0);
    nh.param<double>(name_ + "/initial_rot", initial_rot_, 90.0);
    nh.param<double>(name_ + "/initial_x", initial_x_, 0.0);
    nh.param<double>(name_ + "/initial_y", initial_y_, 0.0);

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
        }

        nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
        if (path_msg != NULL) {
            path_ = *path_msg;
            ROS_INFO("Found %s experiment path.",  name_.c_str());
        } 
    }
}

bool misaligned::icp()
{
   PointCloudT::Ptr cloud_tr (new PointCloudT);
  *cloud_tr = *experiment_cloud_;

  Eigen::Affine3f initial_transformation_affine;
  Eigen::Matrix4d initial_transformation_matrix;

  initial_transformation_affine = pcl::getTransformation(initial_x_, initial_y_, 0, 0, 0, initial_rot_*3.14/180);
  initial_transformation_matrix = initial_transformation_affine.cast<double>().matrix();
  pcl::transformPointCloud (*cloud_tr, *experiment_cloud_, initial_transformation_matrix);

  pcl::VoxelGrid<PointT> downSizeFilter;
  downSizeFilter.setLeafSize(10.0, 10.0, 10.0);
  downSizeFilter.setInputCloud(baseline_cloud_);
  downSizeFilter.filter(*baseline_cloud_);

  downSizeFilter.setLeafSize(10.0, 10.0, 10.0);
  downSizeFilter.setInputCloud(experiment_cloud_);
  downSizeFilter.filter(*experiment_cloud_);

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
    Eigen::Matrix4d icp_tf_matrix = icp.getFinalTransformation ().cast<double>();
    // print4x4Matrix (icp_tf_matrix);

    // transform the experiment cloud to the correct pose
    Eigen::Matrix4d resultant_tf = icp_tf_matrix * initial_transformation_matrix;
    pcl::transformPointCloud (*cloud_tr, *experiment_cloud_, resultant_tf);

    // Get the rotation and translation componets
    Eigen::Matrix2d rot = resultant_tf.block(0,0,2,2);
    Eigen::Vector2d trans = resultant_tf.block(0,3,2,1);

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

void misaligned::transform_pcd_files()
{
    std::cout << "Transforming PCD files...\n";

    // create directory and remove old files;
    int unused = system((std::string("exec rm -r ") + pcd_out_path_).c_str());
    unused = system((std::string("mkdir -p ") + pcd_out_path_).c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f transformation_affine;
    Eigen::Matrix4d transformation_matrix;
    double roll, pitch, yaw; 

    for(auto & pose : path_.poses) { 
      std::string file_name = std::to_string(pose.header.stamp.toSec());

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_in_path_ + file_name, *cloud_source) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
      }

      tf::Quaternion q(
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      transformation_affine = pcl::getTransformation(
        pose.pose.position.x,
        pose.pose.position.y, 
        pose.pose.position.z,
        roll, pitch, yaw);

      transformation_matrix = transformation_affine.cast<double>().matrix();

      pcl::transformPointCloud(*cloud_source, *cloud_transformed, transformation_matrix);
      *full_transformed_cloud += *cloud_transformed;

      pcl::io::savePCDFile(pcd_out_path_ + file_name, *cloud_transformed);
      std::cout << file_name << " saved.\n";

      std::cout << pcd_out_path_ + file_name << " saved.\n";
    }

    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(*full_transformed_cloud, pc_msg);
    pc_pcd_pub_.publish(pc_msg);
}