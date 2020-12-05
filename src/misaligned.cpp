#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pointcloud_align/misaligned.hpp>

misaligned::misaligned(ros::NodeHandle& nh, std::string param_prefix, PointCloudT::Ptr baseline_cloud)
  : nh_(nh), name_(param_prefix), baseline_cloud_(baseline_cloud), experiment_cloud_(new PointCloudT)
{
  ROS_INFO("Aligning %s", name_.c_str());
  // create publishers
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name_ + "/pointcloud", 1, true);
  pc_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name_ + "/transformed_pcd_cloud", 1, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>(name_ + "/path", 1, true);

  nh.param<std::string>(name_ + "/experiment_bag", bag_name_, "");
  nh.param<bool>(name_ + "/transform_pcd", transform_pcd, false);
  nh.param<std::string>(name_ + "/pcd_in_path", pcd_in_dir_, "");
  nh.param<std::string>(name_ + "/pcd_out_path", pcd_out_dir_, "");
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
  for (const auto& messageInstance : bag_view)
  {
    sensor_msgs::PointCloud2ConstPtr pc_msg = messageInstance.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg != NULL)
    {
      pcl::fromROSMsg(*pc_msg, *experiment_cloud_);
      ROS_INFO("Found %s experiment pointcloud.", name_.c_str());
    }

    nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
    if (path_msg != NULL)
    {
      path_ = *path_msg;
      ROS_INFO("Found %s experiment path.", name_.c_str());
    }
  }
}

bool misaligned::icp()
{
  PointCloudT::Ptr cloud_tr(new PointCloudT);
  *cloud_tr = *experiment_cloud_;

  Eigen::Affine3f initial_tf_affine;
  Eigen::Matrix4d initial_tf_matrix;

  initial_tf_affine = pcl::getTransformation(initial_x_, initial_y_, 0, 0, 0, initial_rot_ * 3.14 / 180);
  initial_tf_matrix = initial_tf_affine.cast<double>().matrix();
  pcl::transformPointCloud(*cloud_tr, *experiment_cloud_, initial_tf_matrix);

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

  if (icp.hasConverged())
  {
    Eigen::Matrix4d icp_tf_matrix = icp.getFinalTransformation().cast<double>();

    // transform the experiment cloud to the correct pose
    Eigen::Matrix4d resultant_tf = icp_tf_matrix * initial_tf_matrix;
    pcl::transformPointCloud(*cloud_tr, *experiment_cloud_, resultant_tf);

    Eigen::Vector4d initial_position;
    Eigen::Vector4d transformed_position;
    tf2::Matrix3x3 transformed_orientation;
    tf2::Quaternion quat;

    ROS_INFO("Transforming poses...");
    // transform pose from the local frame to the global frame
    for (auto& pose : path_.poses)
    {
      // Transforming position
      initial_position[0] = pose.pose.position.x;
      initial_position[1] = pose.pose.position.y;
      initial_position[2] = pose.pose.position.z;
      initial_position[3] = 1;

      transformed_position = resultant_tf * initial_position;
      pose.pose.position.x = transformed_position[0];
      pose.pose.position.y = transformed_position[1];
      pose.pose.position.z = transformed_position[2];

      // Transforming orientation
      tf2::Matrix3x3 tf_rot_mat(resultant_tf(0, 0), resultant_tf(0, 1), resultant_tf(0, 2), resultant_tf(1, 0),
                                resultant_tf(1, 1), resultant_tf(1, 2), resultant_tf(2, 0), resultant_tf(2, 1),
                                resultant_tf(2, 2));

      tf2::fromMsg(pose.pose.orientation, quat);
      tf2::Matrix3x3 initial_rotation(quat);

      transformed_orientation = initial_rotation * tf_rot_mat;

      transformed_orientation.getRotation(quat);
      pose.pose.orientation = tf2::toMsg(quat);
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
    PCL_ERROR("\nICP has not converged.\n");
    return false;
  }
}

void misaligned::write_to_csv(std::string dir)
{
  std::ofstream csv_file;
  csv_file.open(dir + name_ + ".csv");
  csv_file << "#timestamp, tx, ty, tz, qx, qy, qz, qw\n";

  for (auto& pose : path_.poses)
  {
    csv_file << pose.header.stamp << ", " << pose.pose.position.x << ", " << pose.pose.position.y << ", "
             << pose.pose.position.z << ", " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", "
             << pose.pose.orientation.z << ", " << pose.pose.orientation.w << std::endl;
  }
  csv_file.close();
}

void misaligned::transform_pcd_files()
{
  std::cout << "Transforming PCD files...\n";

  // create directory and remove old files;
  int unused = system((std::string("exec rm -r ") + pcd_out_dir_).c_str());
  unused = system((std::string("mkdir -p ") + pcd_out_dir_).c_str());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr full_transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Affine3f transformation_affine;
  Eigen::Matrix4d transformation_matrix;
  geometry_msgs::PoseStamped pose;
  double roll, pitch, yaw;
  std::string pcd_path;
  sensor_msgs::PointCloud2 pc_msg;

  int len = path_.poses.size();
  for (int i = 0; i < len && ros::ok(); i++)
  {
    char file_name_buffer[11];  // xxxxxx.pcd
    sprintf(file_name_buffer, "%06d.pcd", i);
    pcd_path = pcd_in_dir_ + std::string(file_name_buffer);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud_source) == -1)  //* load the file
    {
      ROS_ERROR("Couldn't read file %s \n", pcd_path.c_str());
      return;
    }

    pose = path_.poses[i];
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                     pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    transformation_affine =
        pcl::getTransformation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, roll, pitch, yaw);

    transformation_matrix = transformation_affine.cast<double>().matrix();

    pcl::transformPointCloud(*cloud_source, *cloud_transformed, transformation_matrix);
    // *full_transformed_cloud += *cloud_source;

    pcl::toROSMsg(*cloud_transformed, pc_msg);
    pc_msg.header.frame_id = "odom";
    pc_msg.header.stamp = ros::Time::now();
    pc_pcd_pub_.publish(pc_msg);

    pcd_path = pcd_out_dir_ + std::string(file_name_buffer);
    pcl::io::savePCDFile(pcd_path, *cloud_transformed);
    std::cout << pcd_path << " saved.\n";
  }
}
