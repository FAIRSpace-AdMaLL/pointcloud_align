#include <fstream>

#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pointcloud_align/pointcloud_align.hpp>

PointCloudAlign::PointCloudAlign(ros::NodeHandle& nh, std::string name_prefix)
: nh_(nh),
name_prefix_(name_prefix),
tf_cloud_(new pcl::PointCloud<PointT>),
input_cloud_(new pcl::PointCloud<PointT>),
transformed_cloud_(new pcl::PointCloud<PointT>)
{
  ROS_INFO("Aligning %s", name_prefix_.c_str());

  std::string bag_path;
  std::string pointcloud_topic;
  std::string path_topic;

  nh.param<std::string>(name_prefix_ + "/pointcloud_and_trajectory_rosbag", bag_path, "");
  nh.param<std::string>(name_prefix_ + "/pointcloud_topic", pointcloud_topic, "");
  nh.param<std::string>(name_prefix_ + "/path_topic", path_topic, "");
  nh.param<std::vector<std::string>>(name_prefix_ + "/pcd_list", pcd_list_, std::vector<std::string>());
  nh.param<std::vector<std::string>>(name_prefix_ + "/bag_cloud_list", bag_cloud_list_, std::vector<std::string>());

  // create publishers
  pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(name_prefix_ + "/pointcloud", 1, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>(name_prefix_ + "/path", 1, true);

  // Topics to read
  std::vector<std::string> topics;
  topics.push_back(pointcloud_topic);
  topics.push_back(path_topic);

  ROS_INFO("Opening %s", bag_path.c_str());
  bag_.open(bag_path, rosbag::bagmode::Read);
  rosbag::View bag_view(bag_, rosbag::TopicQuery(topics));

  // Read dataset pointcloud and path
  for (const auto& messageInstance : bag_view)
  {
    sensor_msgs::PointCloud2ConstPtr pc_msg = messageInstance.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg != NULL)
    {
      pcl::fromROSMsg(*pc_msg, *input_cloud_);
      pc_pub_.publish(pc_msg);
      ROS_INFO("Found %s dataset pointcloud.", name_prefix_.c_str());
    }

    nav_msgs::Path::ConstPtr path_msg = messageInstance.instantiate<nav_msgs::Path>();
    if (path_msg != NULL)
    {
      path_ = *path_msg;
      path_pub_.publish(path_);
      ROS_INFO("Found %s dataset path.", name_prefix_.c_str());
    }
  }
}

void PointCloudAlign::get_cloud(PointCloudT::Ptr& refernce_cloud)
{
  *refernce_cloud = *input_cloud_;
}

void PointCloudAlign::set_as_global()
{
  tf_matrix_ = Eigen::Matrix4d::Identity();
}

void PointCloudAlign::get_transform(const PointCloudT::Ptr& refernce_cloud)
{
  // ros::Publisher pc_pub = nh_.advertise<sensor_msgs::PointCloud2>(name_prefix_ + "/tf_pointcloud", 1, true);
  ROS_INFO("%s: Getting transform...", name_prefix_.c_str());
  PointCloudT::Ptr ref_cloud(new PointCloudT);
  *ref_cloud = *refernce_cloud;

  int iterations;
  int correspondence;
  int ransac;
  double downsample;
  double init_x;
  double init_y;
  double init_yaw;

  nh_.param<int>(name_prefix_ + "/icp/iterations", iterations, 100);
  nh_.param<int>(name_prefix_ + "/icp/ransac", ransac, 0);
  nh_.param<int>(name_prefix_ + "/icp/correspondence", correspondence, 5);
  nh_.param<double>(name_prefix_ + "/icp/downsample", downsample, 10.0);
  nh_.param<double>(name_prefix_ + "/icp/initial_x", init_x, 0.0);
  nh_.param<double>(name_prefix_ + "/icp/initial_y", init_y, 0.0);
  nh_.param<double>(name_prefix_ + "/icp/initial_yaw", init_yaw, 0.0);

  Eigen::Affine3f initial_tf_affine;
  Eigen::Matrix4d initial_tf_matrix;

  initial_tf_affine = pcl::getTransformation(init_x, init_y, 0, 0, 0, init_yaw * 3.14 / 180);
  initial_tf_matrix = initial_tf_affine.cast<double>().matrix();
  pcl::transformPointCloud(*input_cloud_, *input_cloud_, initial_tf_matrix);

  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*input_cloud_, pc_msg);
  pc_pub_.publish(pc_msg);

  // pcl::VoxelGrid<PointT> downSizeFilter;
  // downSizeFilter.setLeafSize(downsample, downsample, downsample);

  // downSizeFilter.setInputCloud(input_cloud_);
  // downSizeFilter.filter(*input_cloud_);

  // downSizeFilter.setInputCloud(ref_cloud);
  // downSizeFilter.filter(*ref_cloud);

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations);
  icp.setMaxCorrespondenceDistance(correspondence);
  icp.setRANSACIterations(ransac);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-8);
  icp.setInputSource(input_cloud_);
  icp.setInputTarget(ref_cloud);
  icp.align(*tf_cloud_);

  if (icp.hasConverged())
  {
    tf_matrix_ = icp.getFinalTransformation().cast<double>()* initial_tf_matrix;
    ROS_INFO("%s: ICP converged.", name_prefix_.c_str());

    pcl::toROSMsg(*tf_cloud_, pc_msg);
    pc_pub_.publish(pc_msg);
    ROS_INFO("%s: Published transformed pointcloud.", name_prefix_.c_str());
  }
  else
  {
    ROS_ERROR("%s: ICP has not converged.", name_prefix_.c_str());
    return;
  }
}

void PointCloudAlign::transform_path(std::string write_dir)
{
  // transform pose from the local frame to the global frame
  Eigen::Vector4d initial_position;
  Eigen::Vector4d transformed_position;
  tf2::Matrix3x3 transformed_orientation;
  tf2::Quaternion quat;

  ROS_INFO("Transforming poses...");
  for (auto& pose : path_.poses)
  {
    // Transforming position
    initial_position[0] = pose.pose.position.x;
    initial_position[1] = pose.pose.position.y;
    initial_position[2] = pose.pose.position.z;
    initial_position[3] = 1;

    transformed_position = tf_matrix_ * initial_position;
    pose.pose.position.x = transformed_position[0];
    pose.pose.position.y = transformed_position[1];
    pose.pose.position.z = transformed_position[2];

    // Transforming orientation
    tf2::Matrix3x3 tf_rot_mat(tf_matrix_(0, 0), tf_matrix_(0, 1), tf_matrix_(0, 2), tf_matrix_(1, 0), tf_matrix_(1, 1),
                              tf_matrix_(1, 2), tf_matrix_(2, 0), tf_matrix_(2, 1), tf_matrix_(2, 2));

    tf2::fromMsg(pose.pose.orientation, quat);
    tf2::Matrix3x3 initial_rotation(quat);

    transformed_orientation = initial_rotation * tf_rot_mat;

    transformed_orientation.getRotation(quat);
    pose.pose.orientation = tf2::toMsg(quat);
  }

  path_pub_.publish(path_);
  ROS_INFO("Published transformed path.");

  if (write_dir != "")
  {
    ROS_INFO("Writing to CSV.");
    std::ofstream csv_file;
    // TODO: confirm if that creates dir if one doesn't exist or does that need to be explicitly handled
    csv_file.open(write_dir);
    csv_file << "#timestamp, tx, ty, tz, qx, qy, qz, qw\n";

    for (auto& pose : path_.poses)
    {
      csv_file << pose.header.stamp << ", " << pose.pose.position.x << ", " << pose.pose.position.y << ", "
               << pose.pose.position.z << ", " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", "
               << pose.pose.orientation.z << ", " << pose.pose.orientation.w << std::endl;
    }
    csv_file.close();
  }
}

void PointCloudAlign::transform_and_save()
{
  // Transform path and save it to csv
  std::string csv_path;
  nh_.param<std::string>(name_prefix_ + "/csv_path", csv_path, "");
  transform_path(csv_path);

  // Transform pointcloud from bag
  std::string read_topic;
  std::string pub_topic;

  for (auto dataset : bag_cloud_list_)
  {
    ROS_INFO("Transforming: %s", dataset.c_str());

    nh_.param<std::string>(name_prefix_ + "/" + dataset + "/read_topic", read_topic, "");
    nh_.param<std::string>(name_prefix_ + "/" + dataset + "/pub_topic", pub_topic, "");
    transform_pointcloud_from_bag(dataset, read_topic, pub_topic);
  }

  // Transform PCD files
  std::string read_dir;
  std::string write_dir;

  for (auto dataset : pcd_list_)
  {
    ROS_INFO("Transforming: %s", dataset.c_str());

    nh_.param<std::string>(name_prefix_ + "/" + dataset + "/read_dir", read_dir, "");
    nh_.param<std::string>(name_prefix_ + "/" + dataset + "/write_dir", write_dir, "");
    nh_.param<std::string>(name_prefix_ + "/" + dataset + "/publish_topic", pub_topic, "");

    transform_pcd_batch(dataset, read_dir, write_dir, pub_topic);
  }
}

void PointCloudAlign::transform_pointcloud_from_bag(std::string name, std::string read_topic, std::string pub_topic)
{
  ros::Publisher* pc_pub = new ros::Publisher();

  *pc_pub = nh_.advertise<sensor_msgs::PointCloud2>(name_prefix_ + "/"+ name + pub_topic, 1, true);

  rosbag::View bag_view(bag_, rosbag::TopicQuery(std::vector<std::string>(1, read_topic)));

  // Read pointcloud
  for (const auto& messageInstance : bag_view)
  {
    sensor_msgs::PointCloud2ConstPtr pc_msg_ptr = messageInstance.instantiate<sensor_msgs::PointCloud2>();
    if (pc_msg_ptr != NULL)
    {
      input_cloud_->clear();
      pcl::fromROSMsg(*pc_msg_ptr, *input_cloud_);

      tf_cloud_->clear();
      pcl::transformPointCloud(*input_cloud_, *tf_cloud_, tf_matrix_);

      sensor_msgs::PointCloud2 pc_msg;
      pcl::toROSMsg(*tf_cloud_, pc_msg);
      pc_msg.header.frame_id = "odom";
      pc_msg.header.stamp = ros::Time::now();
      pc_pub->publish(pc_msg);
      ROS_INFO("Publishing %s dataset pointcloud.", name.c_str());
    }
  }
}

void PointCloudAlign::transform_pcd_batch(std::string name, std::string read_dir, std::string write_dir, std::string pub_topic)
{
  ROS_INFO("Transforming %s PCD files...", name.c_str());

  // create directory and remove old files;
  if (write_dir != " ")
  {
    int unused = system((std::string("exec rm -r ") + write_dir).c_str());
    unused = system((std::string("mkdir -p ") + write_dir).c_str());
  }

  pcl::PointCloud<PointT>::Ptr cloud_source(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr full_transformed_cloud(new pcl::PointCloud<PointT>);
  Eigen::Affine3f tf_affine;
  Eigen::Matrix4d tf_matrix;
  geometry_msgs::PoseStamped pose;
  double roll, pitch, yaw;
  std::string pcd_path;
  sensor_msgs::PointCloud2 pc_msg;
  ros::Publisher pc_pub;

  pc_pub = nh_.advertise<sensor_msgs::PointCloud2>(name_prefix_ + pub_topic, 1, true);
  pcl::VoxelGrid<PointT> downSizeFilter;
  downSizeFilter.setLeafSize(10.0, 10.0, 10.0);
  int len = path_.poses.size();

  for (int i = 0; i < len && ros::ok(); i++)
  {
    // Generating PCD file name and path
    char file_name_buffer[11];  // xxxxxx.pcd
    sprintf(file_name_buffer, "%06d.pcd", i);
    pcd_path = read_dir + std::string(file_name_buffer);

    if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud_source) == -1)  //* load the file
    {
      ROS_ERROR("Couldn't read file %s \n", pcd_path.c_str());
      return;
    }

    // Getting transform from pose
    pose = path_.poses[i];
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                     pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    tf_affine =
        pcl::getTransformation(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, roll, pitch, yaw);
    tf_matrix = tf_affine.cast<double>().matrix();

    // Transforming the cloud
    pcl::transformPointCloud(*cloud_source, *cloud_transformed, tf_matrix);

    // Publishing transformed cloud
    pcl::toROSMsg(*cloud_transformed, pc_msg);
    pc_msg.header.frame_id = "odom";
    pc_msg.header.stamp = ros::Time::now();
    pc_pub.publish(pc_msg);

    // Saving transformed cloud to a PCD if a dir is provided
    if (write_dir != " ")
    {
      pcd_path = write_dir + std::string(file_name_buffer);
      pcl::io::savePCDFile(pcd_path, *cloud_transformed);
      ROS_INFO("pcd_path %s  saved.", name.c_str());
    }
  }

  ROS_INFO("All %s PCD files transformed.", name.c_str());
}
