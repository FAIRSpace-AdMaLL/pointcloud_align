#include <ros/ros.h>

#include <pointcloud_align/pointcloud_align.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pointcloud_align");
  ros::NodeHandle nh;

  std::vector<std::string> datasets;

  nh.param<std::vector<std::string>>("/datasets_list", datasets, std::vector<std::string>());

  PointCloudT::Ptr refernce_cloud(new PointCloudT);

  PointCloudAlign* refernce_dataset = (new PointCloudAlign(nh, datasets[0]));

  refernce_dataset->get_cloud(&refernce_cloud);

  for (std::size_t i = 1; i < dataset.size(); i++)
  {
    std::unique_ptr<PointCloudAlign> refernce_dataset = std::make_unique<PointCloudAlign>(nh, datasets[i]);
    refernce_dataset->get_transform(refernce_cloud);
    refernce_dataset->transform_and_save();
  }

  ros::spin ();
}