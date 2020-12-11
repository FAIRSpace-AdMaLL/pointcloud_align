#include <ros/ros.h>

#include <pointcloud_align/pointcloud_align.hpp>

#include <memory> 

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pointcloud_align");
  ros::NodeHandle nh;

  std::vector<std::string> datasets;

  nh.param<std::vector<std::string>>("/datasets_list", datasets, std::vector<std::string>());

  PointCloudT::Ptr refernce_cloud(new PointCloudT);

  PointCloudAlign* refernce_dataset = (new PointCloudAlign(nh, datasets[0]));

  refernce_dataset->get_cloud(refernce_cloud);
  refernce_dataset->set_as_global();
  refernce_dataset->transform_and_save();

  for (std::size_t i = 1; i < datasets.size(); i++)
  {
    PointCloudAlign* local_dataset = new PointCloudAlign(nh, datasets[i]);
    local_dataset->get_transform(refernce_cloud);
    local_dataset->transform_and_save();
  }

  ros::spin ();
}