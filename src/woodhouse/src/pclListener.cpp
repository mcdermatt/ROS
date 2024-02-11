#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "Eigen/Dense"

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert PointCloud2 message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*msg, pclCloud);

  // Extract XYZ coordinates and convert to Eigen::MatrixXf
  Eigen::MatrixXf matrix(pclCloud.size(), 3);
  for (size_t i = 0; i < pclCloud.size(); ++i)
  {
    matrix(i, 0) = pclCloud.points[i].x;
    matrix(i, 1) = pclCloud.points[i].y;
    matrix(i, 2) = pclCloud.points[i].z;
  }

  std::cout << "\n pointsxyz: \n" << matrix << std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_subscriber_node");
  ros::NodeHandle nh;

  // Create a subscriber for PointCloud2 messages
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, pointCloudCallback);

  ros::spin();  // Keep spinning to receive messages

  return 0;
}
