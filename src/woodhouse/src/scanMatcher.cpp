#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "Eigen/Dense"
#include "icet.h"

bool firstScan = true;
Eigen::MatrixXf scan1;
Eigen::MatrixXf scan2;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

  auto before = std::chrono::system_clock::now();
  auto beforeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(before);

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


  //check if this is the first scan in a strem
  if (firstScan == true){
    scan1 = matrix;
    firstScan = false;
  }
  //if not first scan in stream, set old "new" scan to old and register the two clouds with ICET
  else{
    scan2 = matrix;
    std::cout << "\n scan1: \n" << scan1.size()/3 << std::endl;
    // std::cout << "\n" << scan1.row(0) <<endl;
    std::cout << "\n scan2: \n" << scan2.size()/3 << std::endl;
    // std::cout << "\n" << scan2.row(0) <<endl;

    //init ICET parameters
    Eigen::VectorXf X0(6);
    X0 << 0., 0, 0., 0., 0., 0.;
    int numBinsPhi = 16;
    int numBinsTheta = 40;
    int n = 10; // min size of the cluster
    float thresh = 0.3; // Jump threshold for beginning and ending radial clusters
    float buff = 0.5; //buffer to add to inner and outer cluster range (helps attract nearby distributions)
    int runlen = 5; //number of iterations
    bool draw = false;

    //run ICET
    Eigen::VectorXf X = icet(scan1, scan2, X0, numBinsPhi, numBinsTheta, n, thresh, buff, runlen, draw);
    std::cout << "X: \n " << X << std::endl;

    scan1 = scan2;
  }

  auto after1 = std::chrono::system_clock::now();
  auto after1Ms = std::chrono::time_point_cast<std::chrono::milliseconds>(after1);
  auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(after1Ms - beforeMs).count();
  std::cout << "loaded point clouds in: " << elapsedTimeMs << " ms" << std::endl;
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
