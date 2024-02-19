// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <Eigen/Dense>
// #include <chrono>
// #include "icet.h"

// class ScanRegistrationNode {
// public:
//   ScanRegistrationNode() : firstScan(true) {
//     scan1 = Eigen::MatrixXf::Zero(1, 3);
//     scan2 = Eigen::MatrixXf::Zero(1, 3);
//   }

//   void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
//     auto before = std::chrono::system_clock::now();
//     auto beforeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(before);

//     // Convert PointCloud2 message to PCL point cloud
//     pcl::PointCloud<pcl::PointXYZ> pclCloud;
//     pcl::fromROSMsg(*msg, pclCloud);

//     std::cout << "starting to load cloud" << std::endl;

//     // Extract XYZ coordinates and convert to Eigen::MatrixXf
//     Eigen::MatrixXf matrix(pclCloud.size(), 3);
//     for (size_t i = 0; i < pclCloud.size(); ++i) {
//       matrix(i, 0) = pclCloud.points[i].x;
//       matrix(i, 1) = pclCloud.points[i].y;
//       matrix(i, 2) = pclCloud.points[i].z;
//     }

//     std::cout << "finished loading cloud" << std::endl;

//     if (!matrix.allFinite()) {
//       ROS_WARN("Matrix contains NaN or Inf values.");
//       return;
//     }

//     // Check if this is the first scan in a stream
//     if (firstScan) {
//       scan1 = matrix;
//       firstScan = false;
//     } else {
//       scan2 = matrix;
//       if (scan1.rows() == 0 || scan1.cols() == 0) {
//         ROS_WARN("Scan1 matrix is empty.");
//         return;
//       }

//       std::cout << "scan1 dimensions: " << scan1.rows() << " x " << scan1.cols() << std::endl;
//       std::cout << "scan2 dimensions: " << scan2.rows() << " x " << scan2.cols() << std::endl;

//       // Additional debug information
//       // std::cout << "scan1 content:\n" << scan1 << std::endl;
//       // std::cout << "scan2 content:\n" << scan2 << std::endl;

//       // Initialize ICET parameters and run ICET
//       Eigen::VectorXf X0(6);
//       X0 << 0., 0, 0., 0., 0., 0.;
//       int numBinsPhi = 24;
//       int numBinsTheta = 75;  // Adjust as needed
//       int n = 25;  // Adjust as needed
//       float thresh = 0.1;
//       float buff = 0.1;
//       int runlen = 5;
//       bool draw = false;

//       Eigen::VectorXf X = icet(scan1, scan2, X0, numBinsPhi, numBinsTheta, n, thresh, buff, runlen, draw);
//       std::cout << "X: \n " << X << std::endl;

//       // Update the old "new" scan to old and register the two clouds with ICET
//       scan1 = scan2;
//     }

//     auto after1 = std::chrono::system_clock::now();
//     auto after1Ms = std::chrono::time_point_cast<std::chrono::milliseconds>(after1);
//     auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(after1Ms - beforeMs).count();
//     std::cout << "Registered scans in: " << elapsedTimeMs << " ms" << std::endl;
//   }

// private:
//   bool firstScan;
//   Eigen::MatrixXf scan1;
//   Eigen::MatrixXf scan2;
// };

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "scan_registration_node");
//   ros::NodeHandle nh;

//   ScanRegistrationNode scanRegistrationNode;

//   ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ScanRegistrationNode::pointCloudCallback, &scanRegistrationNode);

//   ros::spin();

//   return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
// #define EIGEN_NO_DEBUG
#include <Eigen/Dense>
#include "icet.h"

class ScanRegistrationNode {
public:
    ScanRegistrationNode() : nh_("~") {
        // Set up ROS subscribers and publishers
        pointcloud_sub_ = nh_.subscribe("/velodyne_points", 10, &ScanRegistrationNode::pointcloudCallback, this);
        // No need to publish aligned point cloud for now
        // aligned_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_pointcloud_topic", 1);

        // Initialize the previous point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        prev_pcl_cloud_ = prev_point_cloud;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        auto before = std::chrono::system_clock::now();
        auto beforeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(before);

        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Convert PCL PointCloud to Eigen::MatrixXf
        Eigen::MatrixXf pcl_matrix = convertPCLtoEigen(pcl_cloud);

        // Convert previous PCL PointCloud to Eigen::MatrixXf

        // //use all points
        // Eigen::MatrixXf prev_pcl_matrix = convertPCLtoEigen(prev_pcl_cloud_); 

        // Filter out points less than distance 'd' from the origin
        float d = 0.25;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : prev_pcl_cloud_->points) {
            float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (distance >= d) {
                filtered_cloud->points.push_back(point);
            }
        }
        Eigen::MatrixXf prev_pcl_matrix = convertPCLtoEigen(filtered_cloud);

        // ros::Duration(0.005).sleep(); // Wait for 5 milliseconds

        Eigen::VectorXf X0(6);
        X0 << 0., 0, 0., 0., 0., 0.;
        int numBinsPhi = 24;
        int numBinsTheta = 75;  // 75 Adjust as needed
        int n = 25;  //25 Adjust as needed
        float thresh = 0.1;
        float buff = 0.1;
        int runlen = 5;
        bool draw = false;

        // std::cout << "scan1 dimensions: " << prev_pcl_matrix.rows() << " x " << 3 << std::endl;
        // std::cout << "scan2 dimensions: " << pcl_matrix.rows() << " x " << 3 << std::endl;

        Eigen::VectorXf X = icet(prev_pcl_matrix, pcl_matrix, X0, numBinsPhi, numBinsTheta, n, thresh, buff, runlen, draw);
        std::cout << "X: \n " << X << std::endl;

        auto after1 = std::chrono::system_clock::now();
        auto after1Ms = std::chrono::time_point_cast<std::chrono::milliseconds>(after1);
        auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(after1Ms - beforeMs).count();
        std::cout << "Registered scans in: " << elapsedTimeMs << " ms" << std::endl;

        // std::cout << "\n ____________________ \n" << std::endl;

        // Update the previous point cloud for the next iteration
        prev_pcl_cloud_ = pcl_cloud;

        // Convert back to ROS PointCloud2 and publish the aligned point cloud
        // sensor_msgs::PointCloud2 aligned_cloud_msg;
        // pcl::toROSMsg(*pcl_cloud, aligned_cloud_msg);
        // aligned_cloud_msg.header = msg->header;
        // aligned_pointcloud_pub_.publish(aligned_cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    // No need to publish aligned point cloud for now
    // ros::Publisher aligned_pointcloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pcl_cloud_;

    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_registration_node");
    ScanRegistrationNode scan_registration_node;
    ros::spin();
    return 0;
}
