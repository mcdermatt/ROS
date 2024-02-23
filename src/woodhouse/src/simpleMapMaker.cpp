#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>  // Add this line for tf2_ros::Buffer
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include "icet.h"

class MapMakerNode {
public:
    MapMakerNode() : nh_("~") {
        // Set up ROS subscribers and publishers
        pointcloud_sub_ = nh_.subscribe("/velodyne_points", 10, &MapMakerNode::pointcloudCallback, this);
        aligned_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hd_map", 1);

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
        Eigen::MatrixXf prev_pcl_matrix = convertPCLtoEigen(prev_pcl_cloud_); 

        // // Filter out points less than distance 'd' from the origin
        // float d = 0.25;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // for (const auto& point : prev_pcl_cloud_->points) {
        //     float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        //     if (distance >= d) {
        //         filtered_cloud->points.push_back(point);
        //     }
        // }
        // Eigen::MatrixXf prev_pcl_matrix = convertPCLtoEigen(filtered_cloud);

        // ros::Duration(0.005).sleep(); // Wait for 5 milliseconds

        Eigen::VectorXf X0(6);
        X0 << 0., 0, 0., 0., 0., 0.;
        int numBinsPhi = 24;
        int numBinsTheta = 75;  // 75 Adjust as needed
        int n = 25;  //25 Adjust as needed
        float thresh = 0.1;
        float buff = 0.125;
        int runlen = 8;
        bool draw = false;

        Eigen::VectorXf X = icet(prev_pcl_matrix, pcl_matrix, X0, numBinsPhi, numBinsTheta, n, thresh, buff, runlen, draw);
        std::cout << "X: \n " << X << std::endl;

        // Update the previous point cloud for the next iteration
        prev_pcl_cloud_ = pcl_cloud;

        // Convert back to ROS PointCloud2 and publish the aligned point cloud
        sensor_msgs::PointCloud2 aligned_cloud_msg;
        MatrixXf rot_mat = R(X[3], X[4], X[5]);
        // std::cout << rot_mat << endl;
        Eigen::RowVector3f trans(X[0], X[1], X[2]);

        Eigen:MatrixXf scan2_in_scan1_frame = (pcl_matrix * rot_mat.inverse()).rowwise() - trans;
        // std::cout << scan2_in_scan1_frame << endl;

        //"broadcast" transform that relates /velodyne frame to /map frame ~~~~~~~~~~~~~~~~~~~~~
        //convert ICET output X for scan i to homogenous transformation matrix
        Eigen::Matrix4f X_homo_i = Eigen::Matrix4f::Identity();
        X_homo_i.block<3, 3>(0, 0) = rot_mat;
        X_homo_i.block<3, 1>(0, 3) = trans.transpose();

        //update accumulated transfrom
        X_homo = X_homo * X_homo_i; //was this
        // X_homo = X_homo * X_homo_i.inverse(); //nope
        std::cout << X_homo <<endl;

        // Create a geometry_msgs::TransformStamped message
        geometry_msgs::TransformStamped transformStamped;

        // Set the frame IDs
        transformStamped.header.frame_id = "map";         // Parent frame 
        transformStamped.child_frame_id = "velodyne";     // Child frame

        // Convert Eigen rotation matrix to quaternion
        // Eigen::Matrix3f rotationMatrix = rot_mat.topLeftCorner(3, 3); //X_i
        Eigen::Matrix3f rotationMatrix = X_homo.topLeftCorner(3, 3); //X
        Eigen::Quaternionf quaternion(rotationMatrix);

        // Set the translation and rotation in the transform message
        //X_i
        // transformStamped.transform.translation.x = trans(0);
        // transformStamped.transform.translation.y = trans(1);
        // transformStamped.transform.translation.z = trans(2);
        //X
        transformStamped.transform.translation.x = X_homo(0,3);
        transformStamped.transform.translation.y = X_homo(1,3);
        transformStamped.transform.translation.z = X_homo(2,3);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        // Set the timestamp
        // std::cout << msg->header.stamp << endl;
        // transformStamped.header.stamp = msg->header.stamp;  // Use lidar scan timestamp
        transformStamped.header.stamp = ros::Time::now(); //use current timestamp --> issues with using rosbag data?
        // transformStamped.header.stamp = msg->header.stamp + ros::Duration(0.025);  // Use lidar scan timestamp + some delta

        // Broadcast the transform
        broadcaster_.sendTransform(transformStamped);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        //TODO: expand map by keeping scan2 in pcl format so we can use builtin pcl functions for transform
        //     (I think this should be more efficient than converting everything to eigen)

        // Create a header for the ROS message
        std_msgs::Header header; 
        header.stamp = ros::Time::now(); //use current timestamp --> issues with using rosbag data?
        // header.stamp = msg->header.stamp; //use lidar scan timestamp
        // header.stamp = msg-> header.stamp + ros::Duration(0.025); //use lidar scan timestamp + some small delta
        header.frame_id = "map";  // Set frame ID
        sensor_msgs::PointCloud2 rosPointCloud = convertEigenToROS(scan2_in_scan1_frame, header);
        aligned_pointcloud_pub_.publish(rosPointCloud);

        auto after1 = std::chrono::system_clock::now();
        auto after1Ms = std::chrono::time_point_cast<std::chrono::milliseconds>(after1);
        auto elapsedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(after1Ms - beforeMs).count();
        std::cout << "Registered scans in: " << elapsedTimeMs << " ms" << std::endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher aligned_pointcloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pcl_cloud_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformBroadcaster broadcaster_;

    //init variable to hold cumulative homogenous transform
    Eigen::Matrix4f X_homo = Eigen::Matrix4f::Identity(); 


    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
    }

    sensor_msgs::PointCloud2 convertEigenToROS(const Eigen::MatrixXf& eigenPointCloud, const std_msgs::Header& header) {
    pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
        // Assuming each row of the Eigen matrix represents a point (x, y, z)
        for (int i = 0; i < eigenPointCloud.rows(); ++i) {
            pcl::PointXYZ point;
            point.x = eigenPointCloud(i, 0);
            point.y = eigenPointCloud(i, 1);
            point.z = eigenPointCloud(i, 2);
            pclPointCloud.push_back(point);
        }

        // Convert PCL point cloud to ROS message
        sensor_msgs::PointCloud2 rosPointCloud;
        pcl::toROSMsg(pclPointCloud, rosPointCloud);
        rosPointCloud.header = header;  // Set the header from the input

        return rosPointCloud;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_map_maker_node");
    MapMakerNode simple_map_maker_node;
    ros::spin();
    return 0;
}
