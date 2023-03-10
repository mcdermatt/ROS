#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/LaserScanStamped.h>

class LaserScanNode
{
public:
    LaserScanNode()
    {
        // Set up the ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the Gazebo LaserScanStamped messages
        laser_sub_ = nh_.subscribe("/gazebo_ros_laser_scan", 1, &LaserScanNode::laserCallback, this);

        // Advertise the LaserScan messages on a ROS topic
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    }

private:
    void laserCallback(const gazebo_msgs::LaserScanStamped::ConstPtr& msg)
    {
        // Convert the Gazebo LaserScanStamped message to a ROS LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header = msg->header;
        scan.angle_min = msg->scan.angle_min;
        scan.angle_max = msg->scan.angle_max;
        scan.angle_increment = msg->scan.angle_increment;
        scan.time_increment = msg->scan.time_increment;
        scan.scan_time = msg->scan.scan_time;
        scan.range_min = msg->scan.range_min;
        scan.range_max = msg->scan.range_max;
        scan.ranges = msg->scan.ranges;
        scan.intensities = msg->scan.intensities;

        // Publish the ROS LaserScan message
        scan_pub_.publish(scan);
    }

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher scan_pub_;
};

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "laser_scan_node");

    // Create an instance of the LaserScanNode class
    LaserScanNode node;

    // Spin the ROS node
    ros::spin();

    return 0;
}
