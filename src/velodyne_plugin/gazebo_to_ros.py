#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates, ModelState
from sensor_msgs.msg import LaserScan

def callback(data):
    laser_scan = LaserScan()
    laser_scan.header = data.header
    laser_scan.angle_min = data.angle_min
    laser_scan.angle_max = data.angle_max
    laser_scan.angle_increment = data.angle_increment
    laser_scan.time_increment = data.time_increment
    laser_scan.scan_time = data.scan_time
    laser_scan.range_min = data.range_min
    laser_scan.range_max = data.range_max
    laser_scan.ranges = data.ranges
    laser_scan.intensities = data.intensities

    pub = rospy.Publisher('my_laser_scan', LaserScan, queue_size=10)
    pub.publish(laser_scan)

def listener():
    rospy.init_node('laser_scan_subscriber', anonymous=True)
    rospy.Subscriber('/gazebo/default/my_velodye/top/sensor/scan', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass