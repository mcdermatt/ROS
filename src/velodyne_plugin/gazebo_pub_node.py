#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def callback(msg):
    # log the message data to the terminal
    rospy.loginfo("The value is %d", msg.data)
    print("recieved data from Gazebo")


def listener():
    # start the node
    rospy.init_node("listener")
    # subscribe to the '/sr_tactile/touch/ff' topic
    # tactile_sub = rospy.Subscriber("/sr_tactile/touch/ff", Float64, callback)
    pc_sub = rospy.Subscriber("/gazebo/default/my_velodyne/top/sensor/scan", Float64, callback)
    # keep python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()