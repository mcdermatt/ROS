#!/usr/bin/env python3

import rospy # import rospy, the package that lets us use ros in python. 
from std_msgs.msg import String, Bool # A simple message type, String that just wraps the primitive type string (an array of characters)
from geometry_msgs.msg import Twist
import numpy as np
from turtlesim.msg import Pose

def main():
    goal_publisher = rospy.Publisher('goal_pose', Pose, queue_size=10)
    rospy.init_node('director', anonymous=True) 
    rate = rospy.Rate(10) # 10 hz

    ready_subscriber = rospy.Subscriber("ready", Bool, callback)
    
    while not rospy.is_shutdown(): # While there's a roscore
        
        #check to see if the robot is ready for a new command (i.e. has reached the last goal)

        # Create the vel message
        vel_msg = Twist()
        speed = float(input("enter speed:"))
        distance = float(input("enter distance:"))
    
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = np.pi*np.random.randn()

        t0 = rospy.Time.now().to_sec()
        current_distance = 0.0

        print("current_distance", current_distance, type(current_distance))
        print("distance", distance, type(distance))

        #bang-bang controller for simple robot control (laughs in ME-181)
        vel_msg.linear.x = abs(speed)
        while (current_distance < distance):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = speed*(t1-t0)

        #stop the robot after reaching (or overshooting) goal pose
        vel_msg.linear.x = 0.0
        velocity_publisher.publish(vel_msg)
        
        # Publish the string message and log it (optional)
        rospy.loginfo(vel_msg)
        velocity_publisher.publish(vel_msg)
        
        # sleep so we don't publish way too many messages per second. This also gives us a chance to process incoming messages (but this node has no incoming messages)
        rate.sleep()


def callback(data):
    #need to use callback function to act on subscriber input
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


if __name__ == '__main__': # this runs when the file is run
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass