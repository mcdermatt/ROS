#!/usr/bin/env python3
# license removed for brevity
import rospy # import rospy, the package that lets us use ros in python. 
from std_msgs.msg import String # A simple message type, String that just wraps the primitive type string (an array of characters)
from geometry_msgs.msg import Twist
import numpy as np

def main():
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) #
    rospy.init_node('turtle_mover', anonymous=True) 
    rate = rospy.Rate(10) # 10 hz
    
    while not rospy.is_shutdown(): # While there's a roscore
        
        # Create the vel message
        vel_msg = Twist()
        speed = float(input("enter speed:"))
        distance = float(input("enter distance:"))

        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.z = np.random.randn()
        vel_msg.angular.y = 0.0
        vel_msg.angular.x = 0.0

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

if __name__ == '__main__': # this runs when the file is run
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass