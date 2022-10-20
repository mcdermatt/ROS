#!/usr/bin/env python3
import rospy # import rospy, the package that lets us use ros in python. 
from ICET.msg import Num #using custom message type

def main():
    pub = rospy.Publisher('LidarScannerOutput', Num, queue_size=10) # Create a publisher that publishes String messages on a topic called "chatter". This publisher can hold 10 msgs that haven't been sent yet. 
    rospy.init_node('LidarScanner', anonymous=True) # intialize a node named "talker", anonymous=True means that a number will be appended to the name "talker" so we could run as many of these nodes as we want without their names conflicting. 
    rate = rospy.Rate(10) # 10 hz
    
    while not rospy.is_shutdown(): # While there's a roscore
        
        # Create the string message
        hello_str = "hello world %s" % rospy.get_time() # 
        msg = Num()
        msg.status = hello_str
        msg.errors = False
        msg.ch1 = 0.01
        msg.ch2 = 0.0001
        
        # Publish the string message and log it (optional)
        rospy.loginfo(msg)
        pub.publish(msg)
        
        # sleep so we don't publish way too many messages per second. This also gives us a chance to process incoming messages (but this node has no incoming messages)
        rate.sleep()

if __name__ == '__main__': # this runs when the file is run
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass