#!/usr/bin/env python3
import rospy # import rospy, the package that lets us use ros in python. 
from ICET.msg import Num #using custom message type
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

"""script to publish custom LIDAR point cloud messages"""

def main():

    pcPub = rospy.Publisher('point_cloud', numpy_msg(Floats))
    etcPub = rospy.Publisher('lidar_info', Num, queue_size=10) # Create a publisher that publishes String messages on a topic called "chatter". This publisher can hold 10 msgs that haven't been sent yet. 
    rospy.init_node('LidarScanner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown(): # While there's a roscore
        

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # use simulated point clouds I generated for a previous publication 
        # https://github.com/mcdermatt/ASAR/tree/main/v3/spherical_paper/MC_trajectories
        
        #use ROS timestamp as seed for scan idx
        idx = int(rospy.get_time()%40) + 1
        fn = "ASAR/v3/spherical_paper/MC_trajectories/scene1_scan" + str(idx) + ".txt"
        pc = np.loadtxt(fn)

        #publish point cloud as numpy_msg (rospy)
        pcPub.publish(pc)
        rospy.loginfo(pc)
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        #using custom <Num> message type~~~~~~~~~~~~~~~~~~~
        msg = Num()
        msg.frame = idx
        if idx == 1:
            msg.restart = True
        else:
            msg.restart = False

        #have prior knowledge of ground truth for simulated dummy data
        msg.true_transform = [0.5, 0.0, 0.0, 0.0, 0.0, 0.05] #[x, y, z, r, p, y]

        status_str =  "Frame # " + str(idx) + " Lidar Timestamp %s" % rospy.get_time() # 
        msg.status = status_str

        # Publish the string message and log it (optional)
        rospy.loginfo(msg)
        etcPub.publish(msg)
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        # sleep so we don't publish way too many messages per second. This also gives us a chance to process incoming messages (but this node has no incoming messages)
        rate.sleep()

if __name__ == '__main__': # this runs when the file is run
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass