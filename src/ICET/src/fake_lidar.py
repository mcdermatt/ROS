#!/usr/bin/env python3
import rospy # import rospy, the package that lets us use ros in python. 
from ICET.msg import Num #using custom message type
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import std_msgs.msg as std_msgs
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

"""script to publish custom LIDAR point cloud messages"""

#TODO: 
#       make publishing a service 
#       -> wait until listener is done registering previous cloud to send next

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx3 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyzrgba')]

    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main():

    # publish numpy array 
    pcNpyPub = rospy.Publisher('numpy_cloud', numpy_msg(Floats), queue_size = 1)

    # traditional pointcloud2 msg
    pcPub = rospy.Publisher('point_cloud', PointCloud2, queue_size = 1)

    # publish custom message with additional LIDAR info
    etcPub = rospy.Publisher('lidar_info', Num, queue_size=10) #This publisher can hold 10 msgs that haven't been sent yet.  
    rospy.init_node('LidarScanner', anonymous=True)
    r = 10
    rate = rospy.Rate(r) # hz
    
    while not rospy.is_shutdown(): # While there's a roscore

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # use simulated point clouds I generated for a previous publication 
        # https://github.com/mcdermatt/ASAR/tree/main/v3/spherical_paper/MC_trajectories
        
        #use ROS timestamp as seed for scan idx
        idx = int(r*rospy.get_time()%40) + 1
        fn = "/home/derm/ASAR/v3/spherical_paper/MC_trajectories/scene1_scan" + str(idx) + ".txt"
        pcNpy = np.loadtxt(fn)

        #publish point cloud as numpy_msg (rospy)
        pcNpyPub.publish(pcNpy)

        #publish again as true point cloud message (so I can view with rviz)
        pcPub.publish(point_cloud(pcNpy, 'map'))
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
        # rospy.loginfo(msg)
        etcPub.publish(msg)
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        # sleep so we don't publish way too many messages per second. This also gives us a chance to process incoming messages (but this node has no incoming messages)
        rate.sleep()

if __name__ == '__main__': # this runs when the file is run
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass