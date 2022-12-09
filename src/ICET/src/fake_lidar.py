#!/usr/bin/env python3
import rospy # import rospy, the package that lets us use ros in python. 
from ICET.msg import Num #using custom message type
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import pandas as pd

import std_msgs.msg as std_msgs
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

import trimesh #when using KITTI_CARLA dataset

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
    # pcNpyPub = rospy.Publisher('numpy_cloud', numpy_msg(Floats), queue_size = 1)

    # traditional pointcloud2 msg
    pcPub = rospy.Publisher('point_cloud', PointCloud2, queue_size = 10)

    # publish custom message with additional LIDAR info
    etcPub = rospy.Publisher('lidar_info', Num, queue_size=10) #This publisher can hold 10 msgs that haven't been sent yet.  
    rospy.init_node('LidarScanner', anonymous=True)
    r = 10 #real time (too fast to work at full resolution)
    # r = 5 #slower real time (some sensors run at 5Hz)
    # r = 1
    rate = rospy.Rate(r) # hz
    
    while not rospy.is_shutdown(): # While there's a roscore

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        # # use simulated point clouds I generated for a previous publication 
        # # https://github.com/mcdermatt/ASAR/tree/main/v3/spherical_paper/MC_trajectories
        # idx = int(r*rospy.get_time()%40) + 1 #use ROS timestamp as seed for scan idx
        # print(idx)
        # fn = "/home/derm/ASAR/v3/spherical_paper/MC_trajectories/scene1_scan" + str(idx) + ".txt"
        # pcNpy = np.loadtxt(fn)
        # # pcNpy = pcNpy[pcNpy[:,2] > -1.5] #debug

        # #use KITTI_CARLA synthetic LIDAR data
        # idx = int(r*rospy.get_time()%400) + 2300 #use ROS timestamp as seed for scan idx
        # fn = '/home/derm/KITTICARLA/dataset/Town01/generated/frames/frame_%04d.ply' %(idx)
        # dat1 = trimesh.load(fn)
        # pcNpy = dat1.vertices
        # # pcNpy = pcNpy[pcNpy[:,2] > -1.5] #debug

        #use Ouster sample dataset (from high fidelity 128-channel sensor!)
        idx = int(r*rospy.get_time()%700) + 1 #use ROS timestamp as seed for scan idx
        fn1 = "/media/derm/06EF-127D2/Ouster/csv/pcap_out_" + '%06d.csv' %(idx)
        print("Publishing", fn1)
        df1 = pd.read_csv(fn1, sep=',', skiprows=[0])
        pcNpy = df1.values[:,8:11]*0.001 #1st sensor return
        # pcNpy = df1.values[:,11:14]*0.001 #2nd sensor return

        #publish point cloud as numpy_msg (rospy)
        # pcNpyPub.publish(pcNpy)

        #publish point cloud as point_cloud2 message (better?)
        pcPub.publish(point_cloud(pcNpy, 'map'))
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        # publish custom <Num> message type~~~~~~~~~~~~~~~~~
        msg = Num()
        msg.frame = idx
        if idx == 1:
            msg.restart = True
        else:
            msg.restart = False

        #have prior knowledge of ground truth for simulated dummy data
        msg.true_transform = [0.5, 0.0, 0.0, 0.0, 0.0, 0.00] #[x, y, z, r, p, y]

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