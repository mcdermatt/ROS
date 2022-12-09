#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from ICET.msg import Num #custom message type
import std_msgs.msg as std_msgs
from rospy.numpy_msg import numpy_msg
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from laser_geometry import LaserProjection
import tensorflow as tf

#limit GPU memory ------------------------------------------------
# if I don't include this TensorFlow will eat up all the VRAM
gpus = tf.config.experimental.list_physical_devices('GPU')
print(gpus)
if gpus:
  try:
    memlim = 4*1024
    tf.config.experimental.set_virtual_device_configuration(gpus[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=memlim)])
  except RuntimeError as e:
    print(e)
#-----------------------------------------------------------------

from ICET_spherical import ICET #my point cloud registration algorithm


class ScanMatcher():
    """scanmatcher node subscribes to /point_cloud topic and attemts to 
        register sequential point clouds. Outputs 6dof transformation """
    
    def __init__(self, scan_topic="point_cloud"):
        # self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()

        rospy.init_node('scanmatcher', anonymous=True)

        self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)

        self.etc_sub = rospy.Subscriber('lidar_info', Num, self.get_info)

        #TODO: publish this as a Numpy message...
        # [x, y, z, phi, theta, psi, idx_keyframe, idx_newframe]
        self.TPub = rospy.Publisher('Transform', numpy_msg(Floats), queue_size = 1)
        # self.TPub = rospy.Publisher('map', PointCloud2, queue_size = 10)

        r = 10
        self.rate = rospy.Rate(r)

        self.keyframe_scan = None #init scans1 and 2
        self.new_scan = None
        self.x0 = tf.constant([0., 0., 0., 0., 0., 0.])

    def get_info(self, data):
        """ Gets Lidar info from custom Num msg """
        self.scan_data = data

        #TODO-> make this a function
        #reset if data frame is 0 
        if self.scan_data.frame == 0:
            self.keyframe_scan = None
            self.new_scan = None
            self.x0 = tf.constant([0., 0., 0., 0., 0., 0.])

            #TODO- save to disk overall trajectory at the end??

    def on_scan(self, scan):
        """Finds the transformation between current and previous frames"""

        #convert new point cloud msg to np array
        gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
        xyz = []
        for p in gen:
            xyz.append(p)
        self.pc_xyz = np.array(xyz)
        # print("newscan_xyz", np.shape(self.pc_xyz)) #fixed size for Ouster dataset

        #init
        if self.keyframe_scan is not None:
            if self.new_scan is None:
                self.new_scan = self.pc_xyz
                self.new_scan_idx = self.scan_data.frame
                print("new_scan_idx = ", self.new_scan_idx, "\n")
        else:
            self.keyframe_scan = self.pc_xyz
            self.keyframe_idx = self.scan_data.frame
            print("keyframe_idx = ", self.keyframe_idx)

        if self.keyframe_scan is not None and self.new_scan is not None:

            it = ICET(cloud1 = self.keyframe_scan, cloud2 = self.new_scan, fid = 50, niter = 5, 
                draw = False, group = 2, RM = False, DNN_filter = False, x0 = self.x0)

            self.X = it.X
            self.pred_stds = it.pred_stds 
            print("\n X=", self.X)
            print("\n pred_stds=", self.pred_stds)

            self.x0 = self.X

            print("\n keyframe idx = ", self.keyframe_idx)
            print("new_scan idx = ", self.new_scan_idx)
            self.keyframe_scan = self.new_scan
            self.keyframe_idx = self.new_scan_idx
            self.new_scan = self.pc_xyz
            self.new_scan_idx = self.scan_data.frame

if __name__ == '__main__':
    m = ScanMatcher()

    while not rospy.is_shutdown():

        rospy.spin()