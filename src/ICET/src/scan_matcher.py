#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from ICET.msg import Num #custom message type
import std_msgs.msg as std_msgs
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from laser_geometry import LaserProjection
import tensorflow as tf

#limit GPU memory ---------------------------------------------------------------------
# if you don't include this TensorFlow WILL eat up all your VRAM and make rviz run poorly
gpus = tf.config.experimental.list_physical_devices('GPU')
print(gpus)
if gpus:
  try:
    memlim = 4*1024
    tf.config.experimental.set_virtual_device_configuration(gpus[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=memlim)])
  except RuntimeError as e:
    print(e)
#--------------------------------------------------------------------------------------

from ICET_spherical import ICET #my point cloud registration algorithm


class ScanMatcher():
    """scanmatcher node subscribes to /point_cloud topic and attemts to 
        register sequential point clouds. 

        ICET can only run at ~2-3 Hz so it will miss some point cloud frames

        Publishes 6dof transformation with associated frame IDs"""
    
    def __init__(self, scan_topic="raw_point_cloud"):
        # self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.on_scan)
        self.laser_projector = LaserProjection()

        rospy.init_node('scanmatcher', anonymous=True)

        self.scan_sub = rospy.Subscriber(scan_topic, PointCloud2, self.on_scan)
        self.etc_sub = rospy.Subscriber('lidar_info', Num, self.get_info)
        self.TPub = rospy.Publisher('relative_transform', Floats, queue_size = 10)
        self.SigmaPub = rospy.Publisher('relative_covariance', Floats, queue_size = 10)
        #for publishing corrected point clouds with moving objects removed
        self.pcPub = rospy.Publisher('static_point_cloud', PointCloud2, queue_size = 10)

        r = 10 #not going to be able to actually run this fast, but no harm in setting at 10 Hz
        self.rate = rospy.Rate(r)

        self.reset()

    def reset(self):
        self.keyframe_scan = None #init scans1 and 2
        self.new_scan = None
        self.x0 = tf.constant([0., 0., 0., 0., 0., 0.])

    def get_info(self, data):
        """ Gets Lidar info from custom Num msg """
        self.scan_data = data

        if self.scan_data.frame == 0:
            self.reset()
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

            it = ICET(cloud1 = self.keyframe_scan, cloud2 = self.new_scan, fid = 70, niter = 5, 
                draw = False, group = 2, RM = True, DNN_filter = False, x0 = self.x0)

            self.X = it.X
            self.pred_stds = it.pred_stds 
            self.new_scan_static_points = it.cloud2_static  #hold on to non-moving points

            # self.x0 = self.X    #set inital conditions for next registration 

            #publish estimated local transform and error covariance
            # [x, y, z, phi, theta, psi, idx_keyframe, idx_newframe]
            T_msg = np.append(self.X.numpy(), (self.keyframe_idx, self.new_scan_idx)) 
                #produces very weired bug when publishing np_msg -> just use Floats
            print("\n T_msg", T_msg)
            self.TPub.publish(T_msg)
            sigma_msg = np.append(self.pred_stds, (self.keyframe_idx, self.new_scan_idx))
            print("\n sigma_msg", sigma_msg)
            self.SigmaPub.publish(sigma_msg)

            #publish non-moving points in new scan
            #   (overridden by self.keyframe_scan if ICET isn't running moving object suppression)
            self.pcPub.publish(point_cloud(it.cloud2_static, 'map'))

            print("\n keyframe idx = ", self.keyframe_idx)
            print("new_scan idx = ", self.new_scan_idx)
            self.keyframe_scan = self.new_scan
            self.keyframe_idx = self.new_scan_idx
            self.new_scan = self.pc_xyz
            self.new_scan_idx = self.scan_data.frame

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

if __name__ == '__main__':
    m = ScanMatcher()

    while not rospy.is_shutdown():

        rospy.spin()