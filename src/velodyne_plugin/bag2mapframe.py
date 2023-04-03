#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg as std_msgs
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from gazebo_msgs.msg import LinkStates
import sensor_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import tensorflow #as tf #pretty dumb naming convention smh
from time import sleep
from scipy.spatial.transform import Rotation as R

import sys
import tensorflow as tf 
import tf_conversions
import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np

import tf_conversions
import tf2_ros

#limit GPU memory ---------------------------------------------------------------------
# if you don't include this TensorFlow WILL eat up all your VRAM and make rviz run poorly
gpus = tensorflow.config.experimental.list_physical_devices('GPU')
print(gpus)
if gpus:
  try:
    memlim = 4*1024
    tensorflow.config.experimental.set_virtual_device_configuration(gpus[0], [tensorflow.config.experimental.VirtualDeviceConfiguration(memory_limit=memlim)])
  except RuntimeError as e:
    print(e)
#--------------------------------------------------------------------------------------

# cd Downloads
# rosbag play rooster_2020-07-10-09-13-52_0-001.bag

class BagConverter():

  def __init__(self, point_cloud_topic='/os1_cloud_node/points'):

    rospy.init_node('bagconverter', anonymous=False)
    self.pcSub = rospy.Subscriber(point_cloud_topic, PointCloud2, self.on_bag_point_cloud, queue_size = 1)
    self.pcPub = rospy.Publisher('remapped_outster_cloud', PointCloud2, queue_size = 1)

    r = 1_000
    self.rate = rospy.Rate(r)

  def on_bag_point_cloud(self, scan):

    #convert cloud msg to np array
    gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
    xyz = []
    for p in gen:
      xyz.append(p)
    pc_xyz = np.array(xyz)
    # print(pc_xyz)

    self.pcPub.publish(point_cloud(pc_xyz, 'map'))



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

  bc = BagConverter()

  while not rospy.is_shutdown():
    rospy.spin()