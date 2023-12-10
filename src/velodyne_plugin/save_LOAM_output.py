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
from time import sleep
from scipy.spatial.transform import Rotation as R

import sys
import tf_conversions
import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import numpy as np

import tf_conversions
import tf2_ros

## play 05: Quad With Dynamics:
# rosbag play -l -r 0.2 rooster_2020-07-10-09-16-39_1.bag
## play one lap of running around the quad starting from static position
# rosbag play -l -r 0.05 --clock -s 35.1 rooster_2020-07-10-09-16-39_1.bag

#start at very beginning, first onset of motion (280)
#rosbag play -l -r 0.05 --clock -s 28  rooster_2020-07-10-09-13-52_0-001.bag

#start at 1800
# rosbag play -l -r 0.05 --clock -s 12 rooster_2020-07-10-09-16-39_1.bag
#start  at 1790
# rosbag play -l -r 0.05 --clock -s 11 rooster_2020-07-10-09-16-39_1.bag


## play 06: Dynamic Spinning:
# rosbag play -l -r 0.1  rosbag play rooster_2020-07-10-09-23-18_0.bag
## start midway through trial to have LOAM initialize on distorted scene
# rosbag play -r 0.05 --clock -s 95 rooster_2020-07-10-09-23-18_0.bag


class LOAMSaver():
  ''' Saves outputs of LOAM to text file '''

  # odom_topic = '/laser_odom_to_init' #old, starts to drift off a lot -- (05_test1)
  def __init__(self, odom_topic = '/integrated_to_init'):

    rospy.init_node('LOAM_Saver', anonymous=False)

    self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size = 10) #buff_size = 10)
    self.scan_sub = rospy.Subscriber('/velodyne_cloud_3', PointCloud2, self.on_scan, queue_size = 10)

    r = 100
    self.rate = rospy.Rate(r)

    self.save_traj = True
    # self.save_traj = False
    # self.save_clouds = True
    self.save_clouds = False

    self.file_dir = "/home/derm/ASAR/v3/point_cloud_rectification/results/LOAM/"
    self.traj_idx = "05_LOAM_start_from_static_v5"
    # self.traj_idx = '06_LOAM_dynamic_start_v7'
    # self.traj_idx = '05_LOAM_start_from_2800_v1'
    # self.traj_idx = '05_LOAM_start_from_2700_v1'
    # self.traj_idx = '05_LOAM_start_from_1800_v2'
    # self.traj_idx = '05_LOAM_start_from_1790_v1'
    # self.traj_idx = "05_LOAM_start_from_280_v1"

    self.reset()

  def reset(self):
    '''resets node at beginning of trajectory'''
    self.traj = np.zeros([0,6])

    self.count = 0


  def on_odom(self, data):

    TF_to_init = data.pose
    # print(TF_to_init)
    x = TF_to_init.pose.position.x
    y = TF_to_init.pose.position.y
    z = TF_to_init.pose.position.z

    orientation = TF_to_init.pose.orientation
    quats = [orientation.x, orientation.y, orientation.z, orientation.w]
    euls = R.from_quat(quats).as_euler('xyz')

    #save resulting trajectory to numpy file
    fn_traj = self.file_dir + self.traj_idx #+ str(self.count)
    print("\n Scan", self.count, "\n xyz:", x, y, z, "\n phi, theta, psi:", euls)
    print(fn_traj)
    new_pose = np.array([[x, y, z, euls[0], euls[1], euls[2]]])
    self.traj = np.append(self.traj, new_pose, axis = 0)
    # print(self.traj)
    if self.save_traj:
      np.save(fn_traj, self.traj)

    self.count += 1

  def on_scan(self, scan):
    """save undistorted cloud as np array when recieving /velodyne_cloud_3"""

    if self.save_clouds:

      #convert new point cloud msg to np array
      gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
      xyz = []
      for p in gen:
          xyz.append(p)
      pc_xyz = np.array(xyz)

      fn_pc = self.file_dir + self.traj_idx +  "_undistorted_PC_" + str(self.count)
      print("saving undistorted point cloud to:", fn_pc)

      np.save(fn_pc, pc_xyz)

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

  LS = LOAMSaver()

  while not rospy.is_shutdown():
    rospy.spin()


