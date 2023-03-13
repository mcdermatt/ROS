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


#TODO:
#		chose scan angle theta to start point cloud
#		Make base links of velodyne RGB for XYZ

class CloudMaker():
	'''takes in sequential laser scans and generates a point cloud '''

	def __init__(self, scan_line_topic='/my_velodyne/top/sensor/scan'):

		rospy.init_node('cloudmaker', anonymous=False)

		self.scan_line_topic = scan_line_topic
		self.max_range = 100 #(m)

		#init subscriber for scan lines 
		self.scan_line_sub = rospy.Subscriber(self.scan_line_topic, LaserScan, self.on_scan_line, queue_size = 1) #buff_size = 10)
		# subscriber for poses and velocities of each link in model
		self.link_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.on_link_states, queue_size = 1) #buff_size = 10)
		#point cloud publisher
		self.pcPub = rospy.Publisher('static_point_cloud', PointCloud2, queue_size = 1)

		# self.cloud_i = np.zeros([0,3]) #debug
		self.init_scan()
		# self.last_rot = -np.pi #debug
		self.just_published = False #debug

		self.rotation_angle_noise_scale = np.deg2rad(0.5)

		r = 1000
		self.rate = rospy.Rate(r)

	def init_scan(self):
		"""initialize new point cloud after complete rotation of sensor"""

		self.cloud_i = np.zeros([0,3])
		self.scan_line_cart  = np.zeros([0,3])
		self.count = 0

		#start/stop scans w.r.t. world frame
		# self.last_rot = -np.pi 
		self.just_published = True


	def on_scan_line(self, scan_line):

		# print("\n count:", self.count)

		#get max and min phi angles from scan line 
		# print(scan_line)
		self.phi_stepsize = scan_line.angle_increment 

		#get points in spherical coordinates
		ranges = np.array([scan_line.ranges]).T[:,0]
		# ranges = ranges[np.abs(ranges) < self.max_range] #remove points farther than cutoff
		thetas = np.ones([len(ranges)])*self.velodyne_euls_top[2] #no noise in rotation
		# thetas = np.ones([len(ranges)])*self.velodyne_euls[2] + np.random.randn()*self.rotation_angle_noise_scale
		phis = np.pi/2 - np.linspace(scan_line.angle_min, scan_line.angle_max, len(ranges))

		# print(self.velodyne_euls[2])

		scan_line_spherical = np.array([ranges, thetas, phis]).T #was this (wrong??)
		# scan_line_spherical = np.array([ranges, phis, thetas]).T
		# print(np.shape(scan_line_spherical))
		# print(scan_line_spherical)

		#convert spherical to cartesian
		self.scan_line_cart = self.s2c(scan_line_spherical).numpy()
		# print(np.shape(self.scan_line_cart))
		# print(self.scan_line_cart)

		#append euler points cloud_i
		self.cloud_i = np.append(self.cloud_i, self.scan_line_cart, axis = 0)
		# print(np.shape(self.cloud_i))

	def on_link_states(self, link_states):
		#get configuration of LIDAR sensor 

		# print(link_states.name)
		# print(link_states.pose[2].orientation.z)
		# print(link_states.twist[2].angular.z) #gets actual rotational velocity of velodyne sensor

		#TODO: compare against prescribed velodyne setpoint published in: 
		# <rostopic pub /my_velodyne/vel_cmd std_msgs/Float32 1.0>
		
		vq_top = link_states.pose[2].orientation
		velodyne_quat_top = [vq_top.x, vq_top.y, vq_top.z, vq_top.w]
		velodyne_quat_top = R.from_quat(velodyne_quat_top)
		self.velodyne_euls_top = velodyne_quat_top.as_euler('xyz')

		vq_base = link_states.pose[1].orientation
		velodyne_quat_base = [vq_base.x, vq_base.y, vq_base.z, vq_base.w]
		velodyne_quat_base = R.from_quat(velodyne_quat_base)
		self.velodyne_euls_base = velodyne_quat_base.as_euler('xyz')

		#check for finishing a scan
		#w.r.t. world frame:
		#check to see if we've done a full rotatin (3.14 -> -3.14)
		# if self.velodyne_euls_top[2] < self.last_rot:
		#w.r.t. base frame
		#check to see if current rotation and last pose of top are on either side of bottom 
		if self.velodyne_euls_top[2] > self.velodyne_euls_base[2] and self.last_rot < self.velodyne_euls_base[2] and not self.just_published:
			print("publishing scan")
			print("\n base:",self.velodyne_euls_base[2]) #debug
			print("top:",self.velodyne_euls_top[2]) #debug
			print("last:", self.last_rot)
			#publish full point cloud (slow??)
			self.pcPub.publish(point_cloud(self.cloud_i, 'map'))
			# #downsample and publish
			# downsampled_cloud = self.cloud_i[np.random.choice(len(self.cloud_i), size = 100_000)]  
			# self.pcPub.publish(point_cloud(downsampled_cloud, 'map'))

			self.last_rot = self.velodyne_euls_base[2] #test

			self.init_scan()

		else:
			#update incomplete point cloud
			self.last_rot = self.velodyne_euls_top[2]
			self.count += 1
			self.just_published = False
			# print(self.count)

	def c2s(self, pts):
		""" converts points from cartesian coordinates to spherical coordinates """
		r = tf.sqrt(pts[:,0]**2 + pts[:,1]**2 + pts[:,2]**2)
		phi = tf.math.acos(pts[:,2]/r)
		theta = tf.math.atan2(pts[:,1], pts[:,0])

		out = tf.transpose(tf.Variable([r, theta, phi]))
		return(out)

	def s2c(self, pts):
		"""converts spherical -> cartesian"""

		x = pts[:,0]*tf.math.sin(pts[:,2])*tf.math.cos(pts[:,1])
		y = pts[:,0]*tf.math.sin(pts[:,2])*tf.math.sin(pts[:,1]) 
		z = pts[:,0]*tf.math.cos(pts[:,2])

		out = tf.transpose(tf.Variable([x, y, z]))
		# out = tf.Variable([x, y, z])
		return(out)

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

	cm = CloudMaker()

	while not rospy.is_shutdown():
		rospy.spin()