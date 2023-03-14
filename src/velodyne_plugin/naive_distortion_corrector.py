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
		memlim = 2*1024
		tensorflow.config.experimental.set_virtual_device_configuration(gpus[0], [tensorflow.config.experimental.VirtualDeviceConfiguration(memory_limit=memlim)])
	except RuntimeError as e:
		print(e)
#--------------------------------------------------------------------------------------

class DistortionCorrector:
	'''Takes in raw point clouds and attempts to rectify them with a linear velocity assumption'''

	def __init__(self, raw_pc_topic='/raw_point_cloud'):

		rospy.init_node('linear_distortion_corrector', anonymous=False)

		self.pcSub = rospy.Subscriber(raw_pc_topic, PointCloud2, self.on_point_cloud, queue_size = 1)
		self.vel_estimate_sub = rospy.Subscriber('/velodyne_base_vel_setpoint', Twist, self.on_linear_vel_estimate, queue_size = 1)
		self.link_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.on_link_states, queue_size = 1) #buff_size = 10)
		self.pcPub = rospy.Publisher('/rectified_point_cloud', PointCloud2, queue_size = 1)
		r = 1000
		self.rate = rospy.Rate(r)

		self.liner_vel_estimate = np.zeros([6])

	def on_point_cloud(self, scan):
		"""callback function for when node recieves raw point cloud"""

		# need to hold on to linear velocity estimate and sensor body frame poses
		#		at the exact time point cloud data comes in- don't change these values
		#		until we get the next raw point cloud
		vel = self.liner_vel_estimate
		base_rot_euls = self.velodyne_euls_base

		#convert cloud msg to np array
		gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
		xyz = []
		for p in gen:
			xyz.append(p)
		self.pc_xyz = np.array(xyz)

		#remove inf values
		# self.pc_xyz = self.pc_xyz[self.pc_xyz[:,0] < 10_000]
		# print(self.pc_xyz)

		#point cloud already aligned with base frame of LIDAR sensor(?)  

		#convert to spherical coordinates
		self.pc_spherical = self.c2s(self.pc_xyz).numpy()

		#sort by azim angle
		self.pc_xyz = self.pc_xyz[np.argsort(self.pc_spherical[:,1])]

		#TODO: uncurl initial point cloud 
		#		this is something that motion profile does not have baked in automatically

		#Linear velocity model -> assume velocity distortion is directly proportional to each point's theta (yaw) angle 
		motion_profile = np.linspace(0, 1, len(self.pc_xyz))[:,None] @ vel[None,:]
		print(vel)
		undistorted_pc = self.remove_motion_distortion(self.pc_xyz, motion_profile)
		self.pcPub.publish(point_cloud(undistorted_pc, 'map'))
		# print(undistorted_pc)


	def remove_motion_distortion(self, points, motion_profile):
		"""
		Removes motion distortion from 3D LIDAR data.

		Args:
		- points: numpy array of shape (N, 3) containing 3D LIDAR points
		- motion_profile: numpy array of shape (N, 6) containing the motion profile of the LIDAR device during data acquisition

		Returns:
		- corrected_points: numpy array of shape (N, 3) containing the motion distortion corrected 3D LIDAR points
		"""
		# Convert motion profile to homogeneous transformation matrices
		T = []
		for i in range(len(motion_profile)):
			tx, ty, tz, roll, pitch, yaw = motion_profile[i]
			R = np.dot(np.dot(np.array([[1, 0, 0], 
										[0, np.cos(roll), -np.sin(roll)], 
										[0, np.sin(roll), np.cos(roll)]]), 
							np.array([[np.cos(pitch), 0, np.sin(pitch)], 
									  [0, 1, 0], 
									  [-np.sin(pitch), 0, np.cos(pitch)]])), 
							np.array([[np.cos(yaw), -np.sin(yaw), 0], 
									  [np.sin(yaw), np.cos(yaw), 0], 
									  [0, 0, 1]]))
			T.append(np.concatenate((np.concatenate((R, np.array([[tx], [ty], [tz]])), axis=1), np.array([[0, 0, 0, 1]])), axis=0))
		
		# Apply inverse of motion transformation to each point
		corrected_points = np.zeros_like(points)
		for i in range(len(points)):
			point = np.concatenate((points[i], np.array([1])))
			T_inv = np.linalg.inv(T[i])
			corrected_point = np.dot(T_inv, point)[:3]
			corrected_points[i] = corrected_point
		
		return corrected_points


	def on_linear_vel_estimate(self, t):
		"""callback func for when node recieves linear velocity estimate from SensorMover"""
		vel_pub_rate = 1000
		self.liner_vel_estimate = -np.array([t.linear.x, t.linear.y, t.linear.z,
											t.angular.x, t.angular.y, t.angular.z]) * vel_pub_rate
		# print(self.liner_vel_estimate)


	def on_link_states(self, link_states):
		"""callback for getting link frames out of Gazebo"""

		vq_base = link_states.pose[1].orientation
		velodyne_quat_base = [vq_base.x, vq_base.y, vq_base.z, vq_base.w]
		velodyne_quat_base = R.from_quat(velodyne_quat_base)
		self.velodyne_euls_base = velodyne_quat_base.as_euler('xyz')

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

	dc = DistortionCorrector()

	while not rospy.is_shutdown():
		rospy.spin()