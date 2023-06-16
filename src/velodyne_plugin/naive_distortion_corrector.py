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
from std_msgs.msg import Float32

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
		self.vel_estimate_sub = rospy.Subscriber('/velodyne_base_vel_setpoint', Twist, self.base_vel_setpoint, queue_size = 10)
		self.lidar_rotation_rate_sub = rospy.Subscriber('/my_velodyne/vel_cmd', Float32, self.on_lidar_vel_cmd, queue_size = 1)
		self.link_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.on_link_states, queue_size = 1) #buff_size = 10)
		self.pcPub = rospy.Publisher('/rectified_point_cloud', PointCloud2, queue_size = 1)
		r = 1000
		self.rate = rospy.Rate(r)
		self.save_data = False
		# self.save_data = True

		self.frame_idx = 0
		self.linear_vel_estimate = np.zeros([6])
		self.last_xpos = 0
		self.last_theta = 0

		#how much of scan is "lost" while point cloud is saved in </cloudmaker>
		#calculate this from cloudmaker script
		# self.buffer_width = np.deg2rad(2.5) #works(ish)
		# self.buffer_width = np.deg2rad(0.3)
		self.buffer_width = 0

		self.vel_history_for_this_frame = np.zeros([0,6])

	def on_point_cloud(self, scan):
		"""callback function for when node recieves raw point cloud"""

		if self.save_data: 
			print("\n recording frame_idx:", self.frame_idx)
			root_dir = "/home/derm/ASAR/v3/point_cloud_rectification/sample_data/"
			# trajectory_name = "test1/"
			# trajectory_name = "paper_figures/case1/" #static sensor
			# trajectory_name = "paper_figures/case2/"  #constant forward velocity
			trajectory_name = "paper_figures/case3/"  #constant forward/ lateral velocity + yaw  
			fn_vel_cmd = root_dir + trajectory_name + "base_vel_" + str(self.frame_idx)
			np.save(fn_vel_cmd, self.vel_history_for_this_frame)
			self.vel_history_for_this_frame = np.zeros([0,6])

		# # DEBUG: see how far base of LIDAR unit has actually moved during the frame~~~~~~~~~
		# delta_x = self.current_xpos - self.last_xpos
		# delta_theta = self.current_theta - self.last_theta
		# print("measured delta x:", delta_x, "\n measured delta theta:", delta_theta)
		# self.last_xpos =  self.current_xpos
		# self.last_theta = self.current_theta
		# #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		# need to hold on to linear velocity estimate and sensor body frame poses
		#		at the exact time point cloud data comes in- don't change these values
		#		until we get the next raw point cloud
		vel = self.linear_vel_estimate
		period_base = (2*np.pi)/vel[-1]
		base_rot_euls = self.velodyne_euls_base

		#test
		base_rot_quat = self.velodyne_quat_base #in scipy rot format, not actually quat (but is quat under the hood)
		print("\n base_rot_quat", base_rot_quat.as_quat())

		#convert cloud msg to np array
		gen = point_cloud2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))
		xyz = []
		for p in gen:
			xyz.append(p)
		self.pc_xyz = np.array(xyz)

		#remove inf values
		self.pc_xyz = self.pc_xyz[self.pc_xyz[:,0] < 10_000]

		#convert to spherical coordinates
		self.pc_spherical = self.c2s(self.pc_xyz).numpy()

		#Because of body frame yaw rotation, we're not always doing a full roation - we need to "uncurl" initial point cloud
		# (this is NOT baked in to motion profile)
		self.pc_spherical = self.pc_spherical[np.argsort(self.pc_spherical[:,1])] #sort by azim angle

		#get total overlap in rotation between LIDAR and base frames (since both are rotating w.r.t. world Z)
		# point of intersection = (t_intersection) * (angular velocity base)
		#						= ((n * T_a * T_b) / (T_a + T_b)) * omega_base 
		total_rot = -2*np.pi*np.sin(vel[-1]/(-vel[-1] + self.lidar_cmd_vel)) #actually should be this!
		#TODO: add width of buffer used to trigger new rotation to this value  
		total_rot += self.buffer_width
		print("total_rot:", total_rot)

		#scale linearly starting at theta = 0
		self.pc_spherical[:,1] = ((self.pc_spherical[:,1]) % (2*np.pi))*((2*np.pi - total_rot)/(2*np.pi)) + total_rot #works

		#sort by azim angle again- some points will have moved past origin in the "uncurling" process
		self.pc_spherical = self.pc_spherical[np.argsort(self.pc_spherical[:,1])] 

		#reorient
		self.pc_spherical[:,1] = ((self.pc_spherical[:,1] + np.pi) % (2*np.pi)) - np.pi
		undistorted_pc = self.s2c(self.pc_spherical).numpy() #convert back to xyz

		#align scan with linearized velocity prescribed in <motion_profile>
		# rot_vec = R.from_euler('xyz', base_rot_euls).as_dcm() #was this -> relies on converting to euls
		rot_vec =  base_rot_quat.as_dcm() #keeps in scipy rot format as long as possible -> should prevent large angle errors

		# #Linear velocity model -> assume velocity distortion is directly proportional to each point's theta/ yaw angle 
		self.pc_xyz = undistorted_pc
		rectified_vel  = -vel[None,:]
		rectified_vel[0,-1] = 0 #zero out yaw since we already compensated for it
		
		#transform velocity from world frame to lidar base frame 
		rectified_vel[0,:3] = rectified_vel[0,:3] @ rot_vec #fix translational components
		print("rectified_vel (translation fixed):", rectified_vel)

		# # don't think this is necessary...
		# # fix rotational components of <rectified_vel> to align with world frame??? ----------------------
		# rectified_vel_body_frame = R.from_euler('xyz', rectified_vel[0,3:])
		# rectified_vel[0,3:] = (base_rot_quat * rectified_vel_body_frame).as_euler('xyz')
		# print("rectified_vel (trans + rot fixed):", rectified_vel)
		# #------------------------------------------------------------------------------------------------

		T = (2*np.pi)/(-vel[-1] + self.lidar_cmd_vel) #time to complete 1 scan #was this
		print("T:", T)
		rectified_vel = rectified_vel * T
		# print("rectified_vel:", rectified_vel)
		#TODO: is this is a bad way of doing it? ... what happens if most of the points are on one half of the scene??
		part2 = np.linspace(0.5, 1.0, len(self.pc_xyz)//2)[:,None]
		part1 = np.linspace(0, 0.5, len(self.pc_xyz) - len(self.pc_xyz)//2)[:,None]
		motion_profile = np.append(part1, part2, axis = 0) @ rectified_vel  
		undistorted_pc = self.remove_motion_distortion(self.pc_xyz, motion_profile)

		#debug: add points at theta = 0------------------------------------
		# z = np.zeros([100,3])
		# z[:,2] = np.pi/2
		# z[:,0] = np.linspace(0,10,100)
		# undistorted_pc = np.append(undistorted_pc, self.s2c(z), axis  = 0)

		# #identify the points at the end of undistorted PC
		# modified_end_points = undistorted_pc[-100:,:] + np.ones([100,3])*np.array([0,0,3])
		# undistorted_pc = np.append(undistorted_pc, modified_end_points, axis = 0)

		# #draw points at split line
		# sl = np.ones([100,3])
		# sl[:,0] = np.linspace(0,10,100)
		# sl[:,2] = np.pi/2
		# sl[:,1] = total_rot
		# undistorted_pc = np.append(undistorted_pc, self.s2c(sl), axis  = 0)
		#------------------------------------------------------------------

		self.pcPub.publish(point_cloud(undistorted_pc, 'map'))

		if self.save_data:
			fn_raw = root_dir + trajectory_name + "raw_frame_" + str(self.frame_idx)
			fn_rectified = root_dir + trajectory_name + "rectified_linear_frame_" + str(self.frame_idx) 

			np.save(fn_raw, np.array(xyz))
			np.save(fn_rectified, undistorted_pc)
			self.frame_idx += 1

	def remove_motion_distortion(self, points, motion_profile):
		"""
		Removes motion distortion from 3D LIDAR data. 

		Args:
		- points: numpy array of shape (N, 3) containing 3D LIDAR points
		- motion_profile: numpy array of shape (N, 6) containing the motion profile of the LIDAR device during data acquisition
		  				  DATA IS IN WORLD REFERENCE FRAME

		Returns:
		- corrected_points: numpy array of shape (N, 3) containing the motion distortion corrected 3D LIDAR points
		"""

		#OLD METHOD (SLOW) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
		
		#should be the same size:
		# print(len(T))
		# print(len(points))
		# Apply inverse of motion transformation to each point
		corrected_points = np.zeros_like(points)
		for i in range(len(points)):
			point = np.concatenate((points[i], np.array([1])))
			T_inv = np.linalg.inv(T[i])
			corrected_point = np.dot(T_inv, point)[:3]
			corrected_points[i] = corrected_point
		
		#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		return corrected_points


	def base_vel_setpoint(self, t):
		"""callback func for when node recieves linear velocity estimate from SensorMover"""
		# vel_pub_rate = 1000 # THIS DEPENDS ON LIDAR SAMPLING FREQUENCY (i.e. 1Hz for testing)
		vel_pub_rate = 1 #for new main loop in </sensor_mover>
		self.linear_vel_estimate = np.array([t.linear.x, t.linear.y, t.linear.z,
											t.angular.x, t.angular.y, t.angular.z]) * vel_pub_rate
		# print(self.linear_vel_estimate)
		self.vel_history_for_this_frame = np.append(self.vel_history_for_this_frame,
													self.linear_vel_estimate[None,:] / vel_pub_rate, axis = 0)
		# print(np.shape(self.vel_history_for_this_frame))

	def on_link_states(self, link_states):
		"""callback for getting link frames out of Gazebo"""

		vq_base = link_states.pose[1].orientation
		velodyne_quat_base = [vq_base.x, vq_base.y, vq_base.z, vq_base.w]
		self.velodyne_quat_base = R.from_quat(velodyne_quat_base)
		self.velodyne_euls_base = self.velodyne_quat_base.as_euler('xyz')
		# print("base euler angles:", self.velodyne_euls_base)

		#for debug
		self.current_xpos = link_states.pose[1].position.x
		# print(self.current_xpos)
		self.current_xvel = link_states.twist[1].linear.x
		# print(self.current_xvel)
		self.current_theta = link_states.pose[1].orientation.z

	def on_lidar_vel_cmd(self, cmd_vel):
		"""cb to get rotational velocity commanded of LIDAR unit -> need this to properly uncurl the scan"""
		self.lidar_cmd_vel = cmd_vel.data
		self.period_lidar  = (2*np.pi)/self.lidar_cmd_vel
		print("updating lidar cmd_vel to:", self.lidar_cmd_vel)

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