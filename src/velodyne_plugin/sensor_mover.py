#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg as std_msgs
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
# from gazebo_msgs.srv import SetLinkState
import sensor_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import tensorflow #as tf #pretty dumb naming convention smh
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

#TODO: break into two seprate Nodes:
#       1) </sensor_mover>:
#           - subscribes to vel_setpoint Twist msg,
#           - publishes 
#       2) </vel_setpoint_setter>
#           - publishes vel setpoint

# Tradeoff:
#       Slower pub_rate produces measured delta that more closely matches rectified vel
#       BUT slow pub_rates will prevent future testing on non-constant velocity 

class SensorMover():
  """Sends commands to Gazebo to move the LIDAR sensor"""

  def __init__(self):

    rospy.init_node('sensormover', anonymous=False)

    self.sensor_mover_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size = 1)
    # self.sensor_mover_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1) #was this
    self.twist_pub = rospy.Publisher('/velodyne_base_vel_setpoint', Twist, queue_size = 1) #was this
    self.pub_rate = 1000 # Hz #need for older version
    # self.pub_rate = 10 #test
    self.rate = rospy.Rate(self.pub_rate) 
    self.reset()

  def reset(self):
    print("RESET")
    # self.lidar_state = ModelState()
    # self.lidar_state.model_name = 'my_velodyne'
    self.lidar_state = LinkState()
    self.lidar_state.link_name = 'base'
    # set lidar height to that of KITTI
    # (and so we don't get weird collision effects with ground plane)
    self.lidar_state.pose.position.z = 1.72 

    self.count = 0
    self.dv_x = 0
    self.dv_y = 0
    self.lidar_state.pose.orientation.w = 1 #needed to normalize quats

    #Set linear velocity of sensor
    self.Twist_command = Twist()
    self.Twist_command.linear.x =  1.5 # 33 mph
    # self.Twist_command.linear.x =  1.0 # 22 mph
    # self.Twist_command.linear.x =  0.44 # 10 mph
    # self.Twist_command.linear.y = 1.0
    # self.Twist_command.linear.z = 0.5
    # self.Twist_command.angular.x = 0.5
    self.Twist_command.angular.y =  0.2
    self.Twist_command.angular.z = -1.0

  def set_constant_velocity(self):
    """with new and improved anit gimbal-lock technology"""
    while not rospy.is_shutdown():

      self.lidar_state.pose.position.x += (self.Twist_command.linear.x / self.pub_rate)
      self.lidar_state.pose.position.y += (self.Twist_command.linear.y / self.pub_rate)
      self.lidar_state.pose.position.z += (self.Twist_command.linear.z / self.pub_rate)

      #get current pose
      curr_pose_quat = [self.lidar_state.pose.orientation.x,
                        self.lidar_state.pose.orientation.y,
                        self.lidar_state.pose.orientation.z,
                        self.lidar_state.pose.orientation.w]
      # print("\n curr_pose_quat:", curr_pose_quat)

      #IMPORTANT: need to re-normalize quats!
      curr_pose_quat = R.from_quat(curr_pose_quat).as_quat()
      # print("after norm:", curr_pose_quat)

      #convert to euler angles to linearize
      curr_pose_eul = R.from_quat(curr_pose_quat).as_euler('xyz')

      # dot angular twist commands with current pose to get correct angles (and avoid gimbal lock):

      # # using DCM representation
      # body_frame_adjustments_dcm = R.from_euler('xyz', [self.Twist_command.angular.x,
      #                                               self.Twist_command.angular.y,
      #                                               self.Twist_command.angular.z]).as_dcm()
      # print("\n body_frame_adjustments:",  R.from_dcm(body_frame_adjustments_dcm).as_euler('xyz'))
      # curr_pose_dcm = R.from_quat(curr_pose_quat).as_dcm()
      # print("curr_pose_dcm: \n", curr_pose_dcm)
      # # world_frame_adjustments = curr_pose_dcm @ body_frame_adjustments_dcm @ np.linalg.pinv(curr_pose_dcm)  #was this - wrong
      # world_frame_adjustments = curr_pose_dcm @ body_frame_adjustments_dcm 
      # # print(world_frame_adjustments)
      # world_frame_adjustments_eul = R.from_dcm(world_frame_adjustments).as_euler('xyz')
      # print("world_frame_adjustments_eul", world_frame_adjustments_eul)

      # # using Quat representation
      # body_frame_adjustments_quat = R.from_euler('xyz', [self.Twist_command.angular.x,
      #                                               self.Twist_command.angular.y,
      #                                               self.Twist_command.angular.z]).as_quat()
      # print("\n body_frame_adjustments:",  R.from_quat(body_frame_adjustments_quat).as_euler('xyz'))
      # world_frame_adjustments = curr_pose_quat * body_frame_adjustments_quat
      # world_frame_adjustments_eul = R.from_quat(world_frame_adjustments).as_euler('xyz')
      # print("world_frame_adjustments_eul", world_frame_adjustments_eul)

      #using scipy.rot builtin methods
      body_frame_adjustments = R.from_euler('xyz', [self.Twist_command.angular.x,
                                                    self.Twist_command.angular.y,
                                                    self.Twist_command.angular.z])
      # print("\n body_frame_adjustments:", body_frame_adjustments.as_euler('xyz'))
      curr_pose = R.from_quat(curr_pose_quat)
      # print("curr_pose", curr_pose.as_euler('xyz'))
      # world_frame_adjustments = curr_pose * body_frame_adjustments * curr_pose.inv()
      world_frame_adjustments = curr_pose * body_frame_adjustments

      # #convert back to quat
      # world_frame_adjustments_eul = world_frame_adjustments.as_euler('xyz')
      # print("world_frame_adjustments_eul", world_frame_adjustments_eul)
      # curr_pose_eul[0] += world_frame_adjustments_eul[0] / self.pub_rate
      # curr_pose_eul[1] += world_frame_adjustments_eul[1] / self.pub_rate
      # curr_pose_eul[2] += world_frame_adjustments_eul[2] / self.pub_rate
      # # curr_pose_eul[2] = -3*np.pi/4 #set z rotation to static nonzero value
      # # print("curr_pose_eul:", curr_pose_eul)
      # new_pose_quat = R.from_euler('xyz', curr_pose_eul).as_quat()
      # self.lidar_state.pose.orientation.x = new_pose_quat[0]
      # self.lidar_state.pose.orientation.y = new_pose_quat[1]
      # self.lidar_state.pose.orientation.z = new_pose_quat[2]
      # self.lidar_state.pose.orientation.w = new_pose_quat[3]

      #use only quaternion representation
      world_frame_adjustments_quat = world_frame_adjustments.as_quat()
      # print("world_frame_adjustments", world_frame_adjustments.as_euler('xyz'))
      self.lidar_state.pose.orientation.x = curr_pose_quat[0] + (world_frame_adjustments_quat[0] / self.pub_rate)
      self.lidar_state.pose.orientation.y = curr_pose_quat[1] + (world_frame_adjustments_quat[1] / self.pub_rate)
      self.lidar_state.pose.orientation.z = curr_pose_quat[2] + (world_frame_adjustments_quat[2] / self.pub_rate)
      self.lidar_state.pose.orientation.w = curr_pose_quat[3] + (world_frame_adjustments_quat[3] / self.pub_rate)
      self.sensor_mover_pub.publish(self.lidar_state)
      self.twist_pub.publish(self.Twist_command)

      if self.lidar_state.pose.position.x > 16: # 16 for test1, 100 for test3 
        self.reset()

      self.rate.sleep()

  # def set_constant_velocity(self):
  #   """Old version, relies on small angle assumtion"""

  #   while not rospy.is_shutdown(): #while there's still a roscore

  #     self.lidar_state.pose.position.x += (self.Twist_command.linear.x / self.pub_rate)
  #     self.lidar_state.pose.position.y += (self.Twist_command.linear.y / self.pub_rate)
  #     self.lidar_state.pose.position.z += (self.Twist_command.linear.z / self.pub_rate)

  #     #get current pose
  #     curr_pose_quat = [self.lidar_state.pose.orientation.x,
  #                       self.lidar_state.pose.orientation.y,
  #                       self.lidar_state.pose.orientation.z,
  #                       self.lidar_state.pose.orientation.w]
  #     print("\n curr_pose_quat:", curr_pose_quat)

  #     #convert to euler angles to linearize
  #     curr_pose_eul = R.from_quat(curr_pose_quat).as_euler('xyz')

  #     curr_pose_eul[0] += self.Twist_command.angular.x / self.pub_rate
  #     curr_pose_eul[1] += self.Twist_command.angular.y / self.pub_rate
  #     curr_pose_eul[2] += self.Twist_command.angular.z / self.pub_rate
  #     # curr_pose_eul[2] = -3*np.pi/4 #set z rotation to static nonzero value
  #     print("curr_pose_eul:", curr_pose_eul)

  #     #convert back to quat
  #     new_pose_quat = R.from_euler('xyz', curr_pose_eul).as_quat()
  #     self.lidar_state.pose.orientation.x = new_pose_quat[0]
  #     self.lidar_state.pose.orientation.y = new_pose_quat[1]
  #     self.lidar_state.pose.orientation.z = new_pose_quat[2]
  #     self.lidar_state.pose.orientation.w = new_pose_quat[3]

  #     self.sensor_mover_pub.publish(self.lidar_state)
  #     self.twist_pub.publish(self.Twist_command)

  #     if self.lidar_state.pose.position.x > 16: # 16 for test1, 100 for test3 
  #       self.reset()

  #     self.rate.sleep()

  def set_acceleration(self):

    while not rospy.is_shutdown(): #while there's still a roscore

      #IMPORTANT: MAKE SURE TO RESCALE THESE VALUES IF CHANING LIDAR SAMPLING FREQUENCY!

      #acceleration equivalent of going from 0-60 in 10s
      self.Twist_command.linear.x += (0.27 / self.pub_rate) 

      #max truning acceleration observed in Ford Dataset 
      # self.Twist_command.angular.z += (0.05 / self.pub_rate)

      self.lidar_state.pose.position.x += (self.Twist_command.linear.x / self.pub_rate)
      self.lidar_state.pose.position.y += (self.Twist_command.linear.y / self.pub_rate)
      self.lidar_state.pose.position.z += (self.Twist_command.linear.z / self.pub_rate)

      #get current pose
      curr_pose_quat = [self.lidar_state.pose.orientation.x,
                        self.lidar_state.pose.orientation.y,
                        self.lidar_state.pose.orientation.z,
                        self.lidar_state.pose.orientation.w]

      #convert to euler angles to linearize
      curr_pose_eul = R.from_quat(curr_pose_quat).as_euler('xyz')

      curr_pose_eul[0] += self.Twist_command.angular.x / self.pub_rate
      curr_pose_eul[1] += self.Twist_command.angular.y / self.pub_rate
      curr_pose_eul[2] += self.Twist_command.angular.z / self.pub_rate
      # curr_pose_eul[2] = -3*np.pi/4 #set z rotation to static nonzero value

      #convert back to quat
      new_pose_quat = R.from_euler('xyz', curr_pose_eul).as_quat()
      self.lidar_state.pose.orientation.x = new_pose_quat[0]
      self.lidar_state.pose.orientation.y = new_pose_quat[1]
      self.lidar_state.pose.orientation.z = new_pose_quat[2]
      self.lidar_state.pose.orientation.w = new_pose_quat[3]

      self.sensor_mover_pub.publish(self.lidar_state)
      self.twist_pub.publish(self.Twist_command)

      if self.lidar_state.pose.position.x > 16:
        self.reset()

      self.count += 1

      self.rate.sleep()


if __name__ == '__main__':
  sm = SensorMover()
  sm.set_constant_velocity()
  # sm.set_acceleration()