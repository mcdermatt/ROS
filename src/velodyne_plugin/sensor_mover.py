#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg as std_msgs
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from gazebo_msgs.msg import LinkState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetLinkState
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

class SensorMover():
  """Sends commands to Gazebo to move the LIDAR sensor"""

  def __init__(self):

    rospy.init_node('sensormover', anonymous=False)

    # self.sensor_mover_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size = 1)
    self.sensor_mover_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
    self.twist_pub = rospy.Publisher('/velodyne_base_vel_setpoint', Twist, queue_size = 1)
    # self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelState, self.on_model_states, queue_size = 1)    #depreciated??
    self.pub_rate = 1000 # Hz
    # self.pub_rate = 0.25 #debug
    self.rate = rospy.Rate(self.pub_rate) 
    self.reset()

  def reset(self):
    print("RESET")
    self.lidar_state = ModelState()
    # set lidar height to that of KITTI
    # (and so we don't get weird collision effects with ground plane)
    self.lidar_state.pose.position.z = 1.72 

    self.count = 0
    self.dv_x = 0
    self.dv_y = 0
    self.lidar_state.pose.orientation.w = 1 #needed to normalize quats

  def main_new(self):

    while not rospy.is_shutdown():

      self.lidar_state.model_name = 'my_velodyne'
      self.lidar_state.twist.linear.x  = 3.
      self.lidar_state.twist.linear.y  = 0.
      self.lidar_state.twist.linear.z  = 0.
      self.lidar_state.twist.angular.x = 0.00
      self.lidar_state.twist.angular.y = 0.
      self.lidar_state.twist.angular.z = 0. #0.5 #yaw
      self.sensor_mover_pub.publish(self.lidar_state)
      print("publishing sensor mover state")

      # #reset if we get too far from start pose
      while self.lidar_state.pose.position.x < 8:
        # print(self.count) #constanly outputs the same value

        self.twist_pub.publish(self.lidar_state.twist)
        print("publishing twist msg")
        #IMPORTANT: This is tied to simulation time (gazebo) NOT wall time
        self.rate.sleep()

      self.reset()


  def main_old(self):

    while not rospy.is_shutdown(): #while there's still a roscore

      self.lidar_state.model_name = 'my_velodyne'

      # #simple linear motion ---------------------------------------------
      # self.lidar_state.twist.linear.x  = 0.003
      # self.lidar_state.twist.linear.y  = 0.
      # self.lidar_state.twist.linear.z  = 0.
      # self.lidar_state.pose.position.x += self.lidar_state.twist.linear.x
      # self.lidar_state.pose.position.y += self.lidar_state.twist.linear.y
      # self.lidar_state.pose.position.z += self.lidar_state.twist.linear.z
      # #------------------------------------------------------------------

      #more complex motion -----------------------------------------------
      # self.dv_x = np.random.randn()
      # self.dv_y = np.random.randn()
      # self.lidar_state.twist.linear.x  = 0.001 #0.001 #*self.dv_x
      self.lidar_state.twist.linear.y  = 0. #0.001 #0.04*self.dv_y
      self.lidar_state.twist.linear.z  = 0.
      self.lidar_state.twist.angular.x = 0.00
      self.lidar_state.twist.angular.y = 0.
      self.lidar_state.twist.angular.z = -0.00125 #fast yaw
      # self.lidar_state.twist.angular.z = -0.00025 #slower yaw
      # self.lidar_state.twist.angular.z = -2e-5 #realistic KITTI yaw
      self.lidar_state.pose.position.x += self.lidar_state.twist.linear.x
      # self.lidar_state.pose.position.x += 0.003
      self.lidar_state.pose.position.y += self.lidar_state.twist.linear.y
      self.lidar_state.pose.position.z += self.lidar_state.twist.linear.z

      #get current pose
      curr_pose_quat = [self.lidar_state.pose.orientation.x,
                        self.lidar_state.pose.orientation.y,
                        self.lidar_state.pose.orientation.z,
                        self.lidar_state.pose.orientation.w]
      #convert to euler angles to linearize
      curr_pose_eul = R.from_quat(curr_pose_quat).as_euler('xyz')
      curr_pose_eul[0] += self.lidar_state.twist.angular.x
      curr_pose_eul[1] += self.lidar_state.twist.angular.y
      curr_pose_eul[2] += self.lidar_state.twist.angular.z
      # curr_pose_eul[2] = -3*np.pi/4 #set z rotation to static nonzero value
      # print(curr_pose_eul)
      #convert back to quat
      new_pose_quat = R.from_euler('xyz', curr_pose_eul).as_quat()
      # print(new_pose_quat)
      self.lidar_state.pose.orientation.x = new_pose_quat[0]
      self.lidar_state.pose.orientation.y = new_pose_quat[1]
      self.lidar_state.pose.orientation.z = new_pose_quat[2]
      self.lidar_state.pose.orientation.w = new_pose_quat[3]
      # print(self.lidar_state.twist)
      self.count += 1
      #------------------------------------------------------------------

      #reset if we get too far from start pose
      if self.lidar_state.pose.position.x > 16:
        # print(self.count) #constanly outputs the same value
        self.reset()

      self.sensor_mover_pub.publish(self.lidar_state)

      #TODO: pub commanded twist @twistpub
      self.twist_pub.publish(self.lidar_state.twist)

      #IMPORTANT: This is tied to simulation time (gazebo) NOT wall time
      self.rate.sleep()

  # def on_model_states(self, model_states):
  #   """callback for recieving model states from gazebo"""

  #   print(model_states)

if __name__ == '__main__':
  sm = SensorMover()
  sm.main_old() #loses frames, causes simulation to depart from prescribed trajectory (bad)
  # sm.main_new() #attempts to fix above issue
  print("done??")