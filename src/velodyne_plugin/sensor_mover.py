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

#TODO: 
#      prompt user input

class SensorMover():
  """Sends commands to Gazebo to move the LIDAR sensor"""

  def __init__(self):

    rospy.init_node('sensormover', anonymous=False)

    # self.sensor_mover_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size = 1)
    self.sensor_mover_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
    self.model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelState, self.on_model_states, queue_size = 1)
    self.rate = rospy.Rate(1000) # hz

    self.lidar_state = ModelState()
    # set lidar height to that of KITTI
    # (and so we don't get weird collision effects with ground plane)
    self.lidar_state.pose.position.z = 1.72 

    #test
    self.dv_x = 0
    self.dv_y = 0

  def main(self):

    while not rospy.is_shutdown(): #while there's still a roscore

      self.lidar_state.model_name = 'my_velodyne'

      # #simple linear motion ---------------------------------------------
      # self.lidar_state.twist.linear.x  = 0.001
      # self.lidar_state.twist.linear.y  = 0.
      # self.lidar_state.twist.linear.z  = 0.
      # self.lidar_state.twist.angular.x = 0.
      # self.lidar_state.twist.angular.y = 0.
      # self.lidar_state.twist.angular.z = 0.
      # self.lidar_state.pose.position.x += self.lidar_state.twist.linear.x
      # self.lidar_state.pose.position.y += self.lidar_state.twist.linear.y
      # self.lidar_state.pose.position.z += self.lidar_state.twist.linear.z
      # #------------------------------------------------------------------

      #brownian motion --------------------------------------------------
      self.dv_x = np.random.randn()
      self.dv_y = np.random.randn()
      self.lidar_state.twist.linear.x  = 0.001 #0.04*self.dv_x
      self.lidar_state.twist.linear.y  = 0. #0.04*self.dv_y
      self.lidar_state.twist.linear.z  = 0.
      self.lidar_state.twist.angular.x = 0.00
      self.lidar_state.twist.angular.y = 0.
      self.lidar_state.twist.angular.z = 0.001 #yaw
      self.lidar_state.pose.position.x += self.lidar_state.twist.linear.x
      self.lidar_state.pose.position.y += self.lidar_state.twist.linear.y
      self.lidar_state.pose.position.z += self.lidar_state.twist.linear.z
      self.lidar_state.pose.orientation.x += self.lidar_state.twist.angular.x
      self.lidar_state.pose.orientation.y += self.lidar_state.twist.angular.y
      self.lidar_state.pose.orientation.z += self.lidar_state.twist.angular.z
      # self.lidar_state.pose.orientation.z = 2 #debug
      self.lidar_state.pose.orientation.w = 1
      #------------------------------------------------------------------

      self.sensor_mover_pub.publish(self.lidar_state)

      #IMPORTANT: This is tied to simulation time (gazebo) NOT wall time
      self.rate.sleep()

  def on_model_states(self, model_states):
    """callback for recieving model states from gazebo"""

    print(model_states)

if __name__ == '__main__':
  sm = SensorMover()
  sm.main()
  print("done??")