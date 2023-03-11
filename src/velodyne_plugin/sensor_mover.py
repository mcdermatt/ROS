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
#      set sensor velocity without resetting pose to origin 

class SensorMover():
  """Sends commands to Gazebo to move the LIDAR sensor"""

  def __init__(self):

    rospy.init_node('sensormover', anonymous=False)

    # self.sensor_mover_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size = 1)
    self.sensor_mover_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
    self.rate = rospy.Rate(100) # hz

  def main(self):

    while not rospy.is_shutdown(): #while there's still a roscore

      # link_state = LinkState()
      # link_state.link_name = 'my_velodyne::base'
      # link_state.twist.linear.x  = 0.01
      # link_state.twist.linear.y  = 0.
      # link_state.twist.linear.z  = 0.
      # link_state.twist.angular.x = 0.
      # link_state.twist.angular.y = 0.
      # link_state.twist.angular.z = 0.
      # self.sensor_mover_pub.publish(link_state)

      model_state = ModelState()
      model_state.model_name = 'my_velodyne'
      model_state.twist.linear.x  = 0.
      model_state.twist.linear.y  = 5.0
      model_state.twist.linear.z  = 0.
      model_state.twist.angular.x = 0.
      model_state.twist.angular.y = 0.
      model_state.twist.angular.z = 0.
      self.sensor_mover_pub.publish(model_state)

      #IMPORTANT: This is tied to simulation time (gazebo) NOT wall time
      self.rate.sleep()

if __name__ == '__main__':
  sm = SensorMover()
  sm.main()
  print("done??")