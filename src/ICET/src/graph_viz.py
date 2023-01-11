#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import numpy as np
from ICET.msg import Num #using custom message type
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import std_msgs.msg as std_msgs
import sensor_msgs
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from laser_geometry import LaserProjection
from utils import R_tf

import tf
import tf2_ros
import tf2_geometry_msgs


class GraphViz():

	def __init__(self, graph_topic = "incidence_matrix"):

		rospy.init_node('GraphMaker', anonymous=False)

		self.incidence_matrix_sub = rospy.Subscriber(graph_topic, numpy_msg(Floats), self.on_update)

		r = 10
		self.rate = rospy.Rate(r)


	def on_update(self, IM_raw):

		self.IM = np.reshape(IM_raw.data[2:], IM_raw.data[:2].astype(int) )

		# print("IM_raw: \n", IM_raw.data[:2].astype(np.int)  )
		print("Incidence Matrix: \n", self.IM)
		# np.savetxt("/home/derm/ROS/src/ICET/src/incidence_matrix.txt", self.IM.astype(np.int)) #save to disk for debug 


if __name__ == '__main__':
    gv = GraphViz()

    while not rospy.is_shutdown():

        rospy.spin()