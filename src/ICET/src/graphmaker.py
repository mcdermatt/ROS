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

#TODO:
#   start out with "simple" unweighted graph to describe connections between woven nodes (not correct)

#       Make seprate node to draw the graphical output of this 

#   Convert 6DOF transform to Homogenous Transformation

class GraphMaker():
    """ Attemps to optimize many local transformations coming from simulatneous ICET scan registrations

        Takes in /relative_transform from each ICET thread

        Publishes optimized overall trajectory    

    """
    
    def __init__(self):

        rospy.init_node('graphmaker', anonymous=False)

        #Need this to know when LIDAR drive trajectory restarts
        self.etc_sub = rospy.Subscriber('lidar_info', Num, self.get_info)

        # using numpy + raw transfrom outputs (more difficult) ----------------
        # subscribe to local transformation estimates output by ScanMatcher
        self.TSub = rospy.Subscriber('relative_transform', Floats, self.on_transform) 
        self.SigmaSub = rospy.Subscriber('relative_covariance', Floats, self.on_cov) 
        # publish incidence matrix of graph
        self.incidence_matrix_Pub = rospy.Publisher('incidence_matrix', numpy_msg(Floats), queue_size = 1)
        #test
        # self.incidence_matrix_Pub = rospy.Publisher('incidence_matrix', Floats, queue_size = 1)
        # ---------------------------------------------------------------------

        r = 100
        self.rate = rospy.Rate(r)

        self.restart()

    def restart(self):
        """restart graph at the beginning of a new trajectory"""

        #init incidence matrix
        # self.incidence_matrix = None #np.zeros([1, 1])
        self.incidence_matrix = np.zeros([2, 2])
        self.T_vec_history = np.zeros([0,8]) 
        self.cov_vec_history =  np.zeros([0,8]) # np.zeros([0,36]) 

    def get_info(self, data):
        """ Gets Lidar info from custom Num msg """

        self.scan_data = data
        # print("frame idx:", data.frame)

        if self.scan_data.restart == True:
            # np.savetxt("/home/derm/ROS/src/ICET/src/incidence_matrix.txt", self.incidence_matrix) #save to disk for debug 
            self.restart()

    def on_transform(self, local_estimate):
        """called when ScanMatcher node publishes local transformation estimate"""

        #local transforms come in as:
        # [x, y, z, phi, theta, psi, idx_keyframe, idx_newframe]
        self.local_estimate = np.array(local_estimate.data)
        # print("\n self.local_estimate \n", self.local_estimate[None,:])

        #update T_vec
        self.T_vec_history = np.append(self.T_vec_history, self.local_estimate[None,:], axis = 0)
        # print("self.T_vec_history: \n", self.T_vec_history)
        np.save("/home/derm/ROS/src/ICET/src/T_vec_history_3and4and5", self.T_vec_history) #save to disk for testing 
        # ...2and3 - registering on every 2nd and every 3rd frame

        #UPDATE GRAPH ------------------------
        #[Nodes, Edges] 
        #   Nodes are the frame indicecs (LIDAR scans 1, 2, 3...) 
        #   Edges are the local transforms connecting each Node

        ## DEBUG: make sure our incidence matrix indices line up with scan indices, even after looping through dataset 
        # if self.incidence_matrix is None:
        #     #set to size of current graph - 1
        #     self.incidence_matrix = np.zeros([int(self.local_estimate[6])-1, int(self.local_estimate[6])-1])

        #if graph isn't big enough to accomodate new scan, extend graph
        # if np.shape(self.incidence_matrix)[0] < self.local_estimate[7]:
        #     self.incidence_matrix = np.pad(self.incidence_matrix, [(0, int(self.local_estimate[7] - np.shape(self.incidence_matrix)[0]) + 2),(0,0)]) #add nodes

        while np.shape(self.incidence_matrix)[0] <= self.local_estimate[7]:
            self.incidence_matrix = np.pad(self.incidence_matrix, [(0, 1),(0,0)]) #add nodes

        #add edges
        self.incidence_matrix = np.pad(self.incidence_matrix, [(0,0),(0,1)]) 
        self.incidence_matrix[int(self.local_estimate[6]), -1] = 1 #set indices for associated scans to 1
        self.incidence_matrix[int(self.local_estimate[7]), -1] = 1 

        # print("\n Incidence Matrix:", np.shape(self.incidence_matrix))
        # print("\n Incidence Matrix:", self.incidence_matrix)
        #-------------------------------------

        # publish Incidence Matrix
        # self.incidence_matrix_Pub.publish(self.incidence_matrix.astype(np.float32)) #can't publish 2D array???

        #need to send in 1d [(shape), flat_IM]
        temp_arr = np.append(np.array(np.shape(self.incidence_matrix)), self.incidence_matrix.flatten())
        self.incidence_matrix_Pub.publish(temp_arr.astype(np.float32))

    def on_cov(self, local_estimate):
        """ called when covaraince is eastimated for each local transformation """

        self.cov_vec_history = np.append(self.cov_vec_history, np.array(local_estimate.data)[None,:], axis = 0)
        print("self.cov_vec_history: \n", self.cov_vec_history)
        np.save("/home/derm/ROS/src/ICET/src/cov_vec_history_3and4and5", self.cov_vec_history) #save to disk for testing 


if __name__ == '__main__':
    g = GraphMaker()

    while not rospy.is_shutdown():

        rospy.spin()

#old - using TF tree structure -- not sure this will work for woven transforms
# if __name__ == '__main__':

#     rospy.init_node('graphmaker')

#     source_frame = 'map'
#     target_frame = 'child_tf_frame'

#     #use tf2 frames output by <scan_matcher> (more direct??)
#     tfbuffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tfbuffer)

#     rate = rospy.Rate(1)

#     while not rospy.is_shutdown():

#         rate.sleep()
#         try:
#             trans = tfbuffer.lookup_transform(target_frame, source_frame, rospy.Time())
#             print("\n trans: \n", trans)
#         except:
#             print("not ready")