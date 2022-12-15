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

class GraphMaker():
    """ Attemps to optimize many local transformations coming from simulatneous ICET scan registrations

        Takes in tf2 outputs from each ICET thread

        Publishes optimized overall trajectory
        Publishes SMOOTHED trajectory from subsequent measurements
    
    """
    
    def __init__(self):

        rospy.init_node('graphmaker', anonymous=False)

        #Need this to know when LIDAR drive trajectory restarts
        # self.etc_sub = rospy.Subscriber('lidar_info', Num, self.get_info)

        #using numpy + raw transfrom outputs (more difficult) ----------------
        #subscribe to local transformation estimates output by ScanMatcher
        # self.TSub = rospy.Subscriber('relative_transform', Floats, self.on_transform) 
        # self.SigmaSub = rospy.Subscriber('relative_covariance', Floats, self.on_cov) 
        #publish overall trajectory
        # self.GraphPub = rospy.Publisher('graph', numpy_msg(Floats), queue_size = 1)
        #---------------------------------------------------------------------

        r = 10
        self.rate = rospy.Rate(r)

        self.restart()

    def restart(self):
        """restart graph at the beginning of a new trajectory"""

        #init transformation graph
        # [x, y, z, phi, theta, psi, idx_keyframe, idx_newframe]
        self.graph = np.zeros([1, 8]) #TODO: figure out a better way to do this

    def get_info(self, data):
        """ Gets Lidar info from custom Num msg """

        self.scan_data = data
        print("frame idx:", data.frame)

        if self.scan_data.restart == True:
            np.save("/home/derm/ROS/src/ICET/src/graph", self.graph) #save to disk for debug 
            self.restart()

    def on_transform(self, local_estimate):
        """called when ScanMatcher node publishes local transformation estimate"""

        self.local_estimate = np.array(local_estimate.data)[None, :]
        print("\n self.local_estimate \n", self.local_estimate)

        #UPDATE GRAPH ------------------------
        self.graph = np.append(self.graph, self.local_estimate, axis = 0)
        # print("\n graph:",self.graph)

        #-------------------------------------


        #publish updated graph

    def on_cov(self, local_estimate):
        """ called when covaraince is eastimated for each local transformation """

        pass


if __name__ == '__main__':

    rospy.init_node('graphmaker')

    source_frame = 'map'
    target_frame = 'child_tf_frame'

    #use tf2 frames output by <scan_matcher> (more direct??)
    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        rate.sleep()
        try:
            trans = tfbuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            print("\n trans: \n", trans)
        except:
            print("not ready")