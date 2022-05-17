#!/usr/bin/env python

from __future__ import print_function
from six.moves import input
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import time
import tf
import math

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from simple_script_server import *
sss = simple_script_server()       


def distance(frame):
        listener = tf.TransformListener()
        trans = []
        rot = []
        rate = rospy.Rate(10.0)
        while not trans:
            try:
                (trans,rot) = listener.lookupTransform('vicon/box/box', frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        #(trans,rot) = listener.lookupTransform('map', frame, rospy.Time(0))
        #rate.sleep()
        print ('Translation: ' , trans)
        print ('Rotation: ' , rot)
        return trans,rot
       
if __name__ == '__main__':
        rospy.init_node("tf_distance")     
        #obj_name="can"
        #position_can,orientation=get_pose("vicon/"+obj_name+"/"+obj_name)
        obj_name="Ball"
        position_box,orientation=distance("vicon/"+obj_name+"/"+obj_name)
        loss=position_box[0]**2+position_box[1]**2
        print("loss: ",loss)