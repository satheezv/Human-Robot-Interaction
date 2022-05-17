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

from std_msgs.msg import String
from simple_script_server import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray,MultiArrayDimension 
from moveit_commander.conversions import pose_to_list
sss = simple_script_server()       

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


class VelocityNode:
    def __init__(self):
        self.velocity=0
        self.velocities=[]
        self.msg=Float64MultiArray()
        self.dim = MultiArrayDimension()
    def velocity_setter(self):
        self.pub = rospy.Publisher('/arm_right/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.pub_pos = rospy.Publisher('/arm_right/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        self.pub_torso = rospy.Publisher('/torso/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        angle=0.0
        while not rospy.is_shutdown():
            angle-=4
            #4=45
            #1=11
            if angle<=-60:
                print("Limit")
                # self.msg.data = [0.0,0.0,0.0,-1.5708,1.5708,0.0,-1.57]
                # self.dim.size = len(self.msg.data)
                # self.dim.label = "command"
                # self.dim.stride = len(self.msg.data)
                # self.msg.layout.dim.append(self.dim)
                # self.msg.layout.data_offset = 0
                # self.pub_pos.publish(self.msg)
                # self.msg.data = [0.0,0.0]
                # self.dim.size = len(self.msg.data)
                # self.dim.label = "command"
                # self.dim.stride = len(self.msg.data)
                # self.msg.layout.dim.append(self.dim)
                # self.msg.layout.data_offset = 0
                # self.pub_torso.publish(self.msg)
                break
            else:
                self.msg.data = [angle,0.0,angle,0.0,0.0,0.0,0.0]
                self.dim.size = len(self.msg.data)
                self.dim.label = "command"
                self.dim.stride = len(self.msg.data)
                self.msg.layout.dim.append(self.dim)
                self.msg.layout.data_offset = 0
                self.pub.publish(self.msg)

                ####torso
                # self.msg.data = [0.0,0.0]
                # self.dim.size = len(self.msg.data)
                # self.dim.label = "command"
                # self.dim.stride = len(self.msg.data)
                # self.msg.layout.dim.append(self.dim)
                # self.msg.layout.data_offset = 0
                # self.pub_torso.publish(self.msg)
            rate.sleep()   
            


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
    #rospy.init_node('talker', anonymous=True)  
    sss = simple_script_server()
    sss.init("arm_right")
    sss.recover("arm_right")
    sss.init("torso")
    sss.recover("torso")
    torso=sss.move("torso","front")
    torso.wait()
    # torso=sss.move("torso","right")
    # torso.wait()
    right_arm=sss.move("arm_right","home")
    right_arm.wait()
    right_arm=sss.move("arm_right",[[1.5708,0.0,2.0,-1.5708,1.5708,0.0,-1.57]])
    right_arm.wait()
    time.sleep(2)
    vel= VelocityNode()
    vel.velocity_setter()
    #right_arm=sss.move("arm_right","home")
    right_arm.wait()
    time.sleep(3)
    #torso=sss.move("torso","front")
    #torso.wait()
    #obj_name="can"
    #position_can,orientation=get_pose("vicon/"+obj_name+"/"+obj_name)
    obj_name="Ball"
    position_box,orientation=distance("vicon/"+obj_name+"/"+obj_name)
    loss=position_box[0]**2+position_box[1]**2
    print("loss: ",loss)