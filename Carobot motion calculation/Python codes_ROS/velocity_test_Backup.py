#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import math
from simple_script_server import *
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import Float64MultiArray,MultiArrayDimension 

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
            if angle<=-70:
                print("Limit")
                self.msg.data = [0.0,0.0,0.0,-1.5708,1.5708,0.0,-1.57]
                self.dim.size = len(self.msg.data)
                self.dim.label = "command"
                self.dim.stride = len(self.msg.data)
                self.msg.layout.dim.append(self.dim)
                self.msg.layout.data_offset = 0
                self.pub_pos.publish(self.msg)
                self.msg.data = [0.0,0.0]
                self.dim.size = len(self.msg.data)
                self.dim.label = "command"
                self.dim.stride = len(self.msg.data)
                self.msg.layout.dim.append(self.dim)
                self.msg.layout.data_offset = 0
                self.pub_torso.publish(self.msg)
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
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
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
    right_arm=sss.move("arm_right","home")
    right_arm.wait()
    torso=sss.move("torso","front")
    torso.wait()

