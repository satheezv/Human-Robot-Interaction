#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import math
from simple_script_server import *
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import Float64MultiArray,MultiArrayDimension 
import threading

class VelocityNode:
    def __init__(self):
        self.velocity=0
        self.velocities=[]
        self.msg=Float64MultiArray()
        self.dim = MultiArrayDimension()
        self.open_gripper=threading.Thread(target=self.open_hand)
        self.actual_pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.goal_pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.k=[50.0,0.1,0.1,0.1,0.1,0.1,0.1]
        self.p=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        self.i=[0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        self.pub = rospy.Publisher('/arm_right/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)
        self.pub_pos = rospy.Publisher('/arm_right/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        self.pub_pos_gripper = rospy.Publisher('/gripper_right/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        self.pub_torso = rospy.Publisher('/torso/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/arm_right/joint_states', JointState, self.read_joints)
        self.triger=False
    def start(self):
        rospy.spin()
    def read_joints(self,data):
        self.actual_pos= data.position
        error=1.5-self.actual_pos[0]
        if error >0:
            self.triger=True
        if self.triger:
            u=self.k[0]*error
            self.msg.data = [u,0.0,0.0,0.0,0.0,0.0,0.0]
            self.dim.size = len(self.msg.data)
            self.dim.label = "command"
            self.dim.stride = len(self.msg.data)
            self.msg.layout.dim.append(self.dim)
            self.msg.layout.data_offset = 0
            self.pub.publish(self.msg)
        else:
            self.msg.data = [2.0,0.0,0.0,0.0,0.0,0.0,0.0]
            self.dim.size = len(self.msg.data)
            self.dim.label = "command"
            self.dim.stride = len(self.msg.data)
            self.msg.layout.dim.append(self.dim)
            self.msg.layout.data_offset = 0
            self.pub.publish(self.msg)
        print(self.actual_pos[0], u)
    def open_hand(self):
        #right_gripper=sss.move("gripper_right","open")
        self.msg.data = [0.5,0.5]
        self.dim.size = len(self.msg.data)
        self.dim.label = "command"
        self.dim.stride = len(self.msg.data)
        self.msg.layout.dim.append(self.dim)
        self.msg.layout.data_offset = 0
        self.pub_pos_gripper.publish(self.msg)
    def velocity_setter_1(self,velocity,torso_pos,open_time):
        for joint in range(0,len(self.actual_pos)):
            if self.actual_pos[joint]>self.goal_pos[joint]:
                pass
                #PID
            else:
                pass
                #keep velocity
            ##Publish velocities
        return
    def velocity_setter(self,velocity,torso_pos,open_time):
        frecc=10
        
        current_time=0.0
        rate = rospy.Rate(frecc) # 10hz
        theta_p=[velocity,0.0,0.0,0.0,0.0,0.0,0.0]
        theta_i=[1.5708,0.0,2.0,-1.5708,1.5708,0.0,0.0,torso_pos]
        theta_f=[0.0,0.0,0.0,-1.5708,1.5708,0.0,0.0,0.0]
        total_time=abs((theta_f[0]-theta_i[0])/theta_p[0])
        delta_t=1.0/frecc
        steps=total_time/delta_t
        step=0
        #for joint in range(1,7):
        #    theta_p[joint]=(theta_f[joint]-theta_i[joint])/total_time
        open=True
        while not rospy.is_shutdown():
            step+=1
            theta_p_t=[]
            for joint in range(0,len(theta_i)):
                theta_p_t.append(theta_i[joint]+((theta_f[joint]-theta_i[joint])/steps)*step)
            current_time+=delta_t
            #print(theta_p_t)
            #print(current_time)
            #print(step)
            if current_time>=total_time:
                break
            else:
                #self.msg.data = [angle,0.0,angle,0.0,angle,0.0,0.0]
                self.msg.data = theta_p_t[:-1]
                self.dim.size = len(self.msg.data)
                self.dim.label = "command"
                self.dim.stride = len(self.msg.data)
                self.msg.layout.dim.append(self.dim)
                self.msg.layout.data_offset = 0
                self.pub_pos.publish(self.msg)
                ####torso
                self.msg.data = [0.0,theta_p_t[7]]
                self.dim.size = len(self.msg.data)
                self.dim.label = "command"
                self.dim.stride = len(self.msg.data)
                self.msg.layout.dim.append(self.dim)
                self.msg.layout.data_offset = 0
                self.pub_torso.publish(self.msg)
            rate.sleep()        
if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    # sss = simple_script_server()
    # sss.init("gripper_right")
    # sss.recover("gripper_right")
    # sss.init("arm_right")
    # sss.recover("arm_right")
    # sss.init("torso")
    # sss.recover("torso")
    # torso=sss.move("torso","front")
    # torso.wait()
    # torso=sss.move("torso","right")
    # torso.wait()
    # right_arm=sss.move("arm_right","home")
    # right_arm.wait()
    # right_arm=sss.move("arm_right",[[1.5708,0.0,2.0,-1.5708,1.5708,0.0,0.0]])
    # right_arm.wait()
    # torso=sss.move("torso",[[0.0,-0.7]])
    # torso.wait()
    # right_gripper=sss.move("gripper_right","open")
    # time.sleep(2)
    #right_gripper=sss.move("gripper_right","home")
    vel= VelocityNode()
    vel.start()
    ###
    #vel.velocity_setter(0.6,-0.7,0.5)
    ###
    # right_arm=sss.move("arm_right","home")
    # right_arm.wait()
    # torso=sss.move("torso","front")
    # torso.wait()

