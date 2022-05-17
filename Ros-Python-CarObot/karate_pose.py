#!/usr/bin/env python

import rospy
import time
import yaml
from simple_script_server import *
import threading

arm_right_pose=[0.0,0.0,0.0,0.0,0.0,0.0,0]
arm_left_pose=[0.0,0.0,0.0,0.0,0.0,0.0,0]
#handle_torso = sss.move("torso","front")
sss = simple_script_server()

#def thread_right():
#    handle_rightarm = sss.move("arm_right",[arm_right_pose])
#    handle_rightarm.wait()
#    return
#def thread_left():
#    handle_leftarm = sss.move("arm_left",[arm_left_pose])
#    handle_leftarm.wait()
#    return
def poses():
    #rospy.init_node("goal_script")
    sss.init("torso")
    sss.recover("torso")
    handle_torso = sss.move("torso","front")

    #Karate salute
    sss.say("sound",["Karate Salute"], False)
    sss.set_mimic("mimic","angry")
    handle_gripper = sss.move("gripper_right","home",False)
    handle_rightarm = sss.move("arm_right","side",False)
    handle_leftarm = sss.move("arm_left","side")
    handle_gripper = sss.move("gripper_right","open", False)
    handle_rightarm = sss.move("arm_right",[[0.0,-1.0,0.0,-1.8,0.0,0.0,0.0]], False)
    handle_leftarm = sss.move("arm_left",[[0.0,0.98,0.0,1.8,0.0,0.0,0.0]])
    handle_leftarm.wait()
    handle_leftarm = sss.move("arm_left","side",False)
    handle_gripper = sss.move("gripper_right","close", False)
    handle_rightarm = sss.move("arm_right","side")
    handle_rightarm.wait()

    # #To begin pose
    sss.say("sound",["Block"], False)
    sss.say("sound",["kakiwake uke"], False)
    sss.set_mimic("mimic","angry")
    arm_right_pose="side"
    arm_left_pose="side"
    handle_leftarm = sss.move("arm_left",[arm_left_pose],False)
    handle_rightarm = sss.move("arm_right",[arm_right_pose])
    handle_rightarm.wait()
    arm_right_pose=[-0.8,-1.5,1.5,-1.3,0.0,0.0,0]
    arm_left_pose=[1.5,1.4,-1.5,1.9,0.0,0.0,0]
    handle_leftarm = sss.move("arm_left",[arm_left_pose],False)
    handle_rightarm = sss.move("arm_right",[arm_right_pose])
    handle_rightarm.wait()

    #Karate punch
    sss.say("sound",["straight punch"], False)
    sss.say("sound",["choku zuki"], False)
    sss.set_mimic("mimic","angry")
    handle_leftarm = sss.move("arm_left","side",False)
    handle_rightarm = sss.move("arm_right","side",False)
    handle_gripper = sss.move("gripper_right","close")
    handle_rightarm = sss.move("arm_right",[[0.0,-1.5,0.0,0.0,0.0,0.0,0.0]])
    handle_rightarm = sss.move("arm_right",[[-2.2,-1.3,1.5,-2.0,0.0,0.0,1.5]],False)
    handle_leftarm = sss.move("arm_left",[[0.0,1.5,0.0,0.0,0.0,0.0,0.0]])
    handle_leftarm = sss.move("arm_left",[[2.0,1.3,-1.5,2.0,0.0,0.0,-1.5]],False)
    handle_rightarm = sss.move("arm_right",[[0.0,-1.5,0.0,0.0,0.0,0.0,0.0]])
    handle_leftarm = sss.move("arm_left","side",False)
    handle_rightarm = sss.move("arm_right","side")

    #karate punch 2 
    sss.say("sound",["Advanced block"], False)
    sss.set_mimic("mimic","angry")
    handle_torso = sss.move("torso","front")
    arm_right_pose="side"
    arm_left_pose="side"
    handle_leftarm = sss.move("arm_left",[arm_left_pose],False)
    handle_rightarm = sss.move("arm_right",[arm_right_pose])
    handle_rightarm.wait()
    arm_right_pose=[0.5,-1.5,1.5,-0.9,0.0,0.0,1.5]
    arm_left_pose=[-3.0,1.0,1.5,2.4,0.0,0.0,-1.5]
    handle_leftarm = sss.move("arm_left",[arm_left_pose],False)
    handle_rightarm = sss.move("arm_right",[arm_right_pose],False)
    handle_torso = sss.move("torso","front_down")
    handle_rightarm.wait()
    handle_torso = sss.move("torso","front",False)
    handle_leftarm = sss.move("arm_left","side",False)
    handle_rightarm = sss.move("arm_right","side")
def start():
    goal={'goal':{'position':'home','ready':False}}
    with open(r'/home/care/moveit_ws/src/nav_goal_publisher/scripts/goal_karate.yaml', 'w') as file:
        documents = yaml.dump(goal, file)
if __name__ == '__main__':
    rospy.init_node("karate_script")    
    start() 
    while True:
            with open("/home/care/moveit_ws/src/nav_goal_publisher/scripts/goal_karate.yaml", 'r') as file:
                goal_position = yaml.load(file)
            if goal_position['goal']['ready']:
                poses()
                start()
            time.sleep(1)