#!/usr/bin/env python


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
import time
import tf



def get_pose(ref,frame):
    listener = tf.TransformListener()
    trans = []
    rot = []
    rate = rospy.Rate(10.0)
    while not trans:
        try:
            (trans,rot) = listener.lookupTransform(ref, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    #print ('Translation: ' , trans)
    #print ('Rotation: ' , rot)
    return trans,rot

def set_goal(x,y,z,w):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x + 0.1
    goal.target_pose.pose.position.y = y - 0.05
    goal.target_pose.pose.orientation.w = w
    client.send_goal(goal)

   
if __name__ == '__main__':
    rospy.init_node('Base_move')
    rate = rospy.Rate(1.0)
    set_goal(-1.0,-0.6,0.0,1.0)
    #set_goal(0.0,0.0,0.0,1.0)
    while not rospy.is_shutdown():
        traslation, rotation=get_pose('/map','/base_footprint')
        print("X: ",traslation[0],"Y: ",traslation[1])
        print("Rotation: ",rotation)
        time.sleep(2)
        rate.sleep()
