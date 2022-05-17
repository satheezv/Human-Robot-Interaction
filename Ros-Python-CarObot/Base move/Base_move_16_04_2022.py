#!/usr/bin/env python


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geometry_msgs.msg  
import yaml
import time
import tf
import roslib
import math

#roslib.load_manifest('learning_tf')
#a1:down,a2:down_left, a3:left, a4:up_left, a5: up, a6:up_right, a7:right,a8:down_right


action_space={ 'state_1':{'a1':'state_4','a2':'state_5','a3':'state_2','a4':None,'a5':None,'a6':None,'a7':None,'a8':None},
'state_2':{'a1':'state_5','a2':'state_6','a3':'state_3','a4':None,'a5':None,'a6':None,'a7':'state_1','a8':'state_4'},
'state_3':{'a1':'state_6','a2':None,'a3':None,'a4':None,'a5':None,'a6':None,'a7':'state_2','a8':'state_5'},
'state_4':{'a1':'state_7','a2':'state_8','a3':'state_5','a4':'state_2','a5':'state_1','a6':None,'a7':None,'a8':None},
'state_5':{'a1':'state_8','a2':'state_9','a3':'state_6','a4':'state_3','a5':'state_2','a6':'state_1','a7':'state_4','a8':'state_7'},
'state_6':{'a1':'state_9','a2':None,'a3':None,'a4':None,'a5':'state_3','a6':'state_2','a7':'state_5','a8':'state_8'},
'state_7':{'a1':'state_10','a2':'state_11','a3':'state_8','a4':'state_5','a5':'state_4','a6':None,'a7':None,'a8':None},
'state_8':{'a1':'state_11','a2':'state_12','a3':'state_9','a4':'state_6','a5':'state_5','a6':'state_4','a7':'state_7','a8':'state_10'},
'state_9':{'a1':'state_12','a2':None,'a3':None,'a4':None,'a5':'state_6','a6':'state_5','a7':'state_8','a8':'state_11'},


'state_10':{'a1':'state_13','a2':'state_14','a3':'state_11','a4':'state_8','a5':'state_7','a6':None,'a7':None,'a8':None},
'state_11':{'a1':'state_14','a2':'state_15','a3':'state_12','a4':'state_9','a5':'state_8','a6':'state_7','a7':'state_10','a8':'state_13'},
'state_12':{'a1':'state_15','a2':None,'a3':None,'a4':None,'a5':'state_9','a6':'state_8','a7':'state_11','a8':'state_14'},

'state_13':{'a1':'state_16','a2':'state_17','a3':'state_14','a4':'state_11','a5':'state_10','a6':None,'a7':None,'a8':None},
'state_14':{'a1':'state_17','a2':'state_18','a3':'state_15','a4':'state_12','a5':'state_11','a6':'state_10','a7':'state_13','a8':'state_16'},
'state_15':{'a1':'state_18','a2':None,'a3':None,'a4':None,'a5':'state_12','a6':'state_11','a7':'state_14','a8':'state_17'},

'state_16':{'a1':None,'a2':None,'a3':'state_17','a4':'state_14','a5':'state_13','a6':None,'a7':None,'a8':None},
'state_17':{'a1':None,'a2':None,'a3':'state_18','a4':'state_15','a5':'state_14','a6':'state_13','a7':'state_16','a8':None},
'state_18':{'a1':None,'a2':None,'a3':None,'a4':None,'a5':'state_15','a6':'state_14','a7':'state_17','a8':None},

}



def get_pose(ref,frame):
    listener = tf.TransformListener()
    trans = []
    rot = []
    
    rate = rospy.Rate(10.0)
    listener.waitForTransform(ref, frame,rospy.Time(), rospy.Duration(4.0))
    while not trans:
        try:
            (trans,rot) = listener.lookupTransform(ref, frame,rospy.Time())
        except Exception as e:
            print(e)
        rate.sleep()
    #print ('Translation: ' , trans)
    #print ('Rotation: ' , rot)
    #print(frame[-1:])

    # while True:
    #     if listener.frameExists(frame[1:]) and listener.frameExists(ref[1:]):
    #             try:
    #                 t = listener.getLatestCommonTime(ref, frame)
    #                 p1 = geometry_msgs.msg.PoseStamped()
    #                 p1.header.frame_id = frame[1:]
    #                 p1.pose.orientation.w = 1.0    # Neutral orientation
    #                 now = rospy.Time.now()
    #                 p_in_base = listener.transformPose("/base_link", p1,now)
    #                 print ("Position of the fingertip in the robot base:")
    #                 trans=[p_in_base.pose.position.x,p_in_base.pose.position.y,p_in_base.pose.position.z]
    #                 print (p_in_base.pose.position.x,p_in_base.pose.position.y)
    #                 break
    #             except Exception as e:
    #                 print(e)
    #                 continue
    return trans,rot

def set_goal(x,y,z,w):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map_corrected"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x 
    goal.target_pose.pose.position.y = y 
    goal.target_pose.pose.orientation.w = w
    client.send_goal(goal)
def step(next_state):
    action(next_state[0],next_state[1],0.0,1.0)
    return

def action(x,y,z,w):
    # client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # client.wait_for_server()
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "base_link"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = x 
    # goal.target_pose.pose.position.y = y 
    # goal.target_pose.pose.orientation.w = w
    # client.send_goal(goal)

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def move_to_state():
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/state_1', '/state_4', rospy.Time(0))
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            client.wait_for_server()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = trans[0]
            goal.target_pose.pose.position.y = trans[1] 
            goal.target_pose.pose.orientation.w = rot[2]
        except Exception as e:
            print(e)
            continue
        
        
        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # velocity.publish(turtlesim.msg.Velocity(linear, angular))

        rate.sleep()
   

if __name__ == '__main__':
    rospy.init_node('Base_move')  

    while not rospy.is_shutdown(): 
        dis_from_state=[]
        for state in range(1,19):
            trans,rot = get_pose('/base_link', '/state_'+str(state))
            dis_from_state.append(math.sqrt((trans[0]**2)+(trans[1]**2)))
            #print(state,math.sqrt((trans[0]**2)+(trans[1]**2)))
            #time.sleep(0.5)
        state="state_"+str(dis_from_state.index(min(dis_from_state))+1)
        #send robot to that state
        print("Current state state_"+str(dis_from_state.index(min(dis_from_state))+1))
        #step(state)
        print("Actions:",action_space[state])
        actions=action_space[state]
        next_state=actions['a1']
        print(next_state)
        if next_state is not None:
            print("/"+next_state)
            trans,rot = get_pose('/map', "/"+next_state)
            step(trans)
            ##Go to next_state
        break

        #current_state='state_5'
        #act=a1
        #Target_state=action_space[current_state][a1]
        #trans,rot = get_pose('/map', Target_state)
        #a1 = action(trans[0],trans[1],0.0,1.0)



    # while not rospy.is_shutdown():
    #     for s in range(1,18):
    #         trans,rot = get_pose('/map', '/state_'+str(s))
    #         print(trans)
    #         a1 = action_space(trans[0],trans[1],0.0,1.0)
    #     break
