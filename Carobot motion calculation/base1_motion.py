# -*- coding: utf-8 -*-
"""
Created on Wed May  8 18:45:47 2019

@author: Admin
"""
import numpy as np
from coordinates import *

def next_pos(action):
    if action == 1:
        np1=current_position[0]-1,current_position[1]
    elif action == 2:
        np1=current_position[0]-1,current_position[1]+1
    elif action == 3:
        np1=current_position[0],current_position[1]+1
    elif action == 4:
        np1=current_position[0]+1,current_position[1]+1
    elif action == 5:
        np1=current_position[0]+1,current_position[1]
    elif action == 6:
        np1=current_position[0]+1,current_position[1]-1
    elif action == 7:
        np1=current_position[0],current_position[1]-1
    elif action == 8:
        np1=current_position[0]-1,current_position[1]-1
    elif action == -1:
        np1=current_position[0],current_position[1]
    return np1

def odd_even(location):
    if location % 2==0:
        movement="even-LT "   #even-lower triangle
        print ("current_location is " + str(current_location) + " (even)")
    elif location % 2!=0:
        movement="odd-UT "    #odd-upper triangle
        print ("current_location is " + str(current_location) + " (odd)")
    return movement
home=[0,0]
agent1_positions=[]
agent1_actions=[]
agent1_locations=[]
agent1_movements=[]
agent1_policies=np.load("allpolicies_agent1.npy")
agent1_goals=np.load("goals_agent1.npy")
agent1_goals=agent1_goals.tolist()
print (agent1_goals)
xi=[]
for j in range (len(agent1_policies)):
    policy= agent1_policies[j] 
    current_position=home
    agent1_positions.append([])
    agent1_actions.append([])
    agent1_locations.append([])
    agent1_movements.append([])    
    iteration=1
    while True:
        print ("step : " +str(iteration))
        if iteration==1:
            agent1_positions[j].append(current_position)
        action=int(policy[current_position[0],current_position[1]])
        current_location=(int(current_position[0])*13)+int(current_position[1])+1
        #agent1_locations[j].append(current_location)
        print ("current position is " +str(current_position))
        movement=odd_even(current_location)
        agent1_movements[j].append(movement)
        print ("movement is " + str(movement) + "act " + str(action))
        #print "action is " + str(action)    
        next_position=next_pos(action)
        next_location=(int(next_position[0])*13)+int(next_position[1])+1
        agent1_locations[j].append(next_location)
        print ("next_location is " + str(next_location))
        print ("next position is " +str(next_position))
        agent1_actions[j].append(action) 
        if int(policy[next_position])==-1:
            break
        current_position=next_position
        agent1_positions[j].append(current_position)
        iteration+=1
        xi.append(iteration)
    home=agent1_goals[j]

###### this needs to be changed
# n=len(agent1_positions)
# agent1_positions.append(agent1_positions[n-1])
# agent1_locations.append(agent1_locations[n-1])

# base centroids coordinates calculation 
agent1_intermediate_centroid_x=[] 
agent1_intermediate_centroid_y=[] 
agent1_goal_centroid_x=[] 
agent1_goal_centroid_y=[] 
agent1_intermediate_locations=[]
agent1_goal_locations=[]
for a in range(len(agent1_locations)):
    for b in range(len(agent1_locations[a])):
        if b==0:
            if  1<=agent1_locations[a][b]<=13:
                actual_location=agent1_locations[a][b]+65
            elif  14<=agent1_locations[a][b]<=26:
                actual_location=agent1_locations[a][b]+39
            elif  27<=agent1_locations[a][b]<=39:
                actual_location=agent1_locations[a][b]+13
            elif  40<=agent1_locations[a][b]<=52:
                actual_location=agent1_locations[a][b]-13
            elif  53<=agent1_locations[a][b]<=65:
                actual_location=agent1_locations[a][b]-39
            elif  66<=agent1_locations[a][b]<=78:
                actual_location=agent1_locations[a][b]-65
            agent1_goal_locations.append(actual_location)
            agent1_goal_centroid_x.append(A[actual_location-1])
            agent1_goal_centroid_y.append(B[actual_location-1])
        elif b!=0:
            if  1<=agent1_locations[a][b]<=13:
                actual_location=agent1_locations[a][b]+65
            elif  14<=agent1_locations[a][b]<=26:
                actual_location=agent1_locations[a][b]+39
            elif  27<=agent1_locations[a][b]<=39:
                actual_location=agent1_locations[a][b]+13
            elif  40<=agent1_locations[a][b]<=52:
                actual_location=agent1_locations[a][b]-13
            elif  53<=agent1_locations[a][b]<=65:
                actual_location=agent1_locations[a][b]-39
            elif  66<=agent1_locations[a][b]<=78:
                actual_location=agent1_locations[a][b]-65
            agent1_intermediate_locations.append(actual_location)
            agent1_intermediate_centroid_x.append(A[actual_location-1])
            agent1_intermediate_centroid_y.append(B[actual_location-1])



