# -*- coding: utf-8 -*-
"""
Created on Thu Apr 28 10:49:50 2022

@author: scesv7
"""

#!/usr/bin/env python


import numpy as np
from base_gridworld import GridWorld
import time
#from base_csp_test import *
from copy import deepcopy

def update_state_action(state_action_matrix, visit_counter_matrix, observation, new_observation, 
                   action, new_action, reward, alpha, gamma):

    #Getting the values of Q at t and at t+1
    col = observation[1] + (observation[0]*3)
    q = state_action_matrix[action, col]
    col_t1 = new_observation[1] + (new_observation[0]*3)
    q_t1 = state_action_matrix[int(new_action) ,col_t1]
    #Calculate alpha based on how many time it
    #has been visited
    alpha_counted = 1.0 / (1.0 + visit_counter_matrix[action, col])

    state_action_matrix[action ,col] = state_action_matrix[action ,col] + alpha * (reward + gamma * q_t1 - q)
    return state_action_matrix

def update_visit_counter(visit_counter_matrix, observation, action):

    col = observation[1] + (observation[0]*3)
    visit_counter_matrix[action ,col] += 1.0
    return visit_counter_matrix

def update_policy(policy_matrix, state_action_matrix, observation):

    col = observation[1] + (observation[0]*3)
    #Getting the index of the action with the highest utility
    best_action = np.argmax(state_action_matrix[:, col])
    #Updating the policy
    policy_matrix[observation[0], observation[1]] = best_action
    return policy_matrix

def return_epsilon_greedy_action(policy_matrix, observation, epsilon=0.1):

    tot_actions = int(np.nanmax(policy_matrix) + 1)
    action = int(policy_matrix[observation[0], observation[1]])
    non_greedy_prob = epsilon / tot_actions
    greedy_prob = 1 - epsilon + non_greedy_prob
    weight_array = np.full((tot_actions), non_greedy_prob)
    weight_array[action] = greedy_prob
    return np.random.choice(tot_actions, 1, p=weight_array)

def print_policy(policy_matrix):

    counter = 0
    shape = policy_matrix.shape
    policy_string = ""
    for row in range(shape[0]):
        for col in range(shape[1]):
            if(policy_matrix[row,col] == -1): policy_string += " **  "            
            elif(policy_matrix[row,col] == 0): policy_string += " A0  "
            elif(policy_matrix[row,col] == 1): policy_string += " A1  "
            elif(policy_matrix[row,col] == 2): policy_string += " A2  "
            elif(policy_matrix[row,col] == 3): policy_string += " A3  "
            elif(policy_matrix[row,col] == 4): policy_string += " A4  "
            elif(policy_matrix[row,col] == 5): policy_string += " A5  "
            elif(policy_matrix[row,col] == 6): policy_string += " A6  "
            elif(policy_matrix[row,col] == 7): policy_string += " A7  "
            elif(policy_matrix[row,col] == 8): policy_string += " A8  "
            #elif(policy_matrix[row,col] == 3): policy_string += " <  "
            elif(np.isnan(policy_matrix[row,col])): policy_string += " #   "
            counter += 1
        policy_string += '\n'
    print(policy_string)

def return_decayed_value(starting_value, global_step, decay_step):

        decayed_value = starting_value * np.power(0.1, (global_step/decay_step))
        return decayed_value


def policy_maker(destination,other_agent):
    policy_matrices=[]
    policy_array=[]
    iter_segment=[] #Number of heads
    exe_time=[]
    ret_val=[]   # return value plot
    ret_val2=[]
    utility_val=[]
    policy_plot=[]
    avg_rew=[]

    start_time = time.time()
    env = GridWorld(6, 3)

    #Define the state matrix
    state_matrix = np.zeros((6,3))
    state_matrix[destination] = 1
    for obs_sm in range(len(other_agent)):
        obst_statmat=other_agent[obs_sm]
        state_matrix[obst_statmat]=-1
    print("State Matrix:")
    print(state_matrix)

    #Define the reward matrix
    r1 = np.full((6,3), -0.1)
    r1[destination]= 1.5
    for obs_rew in range(len(other_agent)):
        obst_rewmat=other_agent[obs_rew]
        r1[obst_rewmat]= -100
    print("Reward Matrix:")
    reward_matrix=r1
    print(reward_matrix)

    #Define the transition matrix
    transition_matrix = np.eye(9)

    #Random policy
    policy_matrix = np.random.randint(low=0, high=9, size=(6,3)).astype(np.float32)
    policy_matrix[destination] =-1 #No action for the terminal states
    for obs_pol in range(len(other_agent)):
        obst_polmat=other_agent[obs_pol]
        policy_matrix[obst_polmat]= np.NaN
    print("Policy Matrix:")
    print(policy_matrix)

    env.setStateMatrix(state_matrix)
    env.setRewardMatrix(reward_matrix)
    env.setTransitionMatrix(transition_matrix)

    #utility_matrix = np.zeros((3,4))
    state_action_matrix = np.zeros((9,6*3))
    visit_counter_matrix = np.zeros((9,6*3))
    gamma = 0.999
    alpha = 0.001 #constant step size
    tot_epoch = 10000
    print_epoch = 1000
    episod=[]
#####################################
    for epoch in range(tot_epoch):
        iteration=0
        epsilon = return_decayed_value(0.1, epoch, decay_step=100000)
        #Reset and return the first observation
        observation = env.reset(exploring_starts=True)
        is_starting = True 
        for step in range(1000):
            iteration+=1

            action = return_epsilon_greedy_action(policy_matrix, observation, epsilon=0.1)
            if(is_starting): 
                action = np.random.randint(0, 9)
                is_starting = False  
            #Move one step in the environment and get obs and reward
            new_observation, reward, done = env.step(action)
            new_action = policy_matrix[new_observation[0], new_observation[1]]
            #Updating the state-action matrix
            state_action_matrix = update_state_action(state_action_matrix, visit_counter_matrix, observation, new_observation, 
                                                      action, new_action, reward, alpha, gamma)
            
            #Updating the policy
            policy_matrix = update_policy(policy_matrix, state_action_matrix, observation)
            pol_mat=list(policy_matrix)
            #Increment the visit counter
            visit_counter_matrix = update_visit_counter(visit_counter_matrix, observation, action)
            observation = new_observation
            
            #print(utility_matrix)
            if done: break
        episod.append(iteration)
        if(epoch % print_epoch == 0):
            print("")
            print("Epsilon: " + str(epsilon))
            print("State-Action matrix after " + str(epoch+1) + " iterations:") 
            sam=state_action_matrix.copy()
            utility_val.append(sam)
            print(sam)
            print("Policy matrix after " + str(epoch+1) + " iterations:") 
            print_policy(policy_matrix)
    #Time to check the utility matrix obtained

    end_time=time.time()
    exe_time=end_time - start_time
    return policy_matrix,exe_time,utility_val,episod


def next_pos(action):
    if action == 1:
        np1=current_position[0]+1,current_position[1]
    elif action == 2:
        np1=current_position[0]+1,current_position[1]+1
    elif action == 3:
        np1=current_position[0],current_position[1]+1
    elif action == 4:
        np1=current_position[0]-1,current_position[1]+1
    elif action == 5:
        np1=current_position[0]-1,current_position[1]
    elif action == 6:
        np1=current_position[0]-1,current_position[1]-1
    elif action == 7:
        np1=current_position[0],current_position[1]-1
    elif action == 8:
        np1=current_position[0]+1,current_position[1]-1
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



# agent 1 parameters
all_policies_agent1=[]
all_exe_time_agent1=[]
all_iteration=[]
uv_agent1=[]
episodes_agent1=[]

# # CareObot home goal & obst and states to avoid
home=[0,0]
goals_agent1=(5,0)


# # states to avoid (neighbouring states)
state_avoid_agent1=[(4,0),(4,1)]

policy_a1,e_time_a1,utility_value_a1,episod_a1=policy_maker(goals_agent1,state_avoid_agent1)

agent1_positions=[]
agent1_actions=[]
agent1_locations=[]
agent1_movements=[]

xi=[]
# for j in range (len(agent1_policies)):
policy= policy_a1
current_position=home
action_sequence=[]
 
iteration=1
while True:
    print ("step : " +str(iteration))
    if iteration==1:
        agent1_positions.append(current_position)
    action=int(policy[current_position[0],current_position[1]])
    current_location=(int(current_position[0])*3)+int(current_position[1])+1
    #agent1_locations[j].append(current_location)
    print ("current position is " +str(current_position))
    movement=odd_even(current_location)
    agent1_movements.append(movement)
    print ("movement is " + str(movement) + "a" + str(action))
    action_sequence.append("a" + str(action))
    #print "action is " + str(action)    
    next_position=next_pos(action)
    next_location=(int(next_position[0])*3)+int(next_position[1])+1
    agent1_locations.append(next_location)
    print ("next_location is " + str(next_location))
    print ("next position is " +str(next_position))
    agent1_actions.append(action) 
    if int(policy[next_position])==-1:
        break
    current_position=next_position
    agent1_positions.append(current_position)
    iteration+=1
    xi.append(iteration)





        