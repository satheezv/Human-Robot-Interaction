#!/usr/bin/env python

#MIT License
#Copyright (c) 2017 Massimiliano Patacchiola
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

#In this example I will use the class gridworld to generate a 3x4 world
#in which the cleaning robot will move. Using the SARSA algorithm I
#will estimate the state-action matrix.

import numpy as np
from base_gridworld import GridWorld
import time
from base_csp_test import *
from base_gridworld import GridWorld
from copy import deepcopy
def update_state_action(state_action_matrix, visit_counter_matrix, observation, new_observation, 
                   action, new_action, reward, alpha, gamma):
    '''Return the updated utility matrix

    @param state_action_matrix the matrix before the update
    @param observation the state obsrved at t
    @param new_observation the state observed at t+1
    @param action the action at t
    @param new_action the action at t+1
    @param reward the reward observed after the action
    @param alpha the ste size (learning rate)
    @param gamma the discount factor
    @return the updated state action matrix
    '''
    #Getting the values of Q at t and at t+1
    col = observation[1] + (observation[0]*13)
    q = state_action_matrix[action, col]
    col_t1 = new_observation[1] + (new_observation[0]*13)
    q_t1 = state_action_matrix[int(new_action) ,col_t1]
    #Calculate alpha based on how many time it
    #has been visited
    alpha_counted = 1.0 / (1.0 + visit_counter_matrix[action, col])
    #Applying the update rule
    #Here you can change "alpha" with "alpha_counted" if you want
    #to take into account how many times that particular state-action
    #pair has been visited until now.
    state_action_matrix[action ,col] = state_action_matrix[action ,col] + alpha * (reward + gamma * q_t1 - q)
    return state_action_matrix

def update_visit_counter(visit_counter_matrix, observation, action):
    '''Update the visit counter
   
    Counting how many times a state-action pair has been 
    visited. This information can be used during the update.
    @param visit_counter_matrix a matrix initialised with zeros
    @param observation the state observed
    @param action the action taken
    '''
    col = observation[1] + (observation[0]*4)
    visit_counter_matrix[action ,col] += 1.0
    return visit_counter_matrix

def update_policy(policy_matrix, state_action_matrix, observation):
    '''Return the updated policy matrix

    @param policy_matrix the matrix before the update
    @param state_action_matrix the state-action matrix
    @param observation the state obsrved at t
    @return the updated state action matrix
    '''
    col = observation[1] + (observation[0]*13)
    #Getting the index of the action with the highest utility
    best_action = np.argmax(state_action_matrix[:, col])
    #Updating the policy
    policy_matrix[observation[0], observation[1]] = best_action
    return policy_matrix

def return_epsilon_greedy_action(policy_matrix, observation, epsilon=0.1):
    '''Return an action choosing it with epsilon-greedy

    @param policy_matrix the matrix before the update
    @param observation the state obsrved at t
    @param epsilon the value used for computing the probabilities
    @return the updated policy_matrix
    '''
    tot_actions = int(np.nanmax(policy_matrix) + 1)
    action = int(policy_matrix[observation[0], observation[1]])
    non_greedy_prob = epsilon / tot_actions
    greedy_prob = 1 - epsilon + non_greedy_prob
    weight_array = np.full((tot_actions), non_greedy_prob)
    weight_array[action] = greedy_prob
    return np.random.choice(tot_actions, 1, p=weight_array)

def print_policy(policy_matrix):
    '''Print the policy using specific symbol.

    * terminal state
    ^ > v < up, right, down, left
    # obstacle
    '''
    counter = 0
    shape = policy_matrix.shape
    policy_string = ""
    for row in range(shape[0]):
        for col in range(shape[1]):
            if(policy_matrix[row,col] == -1): policy_string += " **  "            
            elif(policy_matrix[row,col] == 0): policy_string += " B0  "
            elif(policy_matrix[row,col] == 1): policy_string += " B1  "
            elif(policy_matrix[row,col] == 2): policy_string += " B2  "
            elif(policy_matrix[row,col] == 3): policy_string += " B3  "
            elif(policy_matrix[row,col] == 4): policy_string += " B4  "
            elif(policy_matrix[row,col] == 5): policy_string += " B5  "
            elif(policy_matrix[row,col] == 6): policy_string += " B6  "
            elif(policy_matrix[row,col] == 7): policy_string += " B7  "
            elif(policy_matrix[row,col] == 8): policy_string += " B8  "
            #elif(policy_matrix[row,col] == 3): policy_string += " <  "
            elif(np.isnan(policy_matrix[row,col])): policy_string += " #   "
            counter += 1
        policy_string += '\n'
    print(policy_string)

def return_decayed_value(starting_value, global_step, decay_step):
        """Returns the decayed value.

        decayed_value = starting_value * decay_rate ^ (global_step / decay_steps)
        @param starting_value the value before decaying
        @param global_step the global step to use for decay (positive integer)
        @param decay_step the step at which the value is decayed
        """
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
    env = GridWorld(6, 13)

    #Define the state matrix
    state_matrix = np.zeros((6,13))
    state_matrix[destination] = 1
    # for obs_sm in range(len(other_agent)):
    #     obst_statmat=other_agent[obs_sm]
    #     state_matrix[obst_statmat]=-1
    print("State Matrix:")
    print(state_matrix)

    #Define the reward matrix
    r1 = np.full((6,13), -0.1)
    r1[destination]= 1.5
    # for obs_rew in range(len(other_agent)):
    #     obst_rewmat=other_agent[obs_rew]
    #     r1[obst_rewmat]= -100
    print("Reward Matrix:")
    reward_matrix=r1
    print(reward_matrix)

    #Define the transition matrix
    transition_matrix = np.eye(9)

    #Random policy
    policy_matrix = np.random.randint(low=0, high=9, size=(6,13)).astype(np.float32)
    policy_matrix[destination] =-1 #No action for the terminal states
    # for obs_pol in range(len(other_agent)):
    #     obst_polmat=other_agent[obs_pol]
    #     policy_matrix[obst_polmat]= np.NaN
    print("Policy Matrix:")
    print(policy_matrix)

    env.setStateMatrix(state_matrix)
    env.setRewardMatrix(reward_matrix)
    env.setTransitionMatrix(transition_matrix)

    #utility_matrix = np.zeros((3,4))
    state_action_matrix = np.zeros((9,6*13))
    visit_counter_matrix = np.zeros((9,6*13))
    gamma = 0.999
    alpha = 0.001 #constant step size
    tot_epoch = 50000
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
            #Take the action from the action matrix
            #action = policy_matrix[observation[0], observation[1]]
            #Take the action using epsilon-greedy
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
    """
    print("State-Action matrix after " + str(tot_epoch) + " iterations:")
    print(state_action_matrix)
    print("Policy matrix after " + str(tot_epoch) + " iterations:")
    print_policy(policy_matrix)
    end_time=time.time()
    exe_time=end_time - start_time
    print(exe_time)"""
    end_time=time.time()
    exe_time=end_time - start_time
    return policy_matrix,exe_time,utility_val,episod

# agent 1 parameters
all_policies_agent1=[]
all_exe_time_agent1=[]
all_iteration=[]
uv_agent1=[]
episodes_agent1=[]


# # agent 1 goal & obst and states to avoid
goals_agent1=ocurrent_position.copy()
# obstacles_agent1=icurrent_position.copy()
# obs_homepos_add=(0,0)
# obstacles_agent1.insert(0,obs_homepos_add)

# # reducing difference between goal positions and obstacle positions of agent 1
# if len(goals_agent1)!=len(obstacles_agent1):
#     if len(goals_agent1)>len(obstacles_agent1):
#         difference=len(goals_agent1)-len(obstacles_agent1)
#         last=len(obstacles_agent1)
#         for d in range(difference):
#             obstacles_agent1.append(obstacles_agent1[last-1])

# # states to avoid (neighbouring states)
state_avoid_agent1=[(0,12),(0,12),(0,12)]
# for a in range(len(obstacles_agent1)):
#     obst_state=obstacles_agent1[a]
#     #if obst_state[0]>0 and obst_state[0]<5:
#         #if obst_state[1]>0 and obst_state[1]<12:
#     state_avoid_agent1.append([])
#     state_avoid_agent1[a].append(obst_state)
#     state1=(obst_state[0]+1,obst_state[1]-1)
#     if state1[0]>=0 and state1[0]<=5:
#         if state1[1]>=0 and state1[1]<=12:
#             state_avoid_agent1[a].append(state1)
#     state2=(obst_state[0]+1,obst_state[1])
#     if state2[0]>=0 and state2[0]<=5:
#         if state2[1]>=0 and state2[1]<=12:
#             state_avoid_agent1[a].append(state2)
#     state3=(obst_state[0]+1,obst_state[1]+1)
#     if state3[0]>=0 and state3[0]<=5:
#         if state3[1]>=0 and state3[1]<=12:
#             state_avoid_agent1[a].append(state3)
#     state4=(obst_state[0],obst_state[1]+1)
#     if state4[0]>=0 and state4[0]<=5:
#         if state4[1]>=0 and state4[1]<=12:
#             state_avoid_agent1[a].append(state4)
#     state5=(obst_state[0]-1,obst_state[1]+1)
#     if state5[0]>=0 and state5[0]<=5:
#         if state5[1]>=0 and state5[1]<=12:
#             state_avoid_agent1[a].append(state5)
#     state6=(obst_state[0]-1,obst_state[1])
#     if state6[0]>=0 and state6[0]<=5:
#         if state6[1]>=0 and state6[1]<=12:
#             state_avoid_agent1[a].append(state6)
#     state7=(obst_state[0]-1,obst_state[1]-1)
#     if state7[0]>=0 and state7[0]<=5:
#         if state7[1]>=0 and state7[1]<=12:
#             state_avoid_agent1[a].append(state7)
#     state8=(obst_state[0],obst_state[1]-1)
#     if state8[0]>=0 and state8[0]<=5:
#         if state8[1]>=0 and state8[1]<=12:
#             state_avoid_agent1[a].append(state8)    

for i in range(len(goals_agent1)):
    if i==0:
        #policy,e_time,iteration=policy_maker(goals[i])
        policy_a1,e_time_a1,utility_value_a1,episod_a1=policy_maker(goals_agent1[i],state_avoid_agent1[i])
    if i>0:
        if goals_agent1[i]!=goals_agent1[i-1]:
            #policy,e_time,iteration=policy_maker(goals[i])
            policy_a1,e_time_a1,utility_value_a1,episod_a1=policy_maker(goals_agent1[i],state_avoid_agent1[i])

    #agent 1 updates
    all_policies_agent1.append(policy_a1)
    uv_agent1.append(utility_value_a1)
    episodes_agent1.append(episod_a1)
    all_exe_time_agent1.append(e_time_a1)
    print (policy_a1)

np.save("allpolicies_agent1.npy",all_policies_agent1)
np.save("goals_agent1.npy",goals_agent1)
np.save("utility_value_agent1",uv_agent1)
np.save("episodes_agent1",episodes_agent1)

# # agent 2 parameters
# all_policies_agent2=[]
# all_exe_time_agent2=[]
# all_iteration=[]
# uv_agent2=[]
# episodes_agent2=[]

# # agent 2 goal & obst...
# goals_agent2=icurrent_position.copy()
# obstacles_agent2=ocurrent_position.copy()


# # states to avoid (neighbouring states)
# state_avoid_agent2=[]
# for a in range(len(obstacles_agent2)):
#     obst_state=obstacles_agent2[a]
#     state_avoid_agent2.append([])
#     state_avoid_agent2[a].append(obst_state)
#     state1=(obst_state[0]+1,obst_state[1]-1)
#     if state1[0]>=0 and state1[0]<=5:
#         if state1[1]>=0 and state1[1]<=12:
#             state_avoid_agent2[a].append(state1)
#     state2=(obst_state[0]+1,obst_state[1])
#     if state2[0]>=0 and state2[0]<=5:
#         if state2[1]>=0 and state2[1]<=12:
#             state_avoid_agent2[a].append(state2)
#     state3=(obst_state[0]+1,obst_state[1]+1)
#     if state3[0]>=0 and state3[0]<=5:
#         if state3[1]>=0 and state3[1]<=12:
#             state_avoid_agent2[a].append(state3)
#     state4=(obst_state[0],obst_state[1]+1)
#     if state4[0]>=0 and state4[0]<=5:
#         if state4[1]>=0 and state4[1]<=12:
#             state_avoid_agent2[a].append(state4)
#     state5=(obst_state[0]-1,obst_state[1]+1)
#     if state5[0]>=0 and state5[0]<=5:
#         if state5[1]>=0 and state5[1]<=12:
#             state_avoid_agent2[a].append(state5)
#     state6=(obst_state[0]-1,obst_state[1])
#     if state6[0]>=0 and state6[0]<=5:
#         if state6[1]>=0 and state6[1]<=12:
#             state_avoid_agent2[a].append(state6)
#     state7=(obst_state[0]-1,obst_state[1]-1)
#     if state7[0]>=0 and state7[0]<=5:
#         if state7[1]>=0 and state7[1]<=12:
#             state_avoid_agent2[a].append(state7)
#     state8=(obst_state[0],obst_state[1]-1)
#     if state8[0]>=0 and state8[0]<=5:
#         if state8[1]>=0 and state8[1]<=12:
#             state_avoid_agent2[a].append(state8)  


# for i in range(len(goals_agent2)):
#     if i==0:
#         #policy,e_time,iteration=policy_maker(goals[i])
#         policy_a2,e_time_a2,utility_value_a2,episod_a2=policy_maker(goals_agent2[i],state_avoid_agent2[i])
#     if i>0:
#         if goals_agent2[i]!=goals_agent2[i-1]:
#             #policy,e_time,iteration=policy_maker(goals[i])
#             policy_a2,e_time_a2,utility_value_a2,episod_a2=policy_maker(goals_agent2[i],state_avoid_agent2[i])
#     #agent 2 updates
#     all_policies_agent2.append(policy_a2)
#     uv_agent2.append(utility_value_a2)
#     episodes_agent2.append(episod_a2)
#     all_exe_time_agent2.append(e_time_a2)
#     print (policy_a2)
# np.save("allpolicies_agent2.npy",all_policies_agent2)
# np.save("goals_agent2.npy",goals_agent2)
# np.save("utility_value_agent2",uv_agent2)
# np.save("episodes_agent2",episodes_agent2)

