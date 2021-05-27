#!/usr/bin/env python3

import rospy
import numpy as np
import os
import csv

class QLearning(object):
    def __init__(self):
        # Initialize node
        rospy.init_node("q_learning")

        # Initialize action matrix to hold state transitions
        self.init_action_matrix()

        # Initialize Q-Matrix for storing rewards calculated by algorithm
        self.q_matrix = np.zeros([31,2])
        self.q_matrix_path =  os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
       
        # Parameter to hold final state before exit 
        self.end_state = 25

        # Initialize Q-Learning parameters
        self.alpha = 1
        self.gamma = .65

        # Initialize parameters for detecting convergence
        self.convergence_count = 0
        self.convergence_max = 5000
        
        # Initialize parameters to track state/action
        self.curr_state = 0
        self.next_state = 0 
        self.curr_action = 0 


    def init_action_matrix(self):
        # 2^0 + 2^1 + 2^2 + 2^3 + 2^4 = 31 total theoretical states
        # 31x31 array initialized to -1
        self.action_matrix = -1 * np.ones((31, 31))

        # Action 1
        # Correct action: 1 
        # Possible State Changes: (0) 0 -> 1, (1) 0 -> 2 
        self.action_matrix[0][1] = 0 
        self.action_matrix[0][2] = 1 

        # Action 2
        # Correct action: 0
        # Possible State Changes: (0) 2 -> 5, (1) 2 -> 6
        self.action_matrix[2][5] = 0 
        self.action_matrix[2][6] = 1 

        # Action 3
        # Correct action: 1
        # Possible State Changes: (0) 5 -> 11, (1) 5 -> 12
        self.action_matrix[5][11] = 0 
        self.action_matrix[5][12] = 1 

        # Action 4
        # Correct action: 0 
        # Possible State Changes: (0) 12 -> 25, (1) 12 -> 26
        self.action_matrix[12][25] = 0
        self.action_matrix[12][26] = 1 

        # End of maze, desired end-state = 2121


    def do_action(self):
        options = self.action_matrix[self.curr_state]
        next_states = []

        for i in range(len(options)):
            action_num = int(options[i])
            if action_num != -1:
                next_states.append(i)

        if len(next_states) == 0:
            self.curr_state = 0
            self.next_state = 0
            return

        self.curr_action = np.random.choice(len(next_states))
        self.next_state = next_states[self.curr_action]


    def get_reward(self):
        # action was not performed, state was reset to 0    
        if self.curr_state == self.next_state:
            return
        
        reward = 0.0
        if self.next_state == self.end_state:
            reward = 100.0

        current_reward = self.q_matrix[self.curr_state][self.curr_action]
        future_reward = max(self.q_matrix[self.next_state])

        new_reward = current_reward + self.alpha * (reward + self.gamma * future_reward - current_reward)

        if current_reward == new_reward:
            self.convergence_count += 1
        else:
            self.convergence_count = 0
            self.q_matrix[self.curr_state][self.curr_action] = new_reward

        self.curr_state = self.next_state


    def save_q_matrix(self):
        # Save converged q matrix to csv file
        print('saving matrix...')
        with open(self.q_matrix_path, 'w+', newline='') as q_matrix_csv:
            writer = csv.writer(q_matrix_csv)
            for row in self.q_matrix:
                writer.writerow(row)
        print('matrix saved, press Ctrl+C to exit...')


    def run(self):
        while self.convergence_count < self.convergence_max:
            self.do_action()
            self.get_reward()

            if self.curr_state == self.end_state:
                self.curr_state = 0
                self.next_state = 0

        self.save_q_matrix()


if __name__ == "__main__":
    node = QLearning()
    node.run()

