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
        self.action_matrix[2][6] = 0 
        self.action_matrix[2][7] = 1 

        # Action 3
        # Correct action: 0 
        # Possible State Changes: (0) 6 -> 11, (1) 6 -> 12
        self.action_matrix[6][12] = 0 
        self.action_matrix[6][13] = 1 

        # Action 4
        # Correct action: 1 
        # Possible State Changes: (0) 12 -> 24, (1) 12 -> 25
        self.action_matrix[12][24] = 0
        self.action_matrix[12][25] = 1 

        # End of maze, desired end-state = 1001 = state 25


    def do_action(self):
        """
        Take a random action out of all the possible actions at a given state, and publish
        that action and at what state it was taken at in order for reward to get calculated.
        """
        # Get options or "action space" - possible actions that robot can take at its current state.
        # Get list of next states possible from our action space.
        options = self.action_matrix[self.curr_state]
        next_states = []
        for i in range(len(options)):
            action_num = int(options[i])
            if action_num != -1:
                next_states.append(i)

        # If we have run out of actions possible to take at current state, reset state to 0 and return.
        if len(next_states) == 0:
            self.curr_state = 0
            self.next_state = 0
            return

        # Select a random action to take within our action space. Specify the next state to occur
        # as a result of taking this action.
        self.curr_action = np.random.choice(len(next_states))
        self.next_state = next_states[self.curr_action]


    def get_reward(self):
        """
        Calculate reward according to q learning algorithm. Track q matrix convergence if the new
        reward value didn't change from the last update.
        """
        # action was not performed, state was reset to 0    
        if self.curr_state == self.next_state:
            return
        
        # Set a reward of 100 for reaching the finish state of the maze, 0 otherwise.
        reward = 0.0
        if self.next_state == self.end_state:
            reward = 100.0

        # Calculate reward currently written in q matrix for our current state/action pair.
        current_reward = self.q_matrix[self.curr_state][self.curr_action]
        # Calculate the max reward for the next state
        future_reward = max(self.q_matrix[self.next_state])

        # Use the q learning algorithm to get a new reward
        new_reward = current_reward + self.alpha * (reward + self.gamma * future_reward - current_reward)

        # If the new reward value didn't change from the last update, we count one instance of
        # convergence and save this to the convergence_count. Otherwise, we write the new reward value
        # in the q matrix.
        if current_reward == new_reward:
            self.convergence_count += 1
        else:
            self.convergence_count = 0
            self.q_matrix[self.curr_state][self.curr_action] = new_reward

        # Update current state
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
        """ Runs Q Learning code. """
        # Until we reach our threshold level of convergence (5000 instances), continue running
        # taking actions and calculating rewards in a loop. Note that when the algorithm chooses
        # the "incorrect" option and there are no longer any possible actions, the state gets reset
        # to the beginning of the maze.
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

