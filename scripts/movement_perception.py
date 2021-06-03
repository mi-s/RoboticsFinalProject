#!/usr/bin/env python3

import rospy, rospkg
import os
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# All possible robot states.
# Used to properly time execution of functions during maze navigation.
INITIALIZING = "initializing"
INITIALIZED = "initialized"
MOVE_FORWARD = "move_forward"
SIMPLE_TURN_LEFT = "simple_turn_left"
SIMPLE_TURN_RIGHT = "simple_turn_right"
ACTION_TURN_01 = "action_turn_01"
ACTION_TURN_02 = "action_turn_02"
ACTION_TURN_12 = "action_turn_12"
CHECK_STATE = "check_state"
CHECK_DISTANCE = "check_distance"


class MovementPerception(object):
    def __init__(self):
        # Initialize node
        rospy.init_node("movement_perception")

        # Set robot state to prevent code from running too early
        self.robot_state = INITIALIZING

        # Load converged Q-Matrix from .csv file
        self.q_matrix_path = os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        self.q_matrix = np.loadtxt(self.q_matrix_path, delimiter=',')
    
        # Read Q-Matrix to load optimal action sequence into an array 
        self.action_seq = []
        self.action_index = 0
        self.get_actions()

        # Initialize arrays for distance measurements 
        self.distances_old = [0.0, 0.0, 0.0]
        self.distances = [0.0, 0.0, 0.0]
        self.front = [358, 359, 0, 1, 2]
        self.left = [88, 89, 90, 91, 92]
        self.right = [268, 269, 270, 271, 272]
        
        # Initialize velocity publisher and distance subscriber 
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_distances)

        # Initialize variables to hold linear and angular velocities
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        # Set robot state to maze navigation 
        self.robot_state = INITIALIZED


    def get_actions(self):
        # Read Q-Matrix to load optimal action sequence into an array 
        for i in range(len(self.q_matrix)):
            for j in range(len(self.q_matrix[i])):
                if self.q_matrix[i][j] > 0.0:
                    self.action_seq.append(j)


    def update_old_distances(self):
        # Helper that updates distances_old
        for i in range(len(self.distances)):
            self.distances_old[i] = self.distances[i]


    def update_distances(self):
        # Helper that updates distances
            self.robot_state = CHECK_DISTANCE
            while self.robot_state == CHECK_DISTANCE:
                pass


    def scan_distances(self, data):
        # Callback function that updates robot's distance from walls
        # in a 5 degree arc in the front, left, and right directions
        if self.robot_state != CHECK_DISTANCE:
            # Prevent function from using resources unless distances needed
            return

        # Save distances before updating to assist in proportional control
        self.update_old_distances()

        # Update frontal distance
        self.distances[0] = data.range_max 
        for i in self.front:
            if data.ranges[i] < self.distances[0]:
                self.distances[0] = data.ranges[i]

        # Update distance to the left 
        self.distances[1] = data.range_max
        for i in self.left:
            if data.ranges[i] < self.distances[1]:
                self.distances[1] = data.ranges[i]

        # Update distance to the right 
        self.distances[2] = data.range_max 
        for i in self.right:
            if data.ranges[i] < self.distances[2]:
                self.distances[2] = data.ranges[i]
        
        self.robot_state = CHECK_STATE


    def check_robot_state(self):
        # Checks distances of the robot on front, left, and right sides
        # to determine if the robot is in an action, turn, or straight passage
        d0 = self.distances[0]
        d1 = self.distances[1]
        d2 = self.distances[2]
        front_min = 0.6
        side_min = 0.8
        max_dist = 1.1

        if d0 < front_min and d1 > max_dist and d2 > max_dist: 
            # Left/Right Action
            self.robot_state = ACTION_TURN_12
        elif d0 > max_dist and d1 > max_dist and d2 < side_min:
            # Front/Left Action
            self.robot_state = ACTION_TURN_01
        elif d0 > max_dist and d1 < side_min and d2 > max_dist:
            # Front/Right Action
            self.robot_state = ACTION_TURN_02
        elif d0 < front_min and d1 > max_dist and d2 < side_min:
            # Turn Left
            self.robot_state = SIMPLE_TURN_LEFT
        elif d0 < front_min and d1 < side_min and d2 > max_dist: 
            # Turn Right
            self.robot_state = SIMPLE_TURN_RIGHT
        else:
            # Move Forward
            self.robot_state = MOVE_FORWARD


    def set_velocity(self, lv, av, sleep):
        # Helper to set robot velocity and sleep afterwards
        msg = Twist()
        self.lin_vel = lv
        self.ang_vel = av
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.cmd_pub.publish(msg)
        rospy.sleep(sleep)


    def smooth_stop(self):
        # Helper to stop the robot smoothly as it enters turns
        if self.lin_vel > 0.0:
            smooth_lv1 = 0.5 * self.lin_vel
            smooth_lv2 = 0.25 * self.lin_vel
            smooth_av1 = -0.5 * self.ang_vel 
            smooth_av2 = -0.25 * self.ang_vel
            sleep = 2.0

            if self.robot_state == ACTION_TURN_01 or self.robot_state == ACTION_TURN_02: 
                sleep = 4.0

            self.set_velocity(smooth_lv1, smooth_av1, sleep)
            self.set_velocity(smooth_lv2, smooth_av1, sleep)
            sleep = 1.0
            self.set_velocity(0.0, 0.0, sleep)


    def move_forward(self):
        # Drives the robot forward in a maze passage while
        # using proportional control to keep the robot aligned
        lv = 0.1
        av = 0.0
        sleep = 0.25 # Low sleep time to continuously check for action/turn
        min_distance_to_side_wall = 0.35

        d1 = self.distances[1]
        d2 = self.distances[2] 
        d1_old = self.distances_old[1]
        d2_old = self.distances_old[2]

        if d1 < min_distance_to_side_wall:
            # too close to left wall, turn right
            av = -1 * (min_distance_to_side_wall - d1) 
        elif d2 < min_distance_to_side_wall:
            # too close to right wall, turn left 
            av = 2 * (min_distance_to_side_wall - d2)
        elif d1 < d1_old:
            # angled to left, turn slightly right 
            av = -4 * (d1_old - d1)
        elif d2 < d2_old:
            # angled to right, turn slightly left 
            av = 8 * (d2_old - d2)

        self.set_velocity(lv, av, sleep)


    def do_turn(self, direction):
        # Turn robot in-place in given direction, then move forward

        # Smoothly stop robot to minimize noise and prepare for turn in-place
        self.smooth_stop()

        # Perform 90 degree turn in-place in the given direction
        lv = 0.0
        av = math.pi / 8.0
        sleep = 3.5
        if direction == "right":
            av *= -1
        self.set_velocity(lv, av, sleep)

        av *= .5
        sleep = 1.3
        self.set_velocity(lv, av, sleep) 
        
        # Stop robot rotation
        av = 0.0
        sleep = 1.0
        self.set_velocity(lv, av, sleep)

        # Move robot forward to exit turn
        lv = 0.1
        sleep = 8.5 
        self.set_velocity(lv, av, sleep)
        self.smooth_stop()

        # Update distances to prevent error in move_forward
        self.update_distances()


    def do_action(self):
        # Peforms the action indicated by the robot state
        
        lv = 0.1        
        av = 0.0
        sleep = 8.5 
        
        # Action = 0 or 1.  Action 0 means to take the first possible path
        # in a counter-clockwise direction starting at 0 degrees.
        # Action 1 means to take the second possible path. 
        action = self.action_seq[self.action_index]

        if self.robot_state == ACTION_TURN_01:
            # Action 0 = Drive Straight, Action 1 = Turn Left
            if action == 0:
                self.set_velocity(lv, av, sleep)
                self.smooth_stop()
            else:
                self.do_turn("left")
        elif self.robot_state == ACTION_TURN_02:
            # Action 0 = Drive Straight, Action 1 = Turn Right
            if action == 0:
                self.set_velocity(lv, av, sleep)
                self.smooth_stop()
            else:
                self.do_turn("right")
        elif self.robot_state == ACTION_TURN_12:
            # Action 0 = Turn Left, Action 2 = Turn Right
            if action == 0:
                self.do_turn("left")
            else:
                self.do_turn("right")
        else:
            print("error: unrecognized action")


    def run(self):
        # Executes maze navigation loop.  Robot will repeatedly scan 
        # surroundings to maneuver through the maze and identify when
        # it has to perform actions.  Once all actions are completed,
        # the robot will escape the maze.
        while self.robot_state == INITIALIZING:
            # Prevent execution from beginning until after initialization
            pass

        while self.action_index < len(self.action_seq):                      
            # Navigate maze until all actions completed

            # Update robot's distances to surrounding walls
            self.update_distances()

            # Use distances to update robot's state
            self.check_robot_state()
            while self.robot_state == CHECK_STATE:
                pass

            # Perform robot maneuver based on state
            if self.robot_state == MOVE_FORWARD:
                self.move_forward()
            elif self.robot_state == SIMPLE_TURN_LEFT:
                self.do_turn("left")
            elif self.robot_state == SIMPLE_TURN_RIGHT:
                self.do_turn("right")
            elif self.robot_state in [ACTION_TURN_01, ACTION_TURN_02, ACTION_TURN_12]:
                self.do_action()
                self.action_index += 1 # Increment so robot knows current action
            else:
                print("error: unrecognized state")
                return

        # Maze complete, turtlebot3 is now free
        self.set_velocity(5.0, 0.0, 100.0)
          

if __name__ == "__main__":
    node = MovementPerception()
    node.run()
