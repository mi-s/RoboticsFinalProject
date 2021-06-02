#!/usr/bin/env python3

import rospy, rospkg
import os
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

INITIALIZING = "initializing"
INITIALIZED = "initialized"
MOVE_FORWARD = "move_forward"
SIMPLE_TURN_LEFT = "simple_turn_left"
SIMPLE_TURN_RIGHT = "simple_turn_right"
ACTION_TURN = "action_turn"
CHECK_STATE = "check_state"
CHECK_DISTANCE = "check_distance"

class MovementPerception(object):
    def __init__(self):
        rospy.init_node("movement_perception")

        self.robot_state = INITIALIZING

        self.q_matrix_path = os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        self.q_matrix = np.loadtxt(self.q_matrix_path, delimiter=',')
    
        self.action_seq = []
        self.action_index = 0
        self.get_actions()

        self.distances_old = [0.0, 0.0, 0.0]
        self.distances = [0.0, 0.0, 0.0]
        self.front = [358, 359, 0, 1, 2]
        self.left = [88, 89, 90, 91, 92]
        self.right = [268, 269, 270, 271, 272]
        self.distances_loaded = False
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_distances)

        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.robot_state = INITIALIZED


    def get_actions(self):
        for i in range(len(self.q_matrix)):
            for j in range(len(self.q_matrix[i])):
                if self.q_matrix[i][j] > 0.0:
                    self.action_seq.append(j)


    def update_old_distances(self):
        for i in range(len(self.distances)):
            self.distances_old[i] = self.distances[i]



    def scan_distances(self, data):
        if self.robot_state != CHECK_DISTANCE:
            return

        self.update_old_distances()

        self.drm = data.range_max

        self.distances[0] = self.drm
        for i in self.front:
            if data.ranges[i] < self.distances[0]:
                self.distances[0] = data.ranges[i]

        self.distances[1] = self.drm
        for i in self.right:
            if data.ranges[i] < self.distances[1]:
                self.distances[1] = data.ranges[i]

        self.distances[2] = self.drm
        for i in self.left:
            if data.ranges[i] < self.distances[2]:
                self.distances[2] = data.ranges[i]

        self.robot_state = CHECK_STATE


    def check_robot_state(self):
        d0 = self.distances[0]
        d1 = self.distances[1]
        d2 = self.distances[2]
        d0_ref = d0 * 2.5 
        d1_ref = d1 * 2.5 
        d2_ref = d2 * 2.5

        # if two openings: 
        #   self.robot_state = ACTION_TURN
        if d1 > d0_ref and d1 > d2_ref:
            self.robot_state = SIMPLE_TURN_RIGHT
        elif d2 > d0_ref and d2 > d1_ref:
            self.robot_state = SIMPLE_TURN_LEFT
        else:
            self.robot_state = MOVE_FORWARD


    def set_velocity(self):
        # Helper to set robot velocity
        msg = Twist()
        msg.linear.x = self.lin_vel
        msg.angular.z = self.ang_vel
        self.cmd_pub.publish(msg)


    def move_forward(self):
        self.lin_vel = 0.15

        if self.distances[1] < self.distances_old[1]:
            # angled to left, turn slightly right 
            self.ang_vel = 2 * (self.distances_old[1] - self.distances[1])
        elif self.distances[2] < self.distances_old[2]:
            # angled to right, turn slightly left 
            self.ang_vel = -4 * (self.distances_old[2] - self.distances[2])
        self.set_velocity()
        rospy.sleep(.25)


    def do_turn(self, direction):
        # Turn robot in-place in given direction, then move forward one tile
        if self.ang_vel != 0:
            self.lin_vel = .15
            self.ang_vel = 0
            rospy.sleep(.5)
            self.lin_vel = 0
            self.set_velocity()
            rospy.sleep(1)

        self.lin_vel = 0
        self.ang_vel = math.pi / 8 

        if direction == "right":
            self.ang_vel *= -1

        self.set_velocity()
        rospy.sleep(3.9)
        self.ang_vel = 0
        self.set_velocity()
        rospy.sleep(1)
        self.lin_vel = .15
        self.set_velocity()
        rospy.sleep(6)
        self.lin_vel = 0
        self.set_velocity()
        rospy.sleep(1)

        self.robot_state = CHECK_DISTANCE
        while self.robot_state == CHECK_DISTANCE:
            pass


    def do_action(self):
        return


    def run(self):
        while self.robot_state == INITIALIZING:
            pass

        while self.action_index <= len(self.action_seq):                      
            self.robot_state = CHECK_DISTANCE
            while self.robot_state == CHECK_DISTANCE:
                pass

            self.check_robot_state()
            while self.robot_state == CHECK_STATE:
                pass

            if self.robot_state == MOVE_FORWARD:
                self.move_forward()
            elif self.robot_state == SIMPLE_TURN_LEFT:
                self.do_turn("left")
            elif self.robot_state == SIMPLE_TURN_RIGHT:
                self.do_turn("right")
            elif self.robot_state == ACTION_TURN:
                self.do_action()
                self.action_index += 1
            else:
                print("error: unrecognized state")
                return

        while distances[0] > .1:
            self.move_forward()
          
if __name__ == "__main__":
    node = MovementPerception()
    node.run()
