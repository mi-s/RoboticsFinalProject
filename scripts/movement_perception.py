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
SIMPLE_TURN = "simple_turn"
ACTION_TURN = "action_turn"
CHECK_STATE = "check_state"
CHECK_DISTANCE = "check_distance"

class MovementPerception(object):
    def __init__(self):
        rospy.init_node("movement_perception")

        print("\nINITIALIZING\n")
        self.robot_state = INITIALIZING

        self.q_matrix_path = os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        self.q_matrix = np.loadtxt(self.q_matrix_path, delimiter=',')
    
        self.action_seq = [0, 0, 0]
        self.action_index = 0
        self.get_actions()

        self.distances_old = [0.0, 0.0, 0.0]
        self.distances = [0.0, 0.0, 0.0]
        self.front = [358, 359, 0, 1, 2]
        self.right = [88, 89, 90, 91, 92]
        self.left = [268, 269, 270, 271, 272]
        self.distances_loaded = False
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_distances)

        print("\nINITIALIZED\n")
        self.robot_state = INITIALIZED


    def get_actions(self):
        return


    def scan_distances(self, data):
        if self.robot_state != CHECK_DISTANCE
            return

        self.distances_old = self.distances
        self.drm = data.range_max

        self.distances[0] = self.drm
        for i in self.front:
            if data.ranges[i] < self.distances[0]:
                self.distances[0] = data.ranges[i]

        self.distances[1] = self.drm
        for i in self.right:
            if data.ranges[i] < d:
                self.distances[1] = data.ranges[i]

        self.distances[2] = self.drm
        for i in self.left:
            if data.ranges[i] < d:
                self.distances[2] = data.ranges[i]

        self.robot_state = CHECK_STATE


    def check_robot_state(self):
        d0 = distances[0]
        d1 = distances[1]
        d2 = distances[2]
        d0_ref = d0 * 3
        d1_ref = d1 * 3
        d2_ref = d2 * 3

        # if two openings: 
        #   self.robot_state = ACTION_TURN
        if d1 > d0_ref and d1 > d2_ref:
            self.robot_state = SIMPLE_TURN_RIGHT
        elif d2 > d0_ref and d2 > d1_ref:
            self.robot_state = SIMPLE_TURN_LEFT
        else:
            self.robot_state = MOVE_FORWARD


    def set_velocity(self, linear_vel, angular_vel):
        # Helper to set robot velocity
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_pub.publish(msg)


    def move_forward(self):
        lin_vel = 0.2
        ang_vel = 0.0
        
        if distances[1] < distances_old[1]:
            # angled to right, turn slightly left
            ang_vel = -0.2 * (distances_old[1] - distances[1])
        elif distances[2] < distances_old[2]:
            # angled to left, turn slightly right
            ang_vel = 0.2 * (distances_old[2] - distances[2])

        self.set_velocity(lin_vel, ang_vel)


    def do_turn(self, direction):
        # Turn robot in-place in given direction, then move forward one tile
        if direction == "left":
            print("left turn")
            rospy.sleep(100)
        elif direction == "right":
            print("right turn")
            rospy.sleep(100)
        else:
            print("error: pass 'right' or 'left' direction to turn()")


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
