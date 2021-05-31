#!/usr/bin/env python3

import rospy, rospkg
import os
import numpy as np
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MovementPerception(object):
    def __init__(self):
        rospy.init_node("movement_perception")

        self.q_matrix_path = os.path.join(os.path.dirname(__file__), '../output/q_matrix.csv')
        self.q_matrix = np.loadtxt(self.q_matrix_path, delimiter=',')
    
        self.action_seq = [0, 0, 0]
        self.action_index = 0
        #self.get_actions()

        self.distances = [0.0, 0.0, 0.0]
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        #self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_distances)

        rospy.sleep(3)


    def scan_distances(self, data):
        front = [358, 359, 0, 1, 2]
        right = [88, 89, 90, 91, 92]
        left = [268, 269, 270, 271, 272]

        d = data.range_max

        for i in front:
            if data.ranges[i] < d:
                d = data.ranges[i]

        self.distances[0] = d
        d = data.range_max

        for i in right:
            if data.ranges[i] < d:
                d = data.ranges[i]

        self.distances[1] = d
        d = data.range_max

        for i in left:
            if data.ranges[i] < d:
                d = data.ranges[i]

        self.distances[2] = d

    # def get_actions(self):

    def check_action(self):
        return False

    # def do_action(self):
    
    def set_velocity(self, linear_vel, angular_vel):
        # Helper to set robot velocity
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        print("publishing..")
        self.cmd_pub.publish(msg)
        print("published")

    def move_forward(self):
        self.set_velocity(.19, 0) 
        print("\n1\n")
        rospy.sleep(5)
        print("\n2\n")
        self.set_velocity(0, 0)

    def turn(self, direction):
        # Turn robot in-place in given direction, then move forward one tile
        if direction == "left":
            vel = -1 * (math.pi / 8) 
        elif direction == "right":
            vel = math.pi / 8 
        else:
            print("error: pass 'right' or 'left' direction to turn()")

    #    self.set_velocity(0, vel)
   #     rospy.sleep(4.1)
        
   #     self.set_velocity(0, 0)
   #     rospy.sleep(1)

        self.move_forward()

    def no_action(self):
        # Handles maze navigation when no action is detected
        distance_index = 0
        distances_max = self.distances[0]

        # Find direction with highest distance (open path)
        for i in range(len(self.distances)):
            if self.distances[i] > distances_max:
                distance_index = i
                distances_max = self.distances[i]
        print("distance_index: ", distance_index)       
        # Perform movement 
        if distance_index == 0:
            self.move_forward()
       # elif distance_index == 1:
            #self.turn("right")
        else:
            print("Exit")
            rospy.sleep(100)
            #self.turn("left")


    def navigation_loop(self):
        i = 0

        while i < len(self.action_seq):
            if self.check_action():
                self.do_action()
                i += 1
                rospy.sleep(3)
            else:
                self.move_forward()
                rospy.sleep(3)


    def run(self):
        self.navigation_loop()
          
if __name__ == "__main__":
    node = MovementPerception()
    node.run()
