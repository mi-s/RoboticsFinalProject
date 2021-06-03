# Final Project

Team members: Michael Su, Arjun Voruganti

![gif](https://github.com/mi-s/final_project/blob/main/maze_run.gif)

## Description

The objective of our project was to expand on what we learned about Q-Learning in the previous project and apply it to a robot maze navigation problem. Our idea was to predefine a maze on a grid, build our own reinforcement learning environment including definitions for state/action spaces, and apply a Q-Learning training algorithm accordingly. From there, our goals were firstly to develop robust robot perception/movement to handle all of the complexities of navigating a maze, and secondly to have the robot execute actions (turns) that lead to the maze exit according to a converged Q-matrix that identifies the "optimal strategy". We divided the overall objective into three steps: 1) design maze, 2) implement Q-Learning algorithm, and 3) implement robot perception/movement.

### 1. Design maze

We wanted to define a maze that was relatively small yet complex enough to exhaust all possible actions at forks (i.e. turn left, turn right, continue forwards). Additinally, we wanted to have a maze with two consecutive forks--this requires our maze navigation to be robust and carefully handle fork cases one by one. (A fork is defined by a point on the maze where there are two different paths the robot can take, not including going backwards.) We also came up with a final constraint that the maze would have no loops or cycles in it. See a diagram of our maze below:

![png](https://github.com/mi-s/final_project/blob/main/maze%20layout.png)

In this maze, there are four forks, and the last three come consecutively after each other. Thus, we were convinced that this particular maze layout was reasonably complex to test our code on. The robot is to turn left at the first fork, go straight at the second, turn left at the third, and turn right at the fourth.

### 2. Q-Learning algorithm

An important element to this project was that we took on the task of defining the environment to run Q-Learning training in; this was more or less given to us in the previous project. Accordingly, we had to make some important design choices with respect to our definitions of the state/action spaces. 

### TODO: add more. Explain more about the 0/1 action space definitions. And also that not all states are actually reachable, only 0 2 6 12 and 25.

### 3. Robot perception/movement

After training, we receive a converged Q-matrix that informs the robot of the correct actions to take at every fork in order to navigate to the maze exit. In order to translate the Q-matrix instructions into robot navigation code, there were a few steps we had to take. Firstly, the robot had to correctly identify whether it was traversing a straight passage, entering a turn, or entering a fork state. Then, the robot had to handle each of these scenarios accordingly. For the straight passage, this meant positioning itself in the middle of each passage so that it did not get too close to either of the side walls. For turns, this means recognizing which direction to turn and also turning 90 degrees with precision. For forks, this meant identifying that there are two possible paths ahead of the robot, and then either turning or continuing straight according to the Q-Matrix instructions. 

## System architecture

### TODO: add more. General outline:

* q_learning.py:
- manually built action matrix in q_learning.py
- implemented Q-learning algorithm and training steps in q_learning.py
- algorithm only receives reward at maze exit
- If action is taken that would lead to a dead end, reset algorithm to initial state
- Q-matrix is considered converged when 5000 iterations pass and no values in the matrix get changed.

* movement_perception.py:
- 



## Challenges

There were two main challenges that we encountered during this project. The first one pertains to differentiating between straights, turns, and forks in the maze. By our maze definitions, a straight occurs when there are two walls on either side of the robot, a turn occurs where there is only one wall on either side of the robot, and a fork occurs when there is only one wall in any of the four cardinal directions relative to the robot. It was a bit of a challenge to determine using the LiDAR sensor which of these three conditions our robot was facing. In particular, if our robot was too close to one wall of a straight, its LiDAR sensor would detect an opening and not another wall on the other side of the robot. Our second and most difficult challenge was getting the robot to turn with precision. This was particularly difficult because we faced lots of random noise and drift after the robot reached the end of a straight and was beginning its turn. Our turn instruction is for the robot to simply turn 90 degrees. However, due to noise and drift, the robot often began its turn ~10˚ off the perpendicular. This slight inaccuracy was enough to cause our robot to keep hitting walls after execution of the turn.

drift and noise on turns

(1 paragraph)

## Future work

(1 paragraph)

## Takeaways

(1 paragraph)

