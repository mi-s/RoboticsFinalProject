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

In the maze environment, we felt that actions most naturally corresponded to choosing a path at a fork.   At the fork, there are two potential paths.  From these two paths, we defined Action 0 and Action 1. Action 0 refers to taking the path closest to 0 degrees CCW with respect to the robot's orientation as it approaches the fork.  Action 1 refers to the second possible path.  Building on this, we defined states as the sequence of actions the robot has taken to reach the section of the maze it is in.  For example, the robot starts in state " ", and if it decides to take action 0 and then action 1 it will be in state "01".

In our Q-Learning action matrix, we have 31 total states.  If states are defined by our definition above and there are four states, then there are 1 + 2 + 2^2 + 2^3 + 2^4 = 31 total states.  However, most of these states are unreachable.  In our maze, taking the wrong action at any action will immediately lead to a dead end.  Thus, if the robot finds itself in state "0" but state "0" is a dead end, then states "01", "00", "010", and so on also do not exist.  In our final converged Q-Matrix, only states 0 2 6 12 and 25 are entered because they are the only states passed through to complete the maze.


### 3. Robot perception/movement

After training, we receive a converged Q-matrix that informs the robot of the correct actions to take at every fork in order to navigate to the maze exit. In order to translate the Q-matrix instructions into robot navigation code, there were a few steps we had to take. Firstly, the robot had to correctly identify whether it was traversing a straight passage, entering a turn, or entering a fork state. Then, the robot had to handle each of these scenarios accordingly. For the straight passage, this meant positioning itself in the middle of each passage so that it did not get too close to either of the side walls. For turns, this means recognizing which direction to turn and also turning 90 degrees with precision. For forks, this meant identifying that there are two possible paths ahead of the robot, and then either turning or continuing straight according to the Q-Matrix instructions. 

## System architecture

### General outline:

#### q_learning.py
This file handles the Q-Learning part of our project.  When run, it will create a converged Q-Matrix for our maze.  In it, we manually built a 31x2 action matrix by adding all possible state transitions.  Then, we implemented a Q-Learning algorithm that gives a reward when the end-state is reached.  If it ever randomly takes an action that leads to a dead-end, then it simply restarts the algorithm.  Once the algorithm has converged after 5000 iterations without any change, the algorithm completes.

#### movement_perception.py:
This file handles the robot's movement and perception as it navigates the maze.  To differentiate between straights, turns, and forks, the robot continuously scans in the 0, 90, and 270 degree directions while moving through straights.  When the robot detects that moving directly forward is no longer possible or is not the only possible path, it will  decide whether it is in a turn or action and then slow to a stop in the intersection.  Should turning be the best possible path, the robot will rotate 90 degrees in-place in the correct direction before driving straight.  If it is in action where it is best to move straight, it will simply continue going straight. 

Although our turns are handled in a way that could potentially lead to error due to lack of proportional control, in our project we handle 3 consecutive actions without issue.  This is due to the proportional control used to align our robot in straights and when transitioning to and between turns.

Actions are read in from a csv file into an array.  The navigation function will continuously run until all actions loaded into the array have been completed.  At that point, the robot has exited the maze and will celebrate.

## Challenges

There were two main challenges that we encountered during this project. The first one pertains to differentiating between straights, turns, and forks in the maze. By our maze definitions, a straight occurs when there are two walls on either side of the robot, a turn occurs where there is only one wall on either side of the robot, and a fork occurs when there is only one wall in any of the four cardinal directions relative to the robot. It was a bit of a challenge to determine using the LiDAR sensor which of these three conditions our robot was facing. In particular, if our robot was too close to one wall of a straight, its LiDAR sensor would detect an opening and not another wall on the other side of the robot. Our second and most difficult challenge was getting the robot to turn with precision. This was particularly difficult because we faced lots of random noise and drift after the robot reached the end of a straight and was beginning its turn. Our turn instruction is for the robot to simply turn 90 degrees. However, due to noise and drift, the robot often began its turn ~5-10˚ off the perpendicular. This slight inaccuracy was enough to cause our robot to keep hitting walls after execution of the turn. Our solution was to "smooth" the transition between moving straight and executing a turn using proportional control methodologies.

## Future work

Given 2-3 more weeks to continue work on the project, there are two major thing we would like to improve on. Firstly, we would try to implement turning with proportional control. Currently, our robot executes turns by simply rotating 90˚, and in order for this method to work, we have to rely on the robot being perfectly perpendicular to the straight before the turn, so that it can be perfectly in-line with the straight after the turn. However, this is certainly assumption to make when programming with robots where noise and drift is often encountered. Accordingly, we came up with the following high-level implementation of turning with proportional control:

![png](https://github.com/mi-s/final_project/blob/main/proportional_control_turn_idea.png)

First, let's make the observation that at a turn (or a fork where the robot is instructed to turn), there are 2 (or 1) walls within the four cardinal directions relative to the robot. Our idea is to take one of these walls detected by the robot's side sensor, "track" its distance from the robot, and then wiggle the robot using proportional control until this tracked wall appears in the robot's backwards sensor. For example: if the robot is to make a right turn, track the distance between the robot and the left wall, and then keep turning until the back sensor picks up the same distance measurement recorded. Implementing turning with proportional control would certainly make our robot's maze navigation more robust and less prone to error.

Secondly, we would like to improve on the robot's general speed and smoothness of navigation. Currently, all of our robot perception is highly dependent on an accurate orientation of our robot, but we would like to have our code less dependent on the robot being perfectly straight or in-line at all points of the maze. In order to do so, we would have to come up with a way to handle state recognition independently of the robot's orientation.

## Takeaways

Firstly, we learned how to design a reinforcement learning environment for Q-Learning from scratch, which gave us a significantly greater understanding of Q-Learning and reinforcement learning algorithms in general. Secondly, we recognized how important implementing proportioal control was for our task. In our maze, even the smallest amounts of noise and drift affected our maze navigation implementation. Proportional control significantly helped us rectify these issues. Thirdly, we recognized that our implementation code would be most effective if it could be generalizable to any maze layout. This means that we did not want to write any code to specifically handle any particular cases in the maze we designed. This gives our code a certain robustness that might be expected when integrating robots into tasks in the "real world", for the real world as an environment is highly variable and robots should be able to withstand minor fluctuations accordingly.

