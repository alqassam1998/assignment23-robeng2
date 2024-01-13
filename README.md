# ROS Simulator

This repository contains code files that are necessary to run the ROS package for the second assignment of Research Track 1. 

## How to run the code

 1. Clone the repository `git clone https://github.com/alqassam1998/assignment23-robeng.git` inside the ROS workspace folder
 2. Rename the folder to `assignment_2_2023`
 3. Run `roscore`
 4. Run `catkin_make` inside the ROS workspace folder
 5. Run on the terminal `roslaunch assignment_2_2023 assignment1.launch`

## How the code works

Three additional nodes were developed to send the goal coordinate input from the user, track the distance between the robot and the goal and its velocity, and run a service that returns the goal coordinate.

The following pseudocode is the explanation of the `client.py` code:
 1. Initialize  `/client` node
 2. Call `Client()` class:
 - Initialize method `__init__`:
	 - Initialize robot position and linear velocity to None
	 - Initialize a publisher to send the position and linear velocity through `/position` topic
	 - Initialize a subscriber of `/odom` topic
	 - Initialize an action client by interacting with `/reaching_goal` topic
	 - Wait for an action server ready
	 - Initialize a goal message for `/reaching_goal` action
	 - Initialize a publisher for the goal coordinate through `/goal` topic
 - Run method `odometry_callback` to update the position and velocity of the robot
	 - Initialize the robot position and linear velocity variables
	 - Call the custom message `Position`
	 - Update the robot position and linear velocity
	 - Publish the message
 - Run method `goal_input`
	 - Request the user to enter the goal coordinate
	 - If the input is `'c'`, cancel the goal
	 - If the input is `float`, update the goal coordinate and publish the message to the action server
 3. Run the loop until the session ends

## Possible Improvements
Implement an algorithm to tell the coordinate is possible to reach and visualize the goal coordinate.
