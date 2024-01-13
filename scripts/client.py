#!/usr/bin/env python
import rospy
import sys
import select
import actionlib
import actionlib.msg
import assignment_2_2023.msg

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import Position

class Client:

    def __init__(self):
        # Initialize variable 'pos' to be published as robot position
        self.position = None
        # Initialize variable 'l_vel' to be published as robot linear velocity
        self.linear_vel = None
        # Initialize a publisher /position for position
        self.position_pub = rospy.Publisher('/position', assignment_2_2023.msg.Position, queue_size = 1)
        # Initialize a subscriber for Odometry
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        # Initialize an action client
        self.ac = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
        # Wait for the action server ready
        self.ac.wait_for_server()
        # Create a goal message
        self.goal = assignment_2_2023.msg.PlanningGoal()
		# Create a publisher to send the target
        self.goal_pub = rospy.Publisher('/goal', Point, queue_size = 1)

    # Method to store position and linear velocity from Odometry and publish them to the custom message
    def odometry_callback(self, data):
        self.position = data.pose.pose.position
        self.linear_vel = data.twist.twist.linear

        msg = assignment_2_2023.msg.Position()
        msg.x = self.position.x
        msg.y = self.position.y
        msg.vel_x = self.linear_vel.x
        msg.vel_y = self.linear_vel.y
		
		# Publish the message
        self.position_pub.publish(msg)

    # Method to request an input to the user
    def goal_input(self):
        print("Enter the goal coordinates (x,y) or enter 'c' to cancel the goal, then press ENTER:")
		
        while not rospy.is_shutdown():
            # Request the user to enter the goal
            input_var = input()
			
            # Cancel the goal if the user enters 'c'
            if input_var == 'c':
                self.ac.cancel_goal()
                print("Goal cancelled")
					
            else:
                # Check whether the input values are float and send it to the action server
                try:
                    x, y = [float(value) for value in input_var.split(',')]
                    self.goal.target_pose.pose.position.x = x
                    self.goal.target_pose.pose.position.y = y  
                
                    goal_msg = Point()
                    goal_msg.x = self.goal.target_pose.pose.position.x
                    goal_msg.y = self.goal.target_pose.pose.position.y
                    self.goal_pub.publish(goal_msg)                                    
                                
                    self.ac.send_goal(self.goal)
                # Return an error message when the value is invalid
                except ValueError:
                    print("Invalid input. Please enter the goal coordinates in the format 'x,y'")

def main():
	# Initialize /client node
	rospy.init_node('client')
	# Create an object for the class Client
	client = Client()
	# Call goal_input() method to request
	client.goal_input()
	# Keep the communication with the ROS network open
	rospy.spin()

if __name__ == '__main__':
	main()
