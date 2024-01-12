#!/usr/bin/env python
import rospy
import time

from math import sqrt
from geometry_msgs.msg import Point
from assignment_2_2023.msg import Position

class PosNode:
    def __init__(self):
        # Initialize time
        self.prev_time = 0
        # Initialize default goal variables
        self.des_x = rospy.get_param('des_pos_x')
        self.des_y = rospy.get_param('des_pos_y')
        # Initialize /goal subscriber
        rospy.Subscriber('/goal', Point, self.goal_callback)
        # Subscribe to the msg
        rospy.Subscriber('/position', Position, self.position_callback)

    def goal_callback(self, msg):
        self.des_x = msg.x
        self.des_y = msg.y

    def position_callback(self, data):
        # Get the current time in miliseconds
        current_time = time.time() * 1000
        # If the time difference is gretaer than the period, print the info
        if current_time - self.prev_time > 1000:
            goal_str = '(' + str(self.des_x) + ' ' + str(self.des_y) + ')'
            distance = sqrt((self.des_x - data.x)**2 + (self.des_y - data.y)**2)
            velocity = sqrt(data.vel_x**2 + data.vel_y**2)
            rospy.loginfo('Distance from the goal: %f', distance)
            rospy.loginfo('Robot velocity: %f', velocity)
            self.prev_time = current_time

def main():
	# Initialize the node
	rospy.init_node('position_info')
	# Call PosNode class
	PosNode()
	# Keep the communication with the ROS network open
	rospy.spin()

if __name__ == '__main__':
	main()