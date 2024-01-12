#!/usr/bin/env python
import rospy
import assignment_2_2023.msg

from geometry_msgs.msg import Point
from assignment_2_2023.srv import Goals, GoalsResponse

class GoalService:
	def __init__(self):
        # Initialize default goal variables
		self.des_x = rospy.get_param('des_pos_x')
		self.des_y = rospy.get_param('des_pos_y')
		# Initialize the service
		rospy.Service('goals_srv', Goals, self.send_response)
		# Initialize a subscriber
		rospy.Subscriber('/goal', Point, self.goal_callback)
		
	def goal_callback(self, msg):
		# Update the goal based on the message
		self.des_x = msg.x
		self.des_y = msg.y
		
	def send_response(self, req):
		# Return the response
		return GoalsResponse(self.des_x, self.des_y)

def main():
	# Initialize the node
	rospy.init_node('goal_service')
	# Call GoalService class
	GoalService()
	# Keep the communication open
	rospy.spin()

if __name__ == '__main__':
	main()