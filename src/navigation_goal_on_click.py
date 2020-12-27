#!/usr/bin/env python


import rospy
import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point



class Navigation():
	def __init__(self):
		
		rospy.init_node('goal_on_click', anonymous=False)
		self.goal_indicate = Twist()
		self.previous_goal_indicate = Twist()
		self.goal_indicate.linear.x = 3
		self.goal_indicate.linear.y = -4.0
		self.finish_task = False
		self.previous_goal_indicate.linear.x = 1
		self.previous_goal_indicate.linear.y = 1
		self.goalDiff = False
		self.task_ended = False
		self.sub = rospy.Subscriber('instructions', Twist, self.callback)
		
	#this method will make the robot move to the goal location
	def callback(self, msg):
		self.goal_indicate.linear.x = msg.linear.x
		self.goal_indicate.linear.y = msg.linear.y  
		#def move_to_goal(self):
		self.diff()
		#rospy.loginfo("parameters are %s and %s", xGoal, yGoal)
		if self.goalDiff == True:
			rospy.loginfo("A new goal have been chosen")
			#define a client for to send goal requests to the move_base server through a SimpleActionClient
			ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

			 #wait for the action server to come up
			while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
					rospy.loginfo("Waiting for the move_base action server to come up")

			goal = MoveBaseGoal()
			 
			 
			#set up the frame parameters
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time.now()

			# moving towards the goal*/
			goal.target_pose.pose.position =  Point(self.goal_indicate.linear.x, self.goal_indicate.linear.y, 0)
			goal.target_pose.pose.orientation.x = 0.0
			goal.target_pose.pose.orientation.y = 0.0
			goal.target_pose.pose.orientation.z = 0.0
			goal.target_pose.pose.orientation.w = 1.0

			rospy.loginfo("Sending goal location ...")
			ac.send_goal(goal)
			#ac.wait_for_result(rospy.Duration(1))
			if(ac.get_state() ==  GoalStatus.SUCCEEDED):
				rospy.loginfo("You have reached the destination")
				return True

			else:
				rospy.loginfo("The robot is not arrived yet")
				return False
		else:
			rospy.loginfo("The robot is not arrived yet")
		
	


	def diff(self):
		if (self.goal_indicate.linear.x == self.previous_goal_indicate.linear.x 
			and self.goal_indicate.linear.y == self.previous_goal_indicate.linear.y):

			self.goalDiff = False
		else:
			self.goalDiff = True
		self.previous_goal_indicate.linear.x = self.goal_indicate.linear.x
		self.previous_goal_indicate.linear.y = self.goal_indicate.linear.y

	def shutdown(self):
				# stop turtlebot
				rospy.loginfo("Stop")
				self.cmd_vel.publish(Twist())
				rospy.sleep(1)
	

def main(args):
	Navigation()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
	

