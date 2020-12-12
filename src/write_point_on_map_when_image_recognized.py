#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from enum import Enum, IntEnum
from sensor_msgs.msg import LaserScan
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
from math import *
"""
roslaunch integrated_robotics_project turtlebot3_search_and_rescue.launch
rosrun find_object_2d find_object_2d image:=/camera/rgb/image_raw
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration
roslaunch explore_lite explore.launch
"""
class MarkerRobot:
	def __init__(self):
		rospy.init_node('path_node')
		self.count = 0
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		self.odom_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
		self.object_sub = rospy.Subscriber('/objects', Float32MultiArray, self.object_cb)
		self.markerPub = rospy.Publisher('/robotMarker', Marker, queue_size=1)
		self.current_time = rospy.get_rostime()
		self.passed_time = rospy.get_rostime()
		self.odom = Odometry()
		self.laserscan_data = []
		self.object_angle = 0
		self.object_found = Float32MultiArray()
		self.first_object_detected = False
		# params
		self.image_alive = 11
		self.image_toxic = 12
		self.image_warning = 13
		self.image_injured = 14
		self.image_fire = 15
		self.image_no_smoke = 16
		self.image_radioactive = 17
		self.image_triangle = 18
		self.image_dead = 19
		"""
		while not rospy.is_shutdown():
			self.main()
		"""
	def odom_cb(self, odom):
		self.odom = odom

	def laserscan_cb(self, msg):
		self.laserscan_data = msg.ranges
		#print (len(self.laserscan_data))

	def object_cb(self, object_found):
		self.current_time = rospy.get_rostime()
		self.object_found = object_found
		self.first_object_detected = True

		
		if(len(object_found.data) != 0):
			if(self.current_time - self.passed_time > rospy.Duration(1)):
				#TODO Define for each objects

				# Get the angle of the object according to the robot view.
				self.object_angle = self.find_angle_object(object_found)
				if (self.object_angle != 'NaN'):
					valide_length = True
				else:
					valide_length = False
				if valide_length == True:	
					if (object_found.data[0] == self.image_alive):
						#Add a color detector with openCV
						print('Alive person found')
						self.createMarker(Shape.SPHERE, 0.0, 1.0, 0.0)
					
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_toxic):
						#Add a color detector with openCV
						print('Toxic area found')
						self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_warning):
						#Add a color detector with openCV
						print('Warning are found')
						self.createMarker(Shape.CUBE, 1.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_injured):
						#Add a color detector with openCV
						print('Injured person found')
						self.createMarker(Shape.SPHERE, 1.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_fire):
						#Add a color detector with openCV
						print('Fire found')
						self.createMarker(Shape.CYLINDER, 1.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_no_smoke):
						#Add a color detector with openCV
						print('Radioactive area found')
						self.createMarker(Shape.CUBE, 0.5, 0.5, 0.5)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_radioactive):
						#Add a color detector with openCV
						print('Radioactive area found')
						self.createMarker(Shape.CUBE, 1.0, 1.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_dead):
						#Add a color detector with openCV
						print('Object 14 found')
						self.createMarker(Shape.CUBE, 1.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					if (object_found.data[0] == self.image_dead):
						#Add a color detector with openCV
						print('Object 14 found')
						self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
						self.markerPub.publish(self.robotMarker)
						self.count = self.count + 1
						self.passed_time = rospy.get_rostime()

					else:
						print("No object found")
					
				else:
					print("Object to far to get the distance")
			
	def find_angle_object(self, object_found):
		data = object_found.data
		camera_angle_conv = 0.10366
		camera_min_angle = -31.1
		camera_max_angle = 31.1
		selected_angle = 0
		if(len(object_found.data) != 0 and len(self.laserscan_data) != 0):
			for i in range(0, len(data), 12):

				# get data
				id = data[i]
				object_width = data[i+1]
				object_height = data[i+2]

				# Find the corner pose
				qtHomography = QTransform(data[i+3], data[i+4], data[i+5], data[i+6], data[i+7], data[i+8], data[i+9], data[i+10], data[i+11])
				qtTopLeft = qtHomography.map(QPointF(0,0))
				qtTopRight = qtHomography.map(QPointF(object_width,0))
				qtBottomLeft = qtHomography.map(QPointF(0,object_height))
				qtBottomRight = qtHomography.map(QPointF(object_width,object_height))
				#print("qtTopLeft is at: " + str(qtBottomRight.x()) + " and "+ str(qtBottomRight.y()))
				ximage = qtBottomRight.x() - (qtBottomRight.x() - qtBottomLeft.x()) / 2
				yimage = qtBottomRight.y() - (qtBottomRight.y() - qtTopRight.y()) / 2
				#print("The object is detected at : (x = " + str(ximage) + ", y = " + str(yimage) + ")")
				# Value of x is between 0 and 600 (0 = -31.1 deg and 600 = 31.1 deg)
				angle_deg = int(camera_min_angle + camera_angle_conv * ximage)
				distance = 100
			for j in range(angle_deg-10,angle_deg+10):
				if self.laserscan_data[j] != 'inf' and self.laserscan_data[j] < distance:
					distance = self.laserscan_data[j]
					selected_angle = int(j)
			print("Angle of the detected object" + str(int(selected_angle)))
			return selected_angle
		else:
			return 'NaN'
	"""
	def main(self):

		if self.first_object_detected == True:
			object_found = self.object_found
			if(len(object_found.data) != 0):
				if(self.current_time - self.passed_time > rospy.Duration(1)):
					#TODO Define for each objects

					# Get the angle of the object according to the robot view.
					self.object_angle = self.find_angle_object(object_found)
					if (self.object_angle != 'NaN'):
						valide_length = True
					else:
						valide_length = False
					if valide_length == True:	
						if (object_found.data[0] == self.image_alive):
							#Add a color detector with openCV
							print('Alive person found')
							self.createMarker(Shape.SPHERE, 0.0, 1.0, 0.0)
						
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_toxic):
							#Add a color detector with openCV
							print('Toxic area found')
							self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_warning):
							#Add a color detector with openCV
							print('Warning are found')
							self.createMarker(Shape.CUBE, 1.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_injured):
							#Add a color detector with openCV
							print('Injured person found')
							self.createMarker(Shape.SPHERE, 1.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_fire):
							#Add a color detector with openCV
							print('Fire found')
							self.createMarker(Shape.CYLINDER, 1.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_no_smoke):
							#Add a color detector with openCV
							print('Radioactive area found')
							self.createMarker(Shape.CUBE, 0.5, 0.5, 0.5)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_radioactive):
							#Add a color detector with openCV
							print('Radioactive area found')
							self.createMarker(Shape.CUBE, 1.0, 1.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_dead):
							#Add a color detector with openCV
							print('Object 14 found')
							self.createMarker(Shape.CUBE, 1.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						if (object_found.data[0] == self.image_dead):
							#Add a color detector with openCV
							print('Object 14 found')
							self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
							self.markerPub.publish(self.robotMarker)
							self.count = self.count + 1
							self.passed_time = rospy.get_rostime()

						else:
							print("No object found")
						
					else:
						print("Object to far to get the distance")
	"""


	def createMarker(self, shape, color_r, color_g, color_b):
		self.robotMarker = Marker()
		self.robotMarker.header.frame_id = 'odom'
		self.robotMarker.header.stamp    = rospy.Time.now()
		self.robotMarker.type = shape
		self.robotMarker.action = Marker.ADD
		self.robotMarker.pose.position = self.odom.pose.pose.position
		print("angle"+ str(self.object_angle))
		#angle_object = radians(self.object_angle)
		angle_object = self.object_angle
		print("x distance"+ str(self.laserscan_data[angle_object] * cos(radians(angle_object))))

		#self.robotMarker.pose.position = self.odom.pose.pose.position
		
		self.robotMarker.pose.position.x = self.odom.pose.pose.position.x + int(self.laserscan_data[self.object_angle] * cos(radians(angle_object)))
		self.robotMarker.pose.position.y = self.odom.pose.pose.position.y + int(self.laserscan_data[self.object_angle] * sin(radians(angle_object)))
		self.robotMarker.id = self.count
		self.robotMarker.scale.x = 0.1
		self.robotMarker.scale.y = 0.1
		self.robotMarker.scale.z = 0.1
		self.robotMarker.color.r = color_r;
		self.robotMarker.color.g = color_g;
		self.robotMarker.color.b = color_b;
		self.robotMarker.color.a = 1.0;

class Shape(IntEnum):
	CUBE = 1
	SPHERE = 2
	CYLINDER = 3

		
if __name__ == '__main__':
	MarkerRobot()
	
	rospy.spin()