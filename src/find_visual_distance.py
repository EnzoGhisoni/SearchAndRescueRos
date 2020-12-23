#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import sys
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from enum import Enum, IntEnum
from sensor_msgs.msg import LaserScan
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
from math import *

"""
Run the camera view
rosrun image_view image_view image:=/image_rectangle
roslaunch integrated_robotics_project turtlebot3_search_and_rescue.launch
rosrun find_object_2d find_object_2d image:=/camera/rgb/image_raw
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration
roslaunch explore_lite explore.launch
"""


class Distance_estimation():
	def __init__(self):
		rospy.init_node("distance_estimation", anonymous=True)
		self.bridge = CvBridge()
		#for turtlebot3 waffle
		#image_topic = '/camera/rgb/image_raw/compressed'
		#for occupancy topic
		self.image_topic = '/map'
		self.resolution = 0.05
		self.rectangle_odom = 5
		#image_topic = '/camera/rgb/image_raw/'
		self.robot_odom_value = Odometry()
		#self.image_sub = rospy.Subscriber(self.image_topic, OccupancyGrid, self.occupancy_callback)
		self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.odom = rospy.Subscriber('/camera/rgb/image_raw/', Image, self.image_callback)
		self.object_sub = rospy.Subscriber('/objects', Float32MultiArray, self.object_cb)
		self.markerPub = rospy.Publisher('/robotMarker', Marker, queue_size=1)
		self.img = rospy.Publisher('/image_rectangle', Image, queue_size=10)

		self.image_message = Image()
		self.image = np.zeros((384, 384, 1), np.uint8)
		self.count = 0

		self.current_time = rospy.get_rostime()
		self.passed_time = rospy.get_rostime()

		# variable updated in topic
		self.actual_image = Image()
		self.actual_odom = Odometry()
		self.robot_yaw = 0
		# image identification
		self.image_person = 11
		self.image_toxic = 12
		self.image_warning = 13
		#self.image_injured = 14
		self.image_fire = 15
		self.image_no_smoke = 16
		self.image_radioactive = 17
		self.image_triangle = 18
		self.image_dead = 19

		self.object_distance = 0
		self.object_angle = 0
		self.listMeanMarker = []

		# people state
		self.alive = 0
		self.injured = 1
		self.person_state = 0

		self.mean_counter = 0
		self.previous_ref_marker = 0
	
	def image_callback(self, ros_image):
		self.actual_image = ros_image

	def odom_callback(self, ros_odom):
		self.actual_odom = ros_odom
		(_, _, self.robot_yaw) = euler_from_quaternion([ros_odom.pose.pose.orientation.x, 
				 ros_odom.pose.pose.orientation.y, ros_odom.pose.pose.orientation.z, ros_odom.pose.pose.orientation.w], 
				 'sxyz')

		"""
		image_to_send = np.zeros((384, 384, 1), np.uint8)
		
		image_to_send = self.image.copy() # without .copy mthod it write directly on self.image
		self.robot_odom_value = ros_odom
		odom_pose_x = int((self.robot_odom_value.pose.pose.position.x + 10)/(self.resolution))
		odom_pose_y = int((self.robot_odom_value.pose.pose.position.y + 10)/(self.resolution))
		print("The calculated value is " + str(odom_pose_x) +" and "+ str(odom_pose_y))
		cv2.rectangle(image_to_send, (odom_pose_x - 5, odom_pose_y - 5), (odom_pose_x + 5, odom_pose_y + 5), (0, 255, 0), 2)
		image_to_send = cv2.rotate(image_to_send, cv2.ROTATE_90_CLOCKWISE)
		image_to_send = cv2.flip(image_to_send, 0)
		#image_to_send = cv2.rotate(image_to_send, cv2.ROTATE_180)
		image_message = bridge.cv2_to_imgmsg(image_to_send, "mono8")
		
		self.img.publish(image_message)
		"""
	
	def object_cb(self, object_found):
		self.current_time = rospy.get_rostime()
		self.object_found = object_found
		self.first_object_detected = True

		
		if(len(object_found.data) != 0):
			#if(self.current_time - self.passed_time > rospy.Duration(1)):
			#TODO Define for each objects
			self.draw_rectangle_on_roi(object_found)
		else:
			self.img.publish(self.actual_image)


	def draw_rectangle_on_roi(self, object_found):

		data = object_found.data
		focal_length_mm = 3.04
		sensor_width_mm = 3.68
		#image_width_px = 1920
		image_width_px = 640
		focal_length_px = (focal_length_mm /sensor_width_mm) * image_width_px

		#camera_angle_conv = 0.0334
		camera_angle_conv = 0.10366
		camera_min_angle = -31.1
		camera_max_angle = 31.1

		object_size = 1
		diagonal_cube = sqrt(2)
		cv_image = self.bridge.imgmsg_to_cv2(self.actual_image, "bgr8")

		if(len(object_found.data) != 0):
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
				height = abs(qtBottomRight.y() - qtTopRight.y())
				ximage = qtBottomRight.x() - (qtBottomRight.x() - qtBottomLeft.x()) / 2
				yimage = qtBottomRight.y() - (qtBottomRight.y() - qtTopRight.y()) / 2
				distance = (focal_length_px * object_size) / height
				self.object_distance = distance
				angle_deg = int(camera_min_angle + camera_angle_conv * ximage)
				self.object_angle = angle_deg
				# Draw the rectangle around the object
				font = cv2.FONT_HERSHEY_SIMPLEX
				cv2.rectangle(cv_image, (int(qtTopLeft.x()), int(qtTopLeft.y())), (int(qtBottomRight.x()), int(qtBottomRight.y())), (0, 255, 0), 2)
				cv2.putText(cv_image, "("+str(round(distance, 2)) + "m," + str(round(angle_deg, 2)) + "degree" + ")", (int(qtTopLeft.x()), int(qtTopLeft.y())), font, 2, (255, 0, 255), 2)
				#cv2.putText(cv_image, str(round(distance, 2)) + "m", (int(qtTopLeft.x()), int(qtTopLeft.y())), font, 2, (255, 0, 255), 2)
				image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
				print(distance)
				
				if (object_found.data[i] == self.image_person):
					#Add a color detector with openCV
					state = self.detect_people_state()
					if(state == self.alive):
						print('Alive person found')
						self.createMarker(Shape.SPHERE, 0.0, 1.0, 0.0)
						self.createMeanMarker(Shape.SPHERE, 0.0, 1.0, 0.0, object_found.data[i])
					else:
						print('Injured person found')
						self.createMarker(Shape.SPHERE, 1.0, 0.0, 0.0)
						self.createMeanMarker(Shape.CUBE, 1.0, 0.0, 0.0, object_found.data[i])
					self.person_state = state
					

				if (object_found.data[i] == self.image_toxic):
					#Add a color detector with openCV
					print('Toxic area found')
					self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
					self.createMeanMarker(Shape.SPHERE, 0.0, 0.0, 0.0, object_found.data[i])

				if (object_found.data[i] == self.image_warning):
					#Add a color detector with openCV
					print('Warning are found')
					self.createMarker(Shape.CUBE, 1.0, 0.0, 1.0)
					self.createMeanMarker(Shape.CUBE, 1.0, 0.0, 0.0, object_found.data[i])
					
				if (object_found.data[i] == self.image_fire):
					#Add a color detector with openCV
					print('Fire found')
					self.createMarker(Shape.CYLINDER, 1.0, 0.5, 0.0)
					self.createMeanMarker(Shape.CYLINDER, 1.0, 0.5, 0.0, object_found.data[i])

				if (object_found.data[i] == self.image_no_smoke):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.createMarker(Shape.CUBE, 0.5, 0.5, 0.5)
					self.createMeanMarker(Shape.CUBE, 0.5, 0.5, 0.5, object_found.data[i])
				

				if (object_found.data[i] == self.image_radioactive):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.createMarker(Shape.CUBE, 1.0, 1.0, 0.0)
					self.createMeanMarker(Shape.CUBE, 1.0, 1.0, 0.0, object_found.data[i])

				if (object_found.data[i] == self.image_dead):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.createMarker(Shape.SPHERE, 0.0, 0.0, 0.0)
					self.createMeanMarker(Shape.SPHERE, 0.0, 0.0, 0.0, object_found.data[i])
					
				else:
					print("No object found")
			self.img.publish(image_message)
			
		else:
			print("Nothing detected")
			

	def createMarker(self, shape, color_r, color_g, color_b):
		self.robotMarker = Marker()
		self.robotMarker.header.frame_id = 'odom'
		self.robotMarker.header.stamp    = rospy.Time.now()
		self.robotMarker.type = shape
		self.robotMarker.action = Marker.ADD
		self.robotMarker.pose.position = self.actual_odom.pose.pose.position
		
		#self.robotMarker.pose.position = self.odom.pose.pose.position
		absolut_angle = self.robot_yaw - radians(self.object_angle)
		#print(self.object_angle)
		#print(self.robot_yaw)
		#print(absolut_angle)
		self.robotMarker.pose.position.x = self.actual_odom.pose.pose.position.x + self.object_distance * cos(absolut_angle)
		self.robotMarker.pose.position.y = self.actual_odom.pose.pose.position.y + self.object_distance * sin(absolut_angle)
		self.robotMarker.id = self.count

		self.robotMarker.scale.x = 0.1
		self.robotMarker.scale.y = 0.1
		self.robotMarker.scale.z = 0.1
		self.robotMarker.color.r = color_r;
		self.robotMarker.color.g = color_g;
		self.robotMarker.color.b = color_b;
		self.robotMarker.color.a = 1.0;
		#self.markerPub.publish(self.robotMarker)
		self.count = self.count + 1
		self.passed_time = rospy.get_rostime()

	def computeMeanMarkers(self, marker_ID):
		min_size_list = 3
		x_mean = 0
		y_mean = 0
		# Create new object Marker to store in the list
		absolut_angle = self.robot_yaw - radians(self.object_angle)
		x_marker = self.actual_odom.pose.pose.position.x + self.object_distance * cos(absolut_angle)
		y_marker = self.actual_odom.pose.pose.position.y + self.object_distance * sin(absolut_angle)

		robotMarker = ObjectMarkerInformation(marker_ID, x_marker, y_marker)

		
		if(robotMarker.ref_object == self.previous_ref_marker):
			self.markerPub.publish(self.robotMarker)
			print("Add the new marker")
			self.listMeanMarker.append(robotMarker)
			self.mean_counter+=1
		else:
			print('Compute the mean')
			#if(len(self.listMeanMarker) > min_size_list):
			if(self.mean_counter > min_size_list):
				#print("Size of the list = " + str(len(self.listMeanMarker)))
				#for i in range(0, self.mean_counter):
				nb_marker = 0
				for i in range(0, len(self.listMeanMarker)):
					x_mean += self.listMeanMarker[i].x
					y_mean += self.listMeanMarker[i].y
					nb_marker = nb_marker + 1

				print("Size list is :" + str(self.mean_counter))
				print("Counter is :"+ str(nb_marker))
				# Create mean marker
				#self.robotMarker.pose.position.x = x_mean / (len(self.listMeanMarker))
				#self.robotMarker.pose.position.y = y_mean / (len(self.listMeanMarker))
				self.robotMarker.pose.position.x = x_mean / (nb_marker)
				self.robotMarker.pose.position.y = y_mean / (nb_marker)
				self.robotMarker.id = self.count
				self.robotMarker.scale.x = 0.5
				self.robotMarker.scale.y = 0.5
				self.robotMarker.scale.z = 0.5
				if (self.previous_ref_marker == self.image_person):
				
					if(self.person_state == self.alive):
						print('Alive person found')
						self.robotMarker.color.r = 0.0;
						self.robotMarker.color.g = 1.0;
						self.robotMarker.color.b = 0.0;

					else:
						print('Injured person found')
						self.robotMarker.color.r = 1.0;
						self.robotMarker.color.g = 0.0;
						self.robotMarker.color.b = 0.0;
					
				if (self.previous_ref_marker == self.image_toxic):
					#Add a color detector with openCV
					print('Toxic area found')
					self.robotMarker.color.r = 0.0;
					self.robotMarker.color.g = 0.0;
					self.robotMarker.color.b = 0.0;

				if (self.previous_ref_marker == self.image_warning):
					#Add a color detector with openCV
					print('Warning are found')
					self.robotMarker.color.r = 1.0;
					self.robotMarker.color.g = 0.0;
					self.robotMarker.color.b = 1.0;
					
					
				if (self.previous_ref_marker == self.image_fire):
					#Add a color detector with openCV
					print('Fire found')
					self.robotMarker.color.r = 1.0;
					self.robotMarker.color.g = 0.5;
					self.robotMarker.color.b = 0.0;

				if (self.previous_ref_marker == self.image_no_smoke):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.robotMarker.color.r = 0.5;
					self.robotMarker.color.g = 0.5;
					self.robotMarker.color.b = 0.5;
				
				if (self.previous_ref_marker == self.image_radioactive):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.robotMarker.color.r = 1.0;
					self.robotMarker.color.g = 1.0;
					self.robotMarker.color.b = 0.0;

				if (self.previous_ref_marker == self.image_dead):
					#Add a color detector with openCV
					print('Radioactive area found')
					self.robotMarker.color.r = 0.0;
					self.robotMarker.color.g = 0.0;
					self.robotMarker.color.b = 0.0;
				
				self.robotMarker.color.a = 1.0;
				self.markerPub.publish(self.robotMarker)
				print('List published so cleaned')
				del self.listMeanMarker[:]
				#print('List is clean: ' + str(self.listMeanMarker))
				self.count = self.count + 1
				self.mean_counter = 0
				self.passed_time = rospy.get_rostime()
			else:
				print("Serie to short")
				print('List cleaned')
				del self.listMeanMarker[:]
		
		self.previous_ref_marker = marker_ID

	def createMeanMarker(self, shape, color_r, color_g, color_b, marker_ID):
		
			
		self.robotMarker = Marker()
		self.robotMarker.header.frame_id = 'odom'
		self.robotMarker.header.stamp = rospy.Time.now()
		self.robotMarker.type = shape
		self.robotMarker.action = Marker.ADD
		
		self.robotMarker.color.r = color_r;
		self.robotMarker.color.g = color_g;
		self.robotMarker.color.b = color_b;
		
		self.computeMeanMarkers(marker_ID)



	"""
	this function takes the last image capture and look for green or red
	if red detected = injured, green = alive
	"""
	def detect_people_state(self):
		widght = 0
		height = 0
		# We use cv bridge to convert the image in opencv format
		cv_people_image = self.bridge.imgmsg_to_cv2(self.actual_image, "bgr8")
		hsv = cv2.cvtColor(cv_people_image, cv2.COLOR_BGR2HSV)
		#redLower = (0, 0, 30)
		#redUpper = (80, 80, 255)
		#redmask = cv2.inRange(hsv, redLower, redUpper)
		# define a mask using the lower and upper bound of the green color
		greenLower = (40, 40,40)
		greenUpper = (70, 255,255)
		greenmask = cv2.inRange(hsv, greenLower, greenUpper)

		_, contours, _ = cv2.findContours(greenmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		if len(contours) > 0:
			green_area = max(contours, key=cv2.contourArea)
			(xg, yg, widght, height) = cv2.boundingRect(green_area)
			print("widght and height : ("+ str(widght) +", "+str(height))
		# Check a minimal size of mask to avoid false detections
		if(widght > 10 and height > 10):
			state = self.alive
		else:
			state = self.injured
		return state

class ObjectMarkerInformation():
  """Object to contain the marker"""
  def __init__(self, ref_object, x, y):
	self.ref_object = ref_object
	self.x = x
	self.y = y


class Shape(IntEnum):
	CUBE = 1
	SPHERE = 2
	CYLINDER = 3

def main(args):
	Distance_estimation()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
