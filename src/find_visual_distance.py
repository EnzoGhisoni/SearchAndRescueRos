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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from enum import Enum, IntEnum
from sensor_msgs.msg import LaserScan
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
from math import *

bridge = CvBridge()

class Distance_estimation():
	def __init__(self):
		rospy.init_node("image_converter", anonymous=True)
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
		# image identification
		self.image_alive = 11
		self.image_toxic = 12
		self.image_warning = 13
		self.image_injured = 14
		self.image_fire = 15
		self.image_no_smoke = 16
		self.image_radioactive = 17
		self.image_triangle = 18
		self.image_dead = 19
	
	def image_callback(self, ros_image):
		self.actual_image = ros_image

	def odom_callback(self, ros_odom):
		self.actual_odom = ros_odom

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

	def draw_rectangle_on_roi(self, object_found):
		data = object_found.data
		focal_length_mm = 3.04
		sensor_width_mm = 3.68
		image_width_px = 1920
		focal_length_px = (focal_length_mm /sensor_width_mm) * image_width_px

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

			#cv2.rectangle(image, (xg, yg), (xg + wg, yg + hg), (0, 255, 0), 2)
			cv2.rectangle(cv_image, (int(qtTopLeft.x()), int(qtTopLeft.y())), (int(qtBottomRight.x()), int(qtBottomRight.y())), (0, 255, 0), 2)
			image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")
			distance = (focal_length_px * object_size) / height
			print(distance)
			self.img.publish(image_message)
			
		else:
			print("Nothing detected")
			self.img.publish(self.actual_image)

	def createMarker(self, shape, color_r, color_g, color_b):
		self.robotMarker = Marker()
		self.robotMarker.header.frame_id = 'odom'
		self.robotMarker.header.stamp    = rospy.Time.now()
		self.robotMarker.type = shape
		self.robotMarker.action = Marker.ADD
		self.robotMarker.pose.position = self.actual_odom.pose.pose.position
		
		

		#self.robotMarker.pose.position = self.odom.pose.pose.position
		
		self.robotMarker.pose.position.x = self.actual_odom.pose.pose.position.x
		self.robotMarker.pose.position.y = self.actual_odom.pose.pose.position.y
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

def main(args):
	Distance_estimation()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
