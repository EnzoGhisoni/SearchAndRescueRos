#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from enum import Enum, IntEnum
from sensor_msgs.msg import LaserScan
#rosrun find_object_2d find_object_2d image:=/camera/rgb/image_raw
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
		self.laserscan_data = []
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

	def odom_cb(self, odom):
		self.odom = odom

	def laserscan_cb(self, msg):
		self.laserscan_data = msg.ranges
		print (len(self.laserscan_data))

	def object_cb(self, object_found):
		self.current_time = rospy.get_rostime()
		
		if(len(object_found.data) != 0):
			if(self.current_time - self.passed_time > rospy.Duration(1)):
				#TODO Define for each objects
				if (object_found.data[0] == self.image_alive):
					#Add a color detector with openCV
					print('Alive person found')
					self.createMarker(Shape.SPHERE, 0.0, 1.0, 0.0)
					"""self.robotMarker = Marker()
					self.robotMarker.header.frame_id = 'odom'
					self.robotMarker.header.stamp    = rospy.Time.now()
					self.robotMarker.type = Marker.SPHERE
					self.robotMarker.action = Marker.ADD
					self.robotMarker.pose.position = self.odom.pose.pose.position
					self.robotMarker.id = self.count
					self.robotMarker.scale.x = 0.1
					self.robotMarker.scale.y = 0.1
					self.robotMarker.scale.z = 0.1
					self.robotMarker.color.r = 0.0;
					self.robotMarker.color.g = 1.0;
					self.robotMarker.color.b = 0.0;
					self.robotMarker.color.a = 1.0;"""
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

	def createMarker(self, shape, color_r, color_g, color_b):
		self.robotMarker = Marker()
		self.robotMarker.header.frame_id = 'odom'
		self.robotMarker.header.stamp    = rospy.Time.now()
		self.robotMarker.type = shape
		self.robotMarker.action = Marker.ADD
		self.robotMarker.pose.position = self.odom.pose.pose.position
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