#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
"""

"""
#from PyQt5.QtGui import QPainter, QTransform
#rosrun find_object_2d find_object_2d image:=/camera/rgb/image_raw
class FindPosition:
	def __init__(self):
		rospy.init_node('object_pose')
		#self.count = 0
		#self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
		#self.odom_sub = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb)
		self.object_sub = rospy.Subscriber('/objects', Float32MultiArray, self.object_detect_cb)
		#self.markerPub = rospy.Publisher('/robotMarker', Marker, queue_size=1)
		self.object = Float32MultiArray()
		

	def object_detect_cb(self, object_found):
		data = object_found.data
		camera_angle_conv = 0.10366
		camera_min_angle = -31.1
		camera_max_angle = 31.1
		#print("Object size is : " + str(data))
		print('function launched')
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
				ximage = qtBottomRight.x() - (qtBottomRight.x() - qtBottomLeft.x()) / 2
				yimage = qtBottomRight.y() - (qtBottomRight.y() - qtTopRight.y()) / 2
				#print("The object is detected at : (x = " + str(ximage) + ", y = " + str(yimage) + ")")
				# Value of x is between 0 and 600 (0 = -31.1 deg and 600 = 31.1 deg)
				angle_deg = camera_min_angle + camera_angle_conv * ximage
			print("Angle of the detected object" + str(int(angle_deg)))

if __name__ == '__main__':
	FindPosition()
	rospy.spin()