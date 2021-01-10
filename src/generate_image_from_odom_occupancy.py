#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry, MapMetaData
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import sys
bridge = CvBridge()

class ROS_image_occupancy_creator():
	def __init__(self):
		rospy.init_node("map_creator", anonymous=True)
		self.bridge = CvBridge()
		#for turtlebot3 waffle
		#image_topic = '/camera/rgb/image_raw/compressed'
		#for occupancy topic
		self.image_topic = '/map'
		self.resolution = 0.05
		self.rectangle_odom = 5
		self.listMeanMarker = []
		#image_topic = '/camera/rgb/image_raw/'
		self.robot_odom_value = Odometry()
		self.image_sub = rospy.Subscriber(self.image_topic, OccupancyGrid, self.occupancy_callback)
		self.odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.markerSub = rospy.Subscriber('/robotMeanMarker', Marker, self.marker_callback)
		self.img = rospy.Publisher('image_from_occupancy', Image, queue_size=10)
		self.image_message = Image()
		self.image = np.zeros((384, 384, 1), np.uint8)
		self.map_info = MapMetaData()
		#self.image = np.zeros((1920, 1080, 1), np.uint8)
		self.count = 0
		self.occupancy_received = False

		# image identification
		self.alive = 11
		self.image_toxic = 12
		self.image_warning = 13
		#self.image_injured = 14
		self.image_fire = 15
		self.image_no_smoke = 16
		self.image_radioactive = 17
		self.image_triangle = 18
		self.image_dead = 19
		self.injured = 20
	

	def occupancy_callback(self, ros_occupancy):

		print 'got an OccupancyGrid'
		image = np.zeros((ros_occupancy.info.height, ros_occupancy.info.width, 1), np.uint8)
		self.map_info = ros_occupancy.info
		self.occupancy = ros_occupancy.data
		raw_data = np.reshape(ros_occupancy.data, (-1, max(ros_occupancy.info.width, ros_occupancy.info.height)))
		#print (raw_data)
		#cv2.imshow("Image Panel", image)
		# convert ros_occupancy into an opencv-compatible image
		for raw in range(0, ros_occupancy.info.height):
			for cols in range(0,ros_occupancy.info.width):
				if(raw_data[raw, cols] == -1):
					image[raw, cols] = 125
				if(raw_data[raw, cols] == 100):
					image[raw, cols] = 0
				if(raw_data[raw, cols] == 0):
					image[raw, cols] = 255

		self.image = image
		self.occupancy_received = True

		
		#intermedate_image.publish(image_message)

	def odom_callback(self, ros_odom):
		if self.occupancy_received == True:
			font = cv2.FONT_HERSHEY_SIMPLEX
			thickness = 2
			image_to_send = np.zeros((self.map_info.height, self.map_info.width, 1), np.uint8)
			self.resolution = self.map_info.resolution
			image_to_send = self.image.copy() # without .copy mthod it write directly on self.image
			self.robot_odom_value = ros_odom
			odom_pose_x = int((self.robot_odom_value.pose.pose.position.x + 10)/(self.resolution))
			odom_pose_y = int((self.robot_odom_value.pose.pose.position.y + 10)/(self.resolution))

			print("The calculated value is " + str(odom_pose_x) +" and "+ str(odom_pose_y))
			cv2.rectangle(image_to_send, (odom_pose_x - 5, odom_pose_y - 5), (odom_pose_x + 5, odom_pose_y + 5), (0, 255, 0), 2)
			if(len(self.listMeanMarker) > 0):
				for i in range(0,len(self.listMeanMarker)):
					objectPoseX = int((self.listMeanMarker[i].pose.position.x + 10)/(self.resolution))
					objectPoseY = int((self.listMeanMarker[i].pose.position.y + 10)/(self.resolution))
					if (self.listMeanMarker[i].id == self.alive):
						print('Alive person found')
						cv2.putText(image_to_send, "A", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
					if (self.listMeanMarker[i].id == self.injured):
						print('Injured person found')
						cv2.putText(image_to_send, "I", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
						

					if (self.listMeanMarker[i].id == self.image_toxic):
						#Add a color detector with openCV
						print('Toxic area found')
						cv2.putText(image_to_send, "T", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

					if (self.listMeanMarker[i].id == self.image_warning):
						#Add a color detector with openCV
						print('Warning are found')
						cv2.putText(image_to_send, "W", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
						
					if (self.listMeanMarker[i].id == self.image_fire):
						#Add a color detector with openCV
						print('Fire found')
						cv2.putText(image_to_send, "F", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

					if (self.listMeanMarker[i].id == self.image_no_smoke):
						#Add a color detector with openCV
						print('Smoke area found')
						cv2.putText(image_to_send, "S", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
					

					if (self.listMeanMarker[i].id == self.image_radioactive):
						#Add a color detector with openCV
						print('Radioactive area found')
						cv2.putText(image_to_send, "R", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

					if (self.listMeanMarker[i].id == self.image_dead):
						#Add a color detector with openCV
						print('Radioactive area found')
						cv2.putText(image_to_send, "D", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

			image_to_send = cv2.rotate(image_to_send, cv2.ROTATE_90_CLOCKWISE)
			image_to_send = cv2.flip(image_to_send, 0)
			#image_to_send = cv2.rotate(image_to_send, cv2.ROTATE_180)
			image_message = bridge.cv2_to_imgmsg(image_to_send, "mono8")
			self.img.publish(image_message)

	def marker_callback(self, maker_message):
		self.listMeanMarker.append(maker_message)
	"""
	def writeMarkerOnMap(self, image_to_send):
		if(len(self.listMeanMarker) > 0):
			for i in range(0,len(self.listMeanMarker)):
				objectPoseX = int((self.listMeanMarker[i].pose.position.x + 10)/(self.resolution))
				objectPoseY = int((self.listMeanMarker[i].pose.position.y + 10)/(self.resolution))
				if (self.listMeanMarker[i].id == self.alive):
					print('Alive person found')
					cv2.putText(image_to_send, "A", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
				if (self.listMeanMarker[i].id == self.injured):
					print('Injured person found')
					cv2.putText(image_to_send, "I", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
					

				if (self.listMeanMarker[i].id == self.image_toxic):
					#Add a color detector with openCV
					print('Toxic area found')
					cv2.putText(image_to_send, "T", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

				if (self.listMeanMarker[i].id == self.image_warning):
					#Add a color detector with openCV
					print('Warning are found')
					cv2.putText(image_to_send, "W", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
					
				if (self.listMeanMarker[i].id == self.image_fire):
					#Add a color detector with openCV
					print('Fire found')
					cv2.putText(image_to_send, "F", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

				if (self.listMeanMarker[i].id == self.image_no_smoke):
					#Add a color detector with openCV
					print('Radioactive area found')
					cv2.putText(image_to_send, "S", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)
				

				if (self.listMeanMarker[i].id == self.image_radioactive):
					#Add a color detector with openCV
					print('Radioactive area found')
					cv2.putText(image_to_send, "R", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

				if (self.listMeanMarker[i].id == self.image_dead):
					#Add a color detector with openCV
					print('Radioactive area found')
					cv2.putText(image_to_send, "D", (objectPoseX, objectPoseY), font, 1, (0, 0, 0), thickness)

				#cv2.putText(image_to_send, str(self.listMeanMarker[i].id), (objectPoseX, objectPoseY), font, 1, (0, 0, 0), 1)
	"""

def main(args):
	ROS_image_occupancy_creator()
	try:
		rospy.spin()
	except rospy.KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
