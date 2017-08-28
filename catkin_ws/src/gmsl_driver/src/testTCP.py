#!/usr/bin/python

import socket

import cv2

#import rospy
import numpy
#import cv_bridge
#from sensor_msgs.msg import Image

class Client:
	def __init__(self, ip='localhost', port=8000):
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.connect((ip, int(port)))

	def recvFrame(self):
		# Read first the message size
		data = b""
		while len(data) < 4:
			packet = self.s.recv(4 - len(data))
			if not packet: return None, None
			data += packet
		message_size = numpy.fromstring(data, dtype='uint32')[0]

		# Read the message
		data = b""
		while len(data) < message_size:
			packet = self.s.recv(message_size - len(data))
			if not packet: return None, None
			data += packet

		cameraID = numpy.fromstring(data[0], dtype='uint8')[0]
		width = numpy.fromstring(data[1:3], dtype='uint16')[0]
		height = numpy.fromstring(data[3:5], dtype='uint16')[0]
                frame = numpy.resize(numpy.fromstring(data[5:], dtype='uint8'), (height, width, 3))

		return cameraID, frame

	def close(self):
		self.s.close()

def publish(client):
	#bridge = cv_bridge.CvBridge()

	while True:
		cameraID, image = client.recvFrame()

		if image is not None:
			#### Temporary
			cv2.imshow('Video', image)
			cv2.waitKey(1)
			####

			#msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')			
			#pub.publish(msg)

if __name__ == '__main__':
	#rospy.init_node('gmsl-camera', anonymous=True)
	#pub = rospy.Publisher('/thermal-camera/image_raw', Image, queue_size=10)
	client = Client(ip='192.168.1.52', port=6969)
	publish(client)
	client.close()
