#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


def image_process(image):
	bridge = CvBridge()
	global error
	try:
		cv_image = bridge.imgmsg_to_cv2(image)
		
		cols = []
		x,y = cv_image.shape
		midx = int(round(x/2))
		midy = int(round(y/2))
		cv_bottom = cv_image[midx:,:]
		
		cols = np.zeros(y)
		cv_image = cv2.Canny(cv_image, 80,150)
		for i in range(midx,x,20):
			for j in range(midy - 400, midy + 400):
				if cv_image[i,j] == 255:
					cols[j] += 1

		
		npix = 0
		tfrac = 0
		for k in range(0,y-1):
			tfrac += k*cols[k]
			npix += cols[k]

		if npix == 0:
			avgcol = midy
		else:
			avgcol = tfrac/npix

		error = (avgcol - midy)/400

		print(error)
	
		cv2.imshow('full image', cv_image)
		#cv2.imshow('bottom half',cv_bottom)
		cv2.waitKey(1)
	except CvBridgeError as e:
		print(e)

def image_node():
	global error
	error = 0

	rospy.init_node('Test_Imager')
	rospy.Subscriber('image_raw', Image, image_process)
	pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)
	rate = rospy.Rate(10)
	
	K = 250

	while not rospy.is_shutdown():
		msg = ECU()
		msg.motor = 1550
		msg.servo = 1500 + K*error
		pub.publish(msg)
		rate.sleep()


if __name__ == '__main__':
	try: 
		image_node()
	except rospy.ROSInterruptException:
		pass