#!/usr/bin/env python
import rospy

from barc.msg import ECU
from std_msgs.msg import String
from marvelmind_nav.msg import hedge_pos

def callback(msg):
	x = msg.x_m
	y = msg.y_m
	print(x)
	print(y)

def gps_node():
	rospy.init_node('key_node')
	sub = rospy.Subscriber('hedge_pos', hedge_pos, callback)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
	try: 
		gps_node()
	except rospy.ROSInterruptException:
		pass
	