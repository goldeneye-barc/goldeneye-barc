#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from std_msgs.msg import String

def key_node():
	pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)
	rospy.init_node('key_node')
	rate = rospy.Rate(10)
	print('Use aswd to navigate the BARC, use q to stop')
	while not rospy.is_shutdown():
		char = getch.getch()
		msg = ECU()
		if char == 'a':
			msg.motor = 1600
			msg.servo = 1200
			pub.publish(msg)
		elif char == 'e':
			msg.motor = 1300
			msg.servo = 1800
			pub.publish(msg)
		elif char == 'q':
			msg.motor = 1300
			msg.servo = 1200
			pub.publish(msg)
		elif char == 's':
			msg.motor = 1300
			msg.servo = 1500
			pub.publish(msg)
		elif char == 'w':
			msg.motor = 1600
			msg.servo = 1500
			pub.publish(msg)
		elif char == 'd':
			msg.motor = 1600
			msg.servo = 1800
			pub.publish(msg)
		elif char == 'r':
			msg.motor = 1600
			msg.servo = 1800
			pub.publish(msg)
		elif char == 'f': exit(0)

if __name__ == '__main__':
	try: 
		key_node()
	except rospy.ROSInterruptException:
		pass
	
