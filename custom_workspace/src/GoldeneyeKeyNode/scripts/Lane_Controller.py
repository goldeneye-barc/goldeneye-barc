#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from std_msgs.msg import String

def callback(input_data):
	global servo_value
	#servo_value = input_data * 300 + 1500
	servo_value = input_data.servo
	#servo_value = input_data.data

def lane_controller():

	global servo_value
	servo_value = None
	# Input
	rospy.Subscriber('ecu_pwm', ECU, callback)

	# Node
	rospy.init_node('lane_controller')

	# Publisher
	pub = rospy.Publisher('test', String, queue_size=10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		"""
		msg = ECU()
		msg.motor = motor_speed
		msg.servo = servo_value"""
		# servo_value = 0
		if servo_value:
			print servo_value
		servo_value = None
		pub.publish(servo_value)
		print "publishing: ", servo_value
		rate.sleep()


if __name__ == '__main__':
	try:
		lane_controller()
	except rospy.ROSInterruptException:
		pass