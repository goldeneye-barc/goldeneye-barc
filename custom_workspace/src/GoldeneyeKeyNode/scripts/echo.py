#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from barc.msg import ECU

def callback(msg):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.servo)
	print msg.servo


def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('ecu_pwm', ECU, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()