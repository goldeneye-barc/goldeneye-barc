#!/usr/bin/env python
import rospy
from barc.msg import Encoder
from std_msgs.msg import Float32

def calculate_rotation(msg):
    global rotation
    pi = 3.14159265
    rotation_diameter = 8.75 * 2
    wheel_diameter = 3.9
    C = 0.374
    diff = msg.BR - msg.BL
    dist_diff = (wheel_diameter * pi * diff) / 8
    rotation = 360.0 * 0.374 * (dist_diff / (rotation_diameter * pi))
    return

def rotation_node():
    global rotation
    rotation = 0
    pub = rospy.Publisher('tick_rotation', Float32, queue_size = 5)
    rospy.init_node('rotation_node')
    sub = rospy.Subscriber('encoder', Encoder, calculate_rotation)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(rotation)
        rate.sleep()

if __name__ == '__main__':
    try:
        rotation_node()
    except rospy.ROSInterruptException:
        pass
