#! /usr/bin/env python
import rospy
from math import pi
from std_msgs.msg import Float32
from barc.msg import Encoder

def encoder_values(data):

    global bl, br
    bl = data.BL
    br = data.BR 

def vel_est():

    global bl, br
    rospy.init_node('Velocity_Est')
    rospy.Subscriber('encoder', Encoder, encoder_values, queue_size=10)
    pub = rospy.Publisher('velocity_est', Float32, queue_size=10)
    rate = rospy.Rate(10)

    bl0 = 0
    br0 = 0
    bl = 0
    br = 0
    t0 = rospy.get_time()
    t0left = t0
    t0right = t0
    vleft = 0
    vright = 0
    R = 2 #(radius of wheel in inches)

    while not rospy.is_shutdown():
        msg = Float32()

        curtime = rospy.get_time() - t0

        if bl - bl0 != 0:
            bldot = (bl - bl0)/(curtime - t0left)
            wleft = bldot*(pi/4)
            vleft = wleft*R
            t0left = curtime

        if br - br0 != 0:
            brdot = (br - br0)/(curtime - t0right)
            wright = brdot*(pi/4)
            vright = wright*R
            t0right = curtime

	msg.data = (vleft + vright)/2
	pub.publish(msg)

	bl0 = bl
	br0 = br
	rate.sleep()

if __name__ == '__main__':
    try:
	    vel_est()
    except rospy.ROSInterruptException:
	    pass
