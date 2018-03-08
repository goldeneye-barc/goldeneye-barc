#!/usr/bin/env python
import rospy
from barc.msg import ECU
from std_msgs.msg import String
from marvelmind_nav.msg import hedge_pos

def callback(msg):
    global x,y,x0,y0
    x = msg.x_m
    y = msg.y_m
    if (x0 == -500) & (y0 == -500):
        x0 = x
        y0 = y

    

def gps_node():
    global x,y,x0,y0
    x0 = -500
    y0 = -500
    x = -500
    y = -500
    ei = 0
    rospy.init_node('gps_node')
    pub = rospy.Publisher('ecu_pwm',ECU,queue_size = 5)
    sub = rospy.Subscriber('hedge_pos',hedge_pos, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dist =((x-x0)**2 + (y-y0)**2)**.5
        e = 1 - dist
        print(e,ei)
        msg = ECU()
        msg.servo = 1500
        if e>0:
            msg.motor = 1500 + 120*e
        if e<0:
            msg.motor = 1500 + 400*e
        pub.publish(msg)
        rate.sleep()

if __name__ =='__main__' :
    try:
        gps_node()
    except rospy.ROSInterruptException:
        pass
