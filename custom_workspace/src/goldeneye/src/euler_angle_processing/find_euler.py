#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from math import sin, cos

def do_ode(t, z):
    global wx, wy, wz
    return [wx + wy*sin(z[1])*sin(z[0])/cos(z[1]) + wz*sin(z[1])*cos(z[0])/cos(z[0]),\
    wy*cos(z[0]) - wz*sin(z[0]), \
    wy*sin(z[0])/cos(z[1]) + wz*cos(z[1])/cos(z[1])]

def grab_omegas(data):
    global wx, wy, wz
    wx = data.angular_velocity.x
    wy = data.angular_velocity.y
    wz = data.angular_velocity.z

def euler_angles():
    global wx, wy, wz
    rospy.init_node('euler_approx')
    rospy.Subscriber('/imu/data',Imu,grab_omegas,queue_size=10)
    pub = rospy.Publisher('/euler_angles',Vector3)
    rate = rospy.Rate(10)
    wx = 0
    wy = 0
    wz = 0
    dt = 1/rate
    r = ode(do_ode
    while not rospy.is_shutdown():
        outmsg = Vector3()
        



        rate.sleep()

if __name__ == '__main__':
    try:
        euler_angles()
    except rospy.ROSInterruptException:
        pass
