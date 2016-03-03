#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from math import sin, cos
from scipy.integrate import ode

def do_ode(t, z, wx, wy, wz):
   # return [wx + (wy*sin(z[1])*sin(z[0])/cos(z[1])) + (wz*sin(z[1])*cos(z[0])/cos(z[1])),\
   # wy*cos(z[0]) - wz*sin(z[0]), \
   # wy*sin(z[0])/cos(z[1]) + wz*cos(z[0])/cos(z[1])]
   return [wx,wy,wz]

def grab_omegas(data):
    global wx, wy, wz
    wtol = 0.01
    wx = data.angular_velocity.x
    if abs(wx) <= wtol:
        wx = 0
      # wx = wx
    wy = data.angular_velocity.y
    if abs(wy) <= wtol:
        wy = 0
      # wy = wy
    wz = -data.angular_velocity.z
    if abs(wz) <= wtol:
       # wz = wz
        wz = 0

def euler_angles():
    global wx, wy, wz
    rospy.init_node('euler_approx')
    rospy.Subscriber('/imu/data',Imu,grab_omegas,queue_size=10)
    pub = rospy.Publisher('/euler_angles',Vector3,queue_size=10)
    rate = rospy.Rate(10)
    wx = 0
    wy = 0
    wz = 0
    dt = 1
    rate.sleep()
    yaw = 0
    r = ode(do_ode).set_integrator('vode',method='adams',order=10,\
    atol=1e-6,with_jacobian=False)
    tcap = rospy.get_time()
    r.set_initial_value([wx,wy,wz],0)
    while not rospy.is_shutdown():# and r.successful():
        outmsg = Vector3()
#        r.set_f_params(wx,wy,wz)
#        r.integrate(r.t + dt)       
#        outmsg.x = r.y[0]
#        outmsg.y = r.y[1]
#        outmsg.z = r.y[2]
      #  outmsg.x = wx
      #  outmsg.y = wy
      #  outmsg.z = wz
        outmsg.x = 0
        outmsg.y = 0
        yaw  = yaw + wz*(rospy.get_time() - tcap)
        outmsg.z = yaw
        tcap = rospy.get_time()
        pub.publish(outmsg)
        rate.sleep()
    #raise Exception('ode failed')

if __name__ == '__main__':
    try:
        euler_angles()
    except rospy.ROSInterruptException:
        pass
