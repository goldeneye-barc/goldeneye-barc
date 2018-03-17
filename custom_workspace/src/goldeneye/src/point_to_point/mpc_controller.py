#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from barc.msg import ECU
from math import sin, cos
import numpy as np
#from marvelmind_nav.msg import hedge_pos

def get_xy(msg):
    global X,Y
    X = msg.x_m
    Y = msg.y_m
    
def get_Psi(msg):
    global Psi
    Psi = msg.z

def get_target(msg):
    global targetX, targetY
    targetX = msg.x
    targetY = msg.y

def mpc_controller(Psi,X,Y,targetX,targetY,V):
    max_steer = np.pi/6
    Rate = 10
    dt = 1/Rate
    R = np.array([cos(Psi), -sin(Psi)],[sin(Psi),cos(Psi)])
    Cost_min = float('inf')
    delta_opt = 0
    deltas = np.linspace(-max_steer,max_steer,21)
    for delta in deltas:
        dx = V*dt 
        dy = 2*V*delta*dt
        dp = np.array([dx],[dy])
        drot = np.mutmul(R,dp)
        Xfin = X + drot[0]
        Yfin = X + drot[1]

        Cost = ((targetX-Xfin)**2 + (targetY - Yfin)**2)**.5
        
        if Cost < Cost_min:
            Cost_min = Cost
            delta_opt = delta
    return delta_opt


def mpc_node():
    global X,Y,Psi
    global targetX, targetY

    targetX, targetY, X, Y, Psi = None, None, None, None, None

    rospy.init_node('mpc-controller')
    rospy.Subscriber('hedge_pos',hedge_pos,get_xy,queue_size=10)
    rospy.Subscriber('/euler_angles',Vector3,get_Psi,queue_size=10)
    rospy.Subscriber('target_position',Vector3,get_target, queue_size=10)
    pub = rospy.Publisher('ecu_pwm',ECU,queue_size=10)
    rate = rospy.Rate(10)
    while X is None or Y is None or Psi is None or targetY is None: continue

    max_steer = np.pi/6
    K = 400/max_steer
    V = 1

    while not rospy.is_shutdown():# and r.successful():
        delta = mpc_controller(Psi,X,Y,targetX,targetY,V)
        msg = ECU()
        msg.motor = 1580
        msg.servo = 1500 + K*delta
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        mpc_node()
    except rospy.ROSInterruptException:
        pass
