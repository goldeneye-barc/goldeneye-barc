#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from barc.msg import ECU
import numpy as np
from marvelmind_nav.msg import hedge_pos

wait_for_heading_angle = rospy.get_param('heading_angle_flag')
while wait_for_heading_angle:
	wait_for_heading_angle = rospy.get_param('heading_angle_flag')

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
    max_steer = np.pi/8
    Rate = 10.0
    dt = 1/Rate
    R = np.array([[np.cos(Psi), -np.sin(Psi)],[np.sin(Psi),np.cos(Psi)]])
    #print(Psi)
    #localCoords = R.dot(np.array([targetX -X , targetY -Y]))
    #localY = localCoords[1]
    #error = localY
    #return error
    Cost_min = float('inf')
    delta_opt = 0
    deltas = np.linspace(-max_steer,max_steer,21)
    for delta in deltas:
        dx = V*dt 
        dy = 2*V*delta*dt
        dp = np.array([dx,dy])
        drot = R.dot(dp) #np.mutmul(R,dp)
        #print('rot', drot, 'R', R, 'dp', dp)
        Xfin = X + drot[0]
        Yfin = Y + drot[1]

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
    pub = rospy.Publisher('/ecu_pwm',ECU,queue_size=10)
    rate = rospy.Rate(10)

    max_steer = np.pi/6
    K = 400/max_steer
    V = 1

    while not rospy.is_shutdown():# and r.successful():
        if X is None or Y is None or Psi is None or targetX is None or targetY is None: continue
        delta = mpc_controller(Psi,X,Y,targetX,targetY,V) 
        msg = ECU()
        msg.motor = 1580
        msg.servo = 1500 - delta*K #/ 10 * 800
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        mpc_node()
    except rospy.ROSInterruptException:
        pass
