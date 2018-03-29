#!/usr/bin/env python
import rospy
import numpy as np
#import roslaunch
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Float32
from barc.msg import ECU
from numpy.linalg import lstsq
from numpy import array, vstack, ones
from math import atan, atan2, pi

servo_pwm = 1520
motor_pwm = 1500

def get_xy(msg):
    global x, y, record_on, tick, grabbed_xy
    x = msg.x_m
    y = msg.y_m
    if record_on:
        grabbed_xy.append((x,y))

def head_angle_action():
    global x, y, record_on, tick, grabbed_xy
    rospy.init_node('get_heading_angle')
 #   rospy.on_shutdown(self.shutdown)
    ecu_pub = rospy.Publisher('ecu_pwm',ECU,queue_size=10)
    rospy.Subscriber('hedge_pos', hedge_pos, get_xy, queue_size=10)

    
    rate = rospy.Rate(10)
    t0 = rospy.get_time()
    grabbed_xy = []
    record_on = 1
    tick = 1
    while not rospy.is_shutdown():
        del_t = rospy.get_time() - t0
        if del_t <= 2:
            motor_pwm = 1580
        elif del_t <= 4:
            motor_pwm = 1420
        elif del_t <= 6:
            motor_pwm = 1580
        elif del_t <= 8:
            motor_pwm = 1580
        else:
            record_on = 0
            motor_pwm = 1500
            break
        
        ecu_pub.publish(ECU(motor_pwm,servo_pwm))
    ecu_pub.publish(ECU(motor_pwm,servo_pwm))
    [xvals, yvals] = [v[0] for v in grabbed_xy],[v[1] for v in grabbed_xy]
    A = np.vstack([xvals, np.ones(len(xvals))]).T
    b = np.array(yvals).T
    lstsq_out = np.linalg.lstsq(A,b)
    theta0 = atan2(lstsq_out[0][0],1)

    theta_est = sum([atan2(v[1] - yvals[0],v[0] - xvals[0]) for v in grabbed_xy[2:20]])/len(grabbed_xy[2:20])
    if not (theta_est <= pi/2 and theta_est >= -pi/2):
        theta0 = theta0 + np.pi
    rospy.set_param('heading_angle',theta0)
    rospy.set_param('heading_angle_flag',False)

#    uuid = roslaunch.rlutil.get_or_generate_uuid(None,False)
#    roslaunch.configure_logging(uuid)
#    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/barc/custtom_workspace/src/goldeneye/launch/point_to_point.launch"])
#    launch.start()
#    rospy.set_param('heading_angle',theta0)

if __name__ == '__main__':
    try:
        head_angle_action()
    except rospy.ROSInterruptException:
        pass
