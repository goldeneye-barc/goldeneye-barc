#!/usr/bin/env python
import rospy
from marvelmind_nav.msg import hedge_pos
from std_msgs.msg import Float32

servo_pwm = 1520
motor_pwm = 1500

def get_xy():
	global x, y, record_on, tick
	x = msg.x_m
	y = msg.y_m
	if record_on and tick == 10:
		grabbed_xy.append((x,y))
		tick = 1
	elif record_on:
		tick += 1

def head_angle_action():
	global x, y, record_on, tick
	rospy.init_node('get_heading_angle')
	ecu_pub = Publsher('ecu_pwm',ECU,queue_size=10)

	
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
			motor_pwm = -1420
		elif del_t <= 5:
			motor_pwm = 1580
		else:
			motor_pwm = 1500
			break
		
		ecu_pub.Publish(ECU(motor_pwm,servo_pwm))


if __name__ == '__main__':
    try:
        head_angle_action()
    except rospy.ROSInterruptException:
        pass
