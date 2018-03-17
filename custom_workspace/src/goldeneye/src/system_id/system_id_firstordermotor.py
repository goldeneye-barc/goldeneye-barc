#!/usr/bin/env python
import rospy
from barc.msg import ECU
from std_msgs.msg import Float32

def system_id():

    rospy.init_node('System_ID')
    #rospy.Subscriber()
    pubecu = rospy.Publisher('ecu_pwm', ECU, queue_size=10) 
    rate = rospy.Rate(10)

    t0 = rospy.get_time()

    while not rospy.is_shutdown():
        curtime = rospy.get_time() - t0
        msgecu = ECU()

        msgecu.servo = 1540
        if curtime <= 2:
            msgecu.motor = 1580
        elif curtime <= 5:
            msgecu.motor = 1620
        elif curtime <= 7:
            msgecu.motor = 1600
        else:
            msgecu.motor = 1500

        pubecu.publish(msgecu)
        rate.sleep()

if __name__ == '__main__':
    try:
        system_id()
    except rospy.ROSInterruptException:
        pass
