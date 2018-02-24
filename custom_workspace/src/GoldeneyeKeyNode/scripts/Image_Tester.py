#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

#out = cv2.VideoWriter('output3.avi', cv2.VideoWriter_fourcc(*'PIM1'), 20.0, ( 1288, 964), False)

def image_process(image):
    bridge = CvBridge()
    global error
    try:
        cv_image = bridge.imgmsg_to_cv2(image)
        #out.write(cv_image)
        gray = cv_image
        #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, gray = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
        gray = gray[200:600, :]
        y_len, x_len = gray.shape
        total_error = []
        for y in range(10, y_len,90):
            weighted_mass = 0
            mass = 0
            for x in range(0, x_len):
                if gray[y,x] == 255:
                    mass+=1
                    weighted_mass += x
            if mass > 0:
                final_x = int(weighted_mass / mass)
                total_error.append(float(final_x - x_len // 2) / float(x_len // 2))
                #cv2.rectangle(gray, (final_x-10,y), (final_x + 10, y+35), 120, 2)
        #cv2.imshow('wooho', gray)

        #print(total_error)
        if len(total_error) > 2:
            error = sum(total_error) / len(total_error)
        else:
            pass
            #error = 0


        #print(error)
    
        #cv2.imshow('full image', cv_image[0:midx+100,:])
        #cv2.imshow('bottom half',cv_bottom)
        cv2.waitKey(1)
        
        #cols = []
        #x,y = cv_image.shape
        ##print(x,y)
        ##print(type(cv_image))
        #img = np.array(cv_image)
        #midx = int(round(x/2))
        #midy = int(round(y/2))
        ##cv_bottom = cv_image[midx:,:]
        #
        #cols = np.zeros(y)
        #cv_image = cv2.Canny(cv_image, 250,255)
        #for i in range(0,midx+100,15):
        #    for j in range(0,y,1):
        #        if cv_image[i,j] == 255:
        #            cols[j] += 1

        #
        #npix = 0
        #tfrac = 0
        #for k in range(0,y-1):
        #    tfrac += k*cols[k]
        #    npix += cols[k]

        #if npix == 0:
        #    avgcol = midy
        #else:
        #    avgcol = tfrac/npix
        #print(avgcol)
        #print(midy)
    except CvBridgeError as e:
        print(e)

def image_node():
    global error
    error = 0

    rospy.init_node('Test_Imager')
    rospy.Subscriber('image_raw', Image, image_process)
    pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)
    rate = rospy.Rate(10)
    
    K = 400
    Ki = 20
    integral = 0
    while not rospy.is_shutdown():
        msg = ECU()
        print(error)
        msg.motor = 1590
        msg.servo = 1500 + K*error # Ki*integral
        pub.publish(msg)
        integral += (1/10)* error
        if integral > 20:
            integral = 20
        if integral < -20:
            integral = -20
        rate.sleep()
    #out.release()


if __name__ == '__main__':
    try: 
        image_node()
    except rospy.ROSInterruptException:
        pass
