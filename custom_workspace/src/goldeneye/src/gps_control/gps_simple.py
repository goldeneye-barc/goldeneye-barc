#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import sys
import cv2
import numpy as np
#from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


def image_process(image, params):
    #bridge = CvBridge()
    lower = params['lowerY']
    upper = params['upperY']
    debug_info = params['debug_info']
    global error
    try:
        #cv_image = bridge.imgmsg_to_cv2(image)
        raw_data = np.fromstring(image.data, np.uint8)
        cv_image = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        gray = cv2.imdecode(raw_data, cv2.IMREAD_GRAYSCALE)
        _, gray = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
        y_len, x_len = gray.shape
        lower, upper = max(0, lower), min(upper, y_len)
        if debug_info: print(lower, upper, 'image crop in Y')
        gray = gray[lower:upper, :]
        y_len, x_len = gray.shape
        total_error = []
        for y in range(10, y_len, 90):
            weighted_mass = 0
            mass = 0
            for x in range(0, x_len):
                if gray[y,x] == 255:
                    mass+=1
                    weighted_mass += x
            if mass > 0:
                final_x = int(weighted_mass / mass)
                total_error.append(float(final_x - x_len // 2) / float(x_len // 2))
                if params['display_processed_image']: cv2.rectangle(gray, (final_x-10,y), (final_x + 10, y+35), 120, 2)
        #cv2.imshow('wooho', gray)

        if debug_info: print(total_error)
        if len(total_error) > 2:
            error = sum(total_error) / len(total_error)
    
        if params['display_image']: cv2.imshow('raw', cv_image)
        if params['display_processed_image']: cv2.imshow('processed', gray)
        if params['publish_processed_image']:
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', gray)[1]).tostring()
            params['img_pub'].publish(msg)

        cv2.waitKey(1)
        
    except Exception as e:
        if debug_info:
            print(e)

def image_node():
    global error
    error = 0

    param_names = ['lowerY', 'upperY', 'display_image', 'display_processed_image', 'debug_info', 'publish_processed_image']
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    print(params)

    rospy.init_node('Test_Imager')
    img_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
    params['img_pub'] = img_pub
    rospy.Subscriber('/image_raw/compressed', CompressedImage, image_process, params)
    pub = rospy.Publisher('ecu_pwm', ECU, queue_size=10)
    rate = rospy.Rate(10)
    
    K = 400
    Ki = 20
    integral = 0
    while not rospy.is_shutdown():
        msg = ECU()

        if params['debug_info']:
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
