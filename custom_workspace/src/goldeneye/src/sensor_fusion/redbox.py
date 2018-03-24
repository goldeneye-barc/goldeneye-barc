#!/usr/bin/env python
import rospy
import getch
from barc.msg import ECU
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


def image_process(image, params):
    bridge = CvBridge()
    global error
    try:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        lower = np.array([110,50,50], dtype="uint8")
        upper = np.array([130,255,255], dtype="uint8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(cv_image, lower, upper)
        #mask = cv2.erode(mask, None, iterations=2)
        #mask = cv2.dilate(mask, None, iterations=2)
        output = cv2.bitwise_and(cv_image,cv_image, mask=mask)
        #output = mask
        #cnts = cv2.findCountours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        #raw_data = np.fromstring(image.data, np.uint8)
        #cv_image = cv2.imdecode(raw_data, cv2.IMREAD_COLOR)
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)
        #print(reds.shape, cv_image.shape, len(cv_image))
        #cv2.imshow('wooho', reds)
        #reds = cv_image[:, :, 2].copy()
        #print(reds.shape)
        #_, reds = cv2.threshold(reds, 250, 255, cv2.THRESH_BINARY)
        #print('here')
        cv2.imshow('raw', cv_image)
        cv2.imshow('mask', mask)
        cv2.imshow('output', output)
        cv2.waitKey(1)
        return

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

        
    except Exception as e:
        print(e)

def image_node():
    global coordinates
    coordinates = [1,2,3,4]    #[x1,y1,x2,y2]

    param_names = [] 
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    print(params)
    
    rospy.init_node('BoxDetector')
    #rospy.Subscriber('/image_raw/compressed', CompressedImage, image_process, params)
    rospy.Subscriber('/image_raw', Image, image_process, params)
    pub = rospy.Publisher('bounding_box', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = Int32MultiArray()
        msg.data = coordinates
        pub.publish(msg)
        rate.sleep()
    #out.release()


if __name__ == '__main__':
    try: 
        image_node()
    except rospy.ROSInterruptException:
        pass
