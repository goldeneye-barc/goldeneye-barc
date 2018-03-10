#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import sys
import cv2
import numpy as np
import traceback
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from bounding_box import frame_to_bounding_box


def image_process(image, params):
    bridge = CvBridge()
    
    global error
    global coordinates
    try:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        cv_image = cv2.resize(cv_image, (0, 0), fx=0.6, fy=0.6)
        #raw_data = np.fromstring(image.data, np.uint8)
        #cv_image = cv2.imdecode(raw_data, cv2.IMREAD_UNCHANGED)
        #print(cv_image.shape)
        output, coordinates = frame_to_bounding_box(cv_image, 'bgr') #image, coordinates
        if params['display_image']: 
            cv2.imshow('raw', cv_image)
            cv2.waitKey(1)
        if params['debug_info']: print('DEBUG: ' + str(coordinates))
        if params['display_processed_image']: 
            cv2.imshow('processed', output)
            cv2.waitKey(1)
    except Exception as e:
        traceback.print_exc()
        print(e)

def image_node():
    global coordinates
    coordinates = [1,2,3,4]    #[x1,y1,x2,y2]

    param_names = ['display_image', 'display_processed_image', 'debug_info'] 
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    print(params)
    
    rospy.init_node('BoxDetector')
    rospy.Subscriber('/image_raw', Image, image_process, params)
    #rospy.Subscriber('/image_raw', Image, image_process, params)
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
