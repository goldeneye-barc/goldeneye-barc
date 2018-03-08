#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from bounding_box import frame_to_bounding_box


def image_process(image, params):
    bridge = CvBridge()
    global error
    global coordinates
    try:
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        output coordinates = frame_to_bounding_box(cv_image, 'bgr') #image, coordinates
        return

        if debug_info: print(total_error)
        if len(total_error) > 2:
            error = sum(total_error) / len(total_error)
    
        if params['display_image']: cv2.imshow('raw', cv_image)
        if params['display_processed_image']: cv2.imshow('processed', output)
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

    param_names = ['display_image', 'display_processed_image'] 
    params = {}
    for param in param_names:
        params[param] = rospy.get_param(param)
    print(params)
    
    rospy.init_node('BoxDetector')
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
