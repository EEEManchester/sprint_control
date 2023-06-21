#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

# global variable to store the latest CameraInfo message
camera_info_msg = None

def image_callback(data):
    global camera_info_msg
    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    try:
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    except CvBridgeError as e:
        print(e)

    # if a CameraInfo message has been received, copy its header to the image message
    if camera_info_msg is not None:
        image_message.header = camera_info_msg.header

    image_pub.publish(image_message)

def info_callback(data):
    global camera_info_msg
    camera_info_msg = data
    info_pub.publish(data)

def main():
    global image_pub, info_pub, camera_info_msg

    rospy.init_node('image_decompressor', anonymous=True)
    
    # Compressed image subscriber
    rospy.Subscriber("/bluerov/usb_cam/image_raw/compressed", CompressedImage, image_callback)
    image_pub = rospy.Publisher("usb_cam/image_raw/decompressed", Image, queue_size=10)

    # Camera info subscriber
    rospy.Subscriber("/bluerov/usb_cam/camera_info", CameraInfo, info_callback)
    info_pub = rospy.Publisher("usb_cam/image_raw/camera_info", CameraInfo, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
