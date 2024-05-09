#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import ctypes
import sys

# Initialize Xlib support for concurrent threads (for Linux)
if sys.platform.startswith('linux'):
    X11 = ctypes.cdll.LoadLibrary('libX11.so')
    X11.XInitThreads()


def callback_color(image_data):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        cv2.imshow("Color Image", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)

def callback_depth(image_data):
    try:
        cv_image = bridge.imgmsg_to_cv2(image_data, "passthrough")
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.15), cv2.COLORMAP_HSV)
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(3)
    except CvBridgeError as e:
        print(e)

def listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback_color)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_depth)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    bridge = CvBridge()
    listener()