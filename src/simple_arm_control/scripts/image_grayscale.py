#!/usr/bin/env python3

import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GrayscalePub:

  def __init__(self):
    self.pub1 = rospy.Publisher('rgb_camera/image_raw/grayscale', Image, queue_size=10)
    self.pub2 = rospy.Publisher('rgb_camera/image_raw/circle', Image, queue_size=10)
    self.sub = rospy.Subscriber('rgb_camera/image_raw', Image, self.image_callback)
    self.bridge = CvBridge()

  def image_callback(self, img):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    except CvBridgeError as e:
      rospy.logerr(e)

    # Get and print image size
    rows, cols, channels  = cv_img.shape
    rospy.loginfo(f"Image rows: {rows}, cols: {cols}, channels: {channels}")

    # Convert to grayscale
    gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    # Draw a circle on color image
    if cols > 60 and rows > 60:
      cv2.circle(cv_img, (int(cols/2), int(rows/2)), 50, (0, 0, 255), 5)

  # Convert cv2_img to msg_img
    msg_gray_img = self.bridge.cv2_to_imgmsg(gray_img, encoding='passthrough')
    msg_circle_img = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')

    try:
      self.pub1.publish(msg_gray_img)
      self.pub2.publish(msg_circle_img)
    except CvBridgeError as e:
      rospy.logerr(e)


if __name__ == '__main__':
  # Initialize node
  rospy.init_node('image_converter')
  # Create instance for conveting image
  GrayscalePub()
  # Handle ROS subcribers
  rospy.spin()
