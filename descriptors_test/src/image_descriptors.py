#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_handler:

  # Constructor
  def __init__(self):
    self.pub_keypoints = rospy.Publisher("image/keypoints", Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

  # Callback
  def callback(self, data):

    # recibimos una nueva imagen como 'data' y la convertimos en imagen de opencv
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # obtener dimensiones de la imagen
    (rows, cols, channels) = cv_image.shape

    # pasar a escala de grises
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #-------------------------------------------------------
    # Feature Detector and Description

    detector = cv2.xfeatures2d.SIFT_create()
    #detector = cv2.xfeatures2d.SURF_create()
    #detector = cv2.KAZE_create()
    #detector = cv2.AKAZE_create()
    #detector = cv2.BRISK_create()
    #detector = cv2.ORB_create()

    (kp, desc) = detector.detectAndCompute(gray, None)
    img_kp = cv2.drawKeypoints(gray, kp, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


    # re-publicamos la imagen al topico /image_republished
    try:
      self.pub_keypoints.publish(self.bridge.cv2_to_imgmsg(img_kp, "bgr8"))
    except CvBridgeError as e:
      print('Keypoints')
      print(e)


def main(args):
  # creamos instancia de clase image converter
  ih = image_handler()

  # iniciamos el nodo con su nombre
  rospy.init_node('image_descriptors')

  # spin esperara por los nuevos mensajes
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
