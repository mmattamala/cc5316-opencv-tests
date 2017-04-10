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
    self.pub_harris = rospy.Publisher("image/corners_harris",Image)
    self.pub_st = rospy.Publisher("image/corners_shitomasi",Image)
    self.pub_fast = rospy.Publisher("image/corners_fast",Image)

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

    # casteo a float
    gray = np.float32(gray)

    # ------------------------------------------------------
    # Harris
    # blockSize - It is the size of neighbourhood considered for corner detection
    # ksize - Aperture parameter of Sobel derivative used (kernel size)
    # k - Harris detector free parameter in the equation.
    img_harris = np.copy(cv_image)

    corners_harris = cv2.cornerHarris(gray, 2, 3, 0.004)
    corners_harris = cv2.dilate(corners_harris, None)
    img_harris[corners_harris>0.01*corners_harris.max()]=[0,0,255]

    img_harris = np.uint8(img_harris)

    # ------------------------------------------------------
    # Shi-Tomasi
    corners = cv2.goodFeaturesToTrack(gray,25,0.01,10)
    corners = np.int0(corners)
    img_st = np.copy(cv_image)
    for i in corners:
        x,y = i.ravel()
        cv2.circle(img_st,(x,y),3,(0,255,0),-1)
    img_st = np.uint8(img_st)

    # ------------------------------------------------------
    # FAST
    fast = cv2.FastFeatureDetector_create()
    fast.setThreshold(10)
    kp = fast.detect(cv_image, None)
    dummy = np.zeros((1,1))
    img_fast = cv2.drawKeypoints(cv_image, kp, dummy, color=(255,0,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    img_fast = np.uint8(img_fast)

    # ------------------------------------------------------


    # re-publicamos la imagen al topico /image_republished
    try:
      self.pub_harris.publish(self.bridge.cv2_to_imgmsg(img_harris, "bgr8"))
    except CvBridgeError as e:
      print('Harris')
      print(e)

    try:
      self.pub_st.publish(self.bridge.cv2_to_imgmsg(img_st, "bgr8"))
    except CvBridgeError as e:
      print('Shi-Tomasi')
      print(e)

    try:
      self.pub_fast.publish(self.bridge.cv2_to_imgmsg(img_fast, "bgr8"))
    except CvBridgeError as e:
      print('Fast')
      print(e)




def main(args):
  # creamos instancia de clase image converter
  ih = image_handler()

  # iniciamos el nodo con su nombre
  rospy.init_node('image_republisher')

  # spin esperara por los nuevos mensajes
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
