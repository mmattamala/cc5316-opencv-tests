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
    self.img_thr_pub = rospy.Publisher("image/threshold",Image)
    self.img_h_pub = rospy.Publisher("image/h_channel",Image)
    self.img_s_pub = rospy.Publisher("image/s_channel",Image)
    self.img_v_pub = rospy.Publisher("image/v_channel",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

    # definir umbrales
    self.thr_h_min = 100;
    self.thr_h_max = 120;

    self.thr_s_min = 10;
    self.thr_s_max = 150;

    self.thr_v_min = 0;
    self.thr_v_max = 150;


  # Callback
  def callback(self, data):

    # recibimos una nueva imagen como 'data' y la convertimos en imagen de opencv
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # obtener dimensiones de la imagen
    (rows, cols, channels) = cv_image.shape

    # Cambio de espacio de colores
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # ahora separaremos los canales
    img_h, img_s, img_v = cv2.split(hsv)

    # aplicamos un umbral sobre cada canal
    ret_h, mask_h = cv2.threshold(img_h, self.thr_h_min, self.thr_h_max, cv2.THRESH_BINARY)
    ret_s, mask_s = cv2.threshold(img_s, self.thr_s_min, self.thr_s_max, cv2.THRESH_BINARY)
    ret_v, mask_v = cv2.threshold(img_v, self.thr_v_min, self.thr_v_max, cv2.THRESH_BINARY)

    # aplicamos las mascaras
    img_thr_h   = cv2.bitwise_and(hsv, hsv, mask = mask_h)
    img_thr_hs  = cv2.bitwise_and(img_thr_h, img_thr_h, mask = mask_s)
    img_thr_hsv = cv2.bitwise_and(img_thr_hs, img_thr_hs, mask = mask_v)

    rgb = cv2.cvtColor(img_thr_hsv, cv2.COLOR_HSV2BGR)

    #rospy.loginfo(new_img.shape)

    # re-publicamos la imagen
    try:
      self.img_thr_pub.publish(self.bridge.cv2_to_imgmsg(rgb, "bgr8"))
      self.img_h_pub.publish(self.bridge.cv2_to_imgmsg(img_h, "mono8"))
      self.img_s_pub.publish(self.bridge.cv2_to_imgmsg(img_s, "mono8"))
      self.img_v_pub.publish(self.bridge.cv2_to_imgmsg(img_v, "mono8"))

    except CvBridgeError as e:
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
