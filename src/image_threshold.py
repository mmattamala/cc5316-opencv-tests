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

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

    # definir umbrales
    self.thr_r_min = 100;
    self.thr_r_max = 200;

    self.thr_g_min = 100;
    self.thr_g_max = 200;

    self.thr_b_min = 100;
    self.thr_b_max = 200;


  # Callback
  def callback(self, data):

    # recibimos una nueva imagen como 'data' y la convertimos en imagen de opencv
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # obtener dimensiones de la imagen
    (rows, cols, channels) = cv_image.shape

    # ahora separaremos los canales
    img_b, img_g, img_r = cv2.split(cv_image)

    # aplicamos un umbral sobre cada canal
    ret_b, mask_b = cv2.threshold(img_b, self.thr_b_min, self.thr_b_max, cv2.THRESH_BINARY)
    ret_g, mask_g = cv2.threshold(img_g, self.thr_g_min, self.thr_g_max, cv2.THRESH_BINARY)
    ret_r, mask_r = cv2.threshold(img_r, self.thr_r_min, self.thr_r_max, cv2.THRESH_BINARY)

    # aplicamos las mascaras
    img_thr_b = cv2.bitwise_and(cv_image,cv_image, mask = mask_b)
    img_thr_bg = cv2.bitwise_and(img_thr_b,img_thr_b, mask = mask_g)
    img_thr_bgr = cv2.bitwise_and(img_thr_bg,img_thr_bg, mask = mask_r)

    #rospy.loginfo(new_img.shape)

    # re-publicamos la imagen
    try:
      self.img_thr_pub.publish(self.bridge.cv2_to_imgmsg(img_thr_bgr, "bgr8"))
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
