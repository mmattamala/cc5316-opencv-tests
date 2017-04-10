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
    self.img_filtered = rospy.Publisher("image/filtered",Image)

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

    #kernel = 1.0/9 * np.array([[1,1,1],
    #                   [1,1,1],
    #                   [1,1,1]])

    #kernel = 1.0 / 36 * np.ones([6,6])

    #kernel = np.array([[1,0,-1],
    #                   [1,0,-1],
    #                   [1,0,-1]])
    #kernel = np.array([[-1,-1,-1],
    #                   [0,0,0],
    #                   [1,1,1]])

    kernel = np.array([[0,1,0],
                       [1,-4,1],
                       [0,1,0]])

    # filtrar imagen
    filtered = cv2.filter2D(cv_image, -1, kernel)

    # re-publicamos la imagen al topico /image_republished
    try:
      self.img_filtered.publish(self.bridge.cv2_to_imgmsg(filtered, "bgr8"))

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
