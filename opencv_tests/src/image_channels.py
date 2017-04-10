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
    self.img_r_pub = rospy.Publisher("image/r_channel",Image)
    self.img_g_pub = rospy.Publisher("image/g_channel",Image)
    self.img_b_pub = rospy.Publisher("image/b_channel",Image)

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

    # ahora separaremos los canales
    img_b, img_g, img_r = cv2.split(cv_image)

    # preparamos nuevas imagenes RGB para ver los canales adecuadamente
    image_b = np.zeros([rows, cols, channels], np.uint8)
    image_b[:,:,0] = img_b

    image_g = np.zeros([rows, cols, channels], np.uint8)
    image_g[:,:,1] = img_g

    image_r = np.zeros([rows, cols, channels], np.uint8)
    image_r[:,:,2] = img_r

    # re-publicamos la imagen al topico /image_republished
    try:
      self.img_b_pub.publish(self.bridge.cv2_to_imgmsg(image_b, "bgr8"))
      self.img_g_pub.publish(self.bridge.cv2_to_imgmsg(image_g, "bgr8"))
      self.img_r_pub.publish(self.bridge.cv2_to_imgmsg(image_r, "bgr8"))
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
