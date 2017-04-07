#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_handler:

  # Constructor
  def __init__(self):
    self.image_pub = rospy.Publisher("image_republished",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

  # Callback
  def callback(self, data):

    # recibimos una nueva imagen como 'data' y la convertimos en imagen de opencv
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # re-publicamos la imagen al tópico /image_republished
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
