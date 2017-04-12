#!/usr/bin/env python
import roslib

import sys
import rospy
import cv2
import numpy as np
import math

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_handler:

  # Constructor
  def __init__(self):
    self.pub_tracks = rospy.Publisher("image/tracks", Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

    self.curr_kp   = None
    self.curr_desc = None
    self.last_kp   = None
    self.last_desc = None


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

    (self.curr_kp, self.curr_desc) = detector.detectAndCompute(gray, None)

    if self.last_kp == None and self.last_desc==None:
        self.last_kp = self.curr_kp
        self.last_desc = self.curr_desc



    #-------------------------------------------------------
    # Image matching

    # creamos objeto que realiza matching de fuerza bruta (Brute Force Matching, BFMatcher)
    # OJO: cuidado con el tipo de descriptor (binario o punto flotante)
    #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    # Matching de descriptores
    matches = bf.match(self.curr_desc, self.last_desc)

    # Los ordenamos en funcion de su distancia
    matches = sorted(matches, key = lambda x:x.distance)
    N = len(matches)
    M = int(math.floor(0.6 * N))    # seleccionaremos el 60% de los puntos con menor distancia

    # Dbujamos los tracks del frame anterior al actual
    img_kp = cv2.drawKeypoints(cv_image, self.curr_kp, None, color=(0,255,0))
    img_kp = cv2.drawKeypoints(img_kp,   self.last_kp, None, color=(0,125,0))

    for m in matches[:M]:
        p1 = tuple(np.round(self.curr_kp[m.queryIdx].pt).astype(int))
        p2 = tuple(np.round(self.last_kp[m.trainIdx].pt).astype(int))

        # cv2.line(imagen, punto1, punto 2, color, grosor de linea)
        cv2.line(img_kp, p1, p2, (0,255,0), 1)

    #-------------------------------------------------------
    # Guardar keypoints y descriptores
    self.last_kp = self.curr_kp
    self.last_desc = self.curr_desc


    # publicamos la imagen al topico /image_republished
    try:
      self.pub_tracks.publish(self.bridge.cv2_to_imgmsg(img_kp, "bgr8"))
    except CvBridgeError as e:
      print('Tracks')
      print(e)


def main(args):
  # creamos instancia de clase image converter
  ih = image_handler()

  # iniciamos el nodo con su nombre
  rospy.init_node('image_tracks')

  # spin esperara por los nuevos mensajes
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
