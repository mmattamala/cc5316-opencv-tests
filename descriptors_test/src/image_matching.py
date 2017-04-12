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

MIN_MATCH_COUNT = 10

class image_handler:

  # Constructor
  def __init__(self):
    self.pub_det = rospy.Publisher("image/detection", Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image_raw",Image, self.callback)

    self.curr_kp   = None
    self.curr_desc = None
    self.train_kp   = None
    self.train_desc = None
    self.detect = False
    self.img_train = None

    self.referencePoints = []
    self.cropping = False


  # Callback
  def callback(self, data):

    # recibimos una nueva imagen como 'data' y la convertimos en imagen de opencv
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # obtener dimensiones de la imagen
    (rows, cols, channels) = self.cv_image.shape

    # pasar a escala de grises
    gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

    #-------------------------------------------------------
    # Feature Detector and Description

    self.detector = cv2.xfeatures2d.SIFT_create()
    #self.detector = cv2.xfeatures2d.SURF_create()
    #self.detector = cv2.KAZE_create()
    #self.detector = cv2.AKAZE_create()
    #self.detector = cv2.BRISK_create()
    #self.detector = cv2.ORB_create()

    (self.curr_kp, self.curr_desc) = self.detector.detectAndCompute(gray, None)

    img_kp = cv2.drawKeypoints(self.cv_image, self.curr_kp, None, color=(0,0,255))

    cv2.imshow('image', img_kp)
    cv2.setMouseCallback('image', self.clipAndCrop)

    # eventos de teclado
    k = cv2.waitKey(30)

    #-------------------------------------------------------
    # Image matching

    if self.detect:
      # creamos objeto que realiza matching de fuerza bruta (Brute Force Matching, BFMatcher)
      # OJO: cuidado con el tipo de descriptor (binario o punto flotante)
      #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
      bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

      # Matching de descriptores
      matches = bf.match(self.train_desc, self.curr_desc)

      # Los ordenamos en funcion de su distancia
      matches = sorted(matches, key = lambda x:x.distance)
      N = len(matches)
      M = int(math.floor(0.6 * N))    # seleccionaremos el 60% de los puntos con menor distancia

      if M>MIN_MATCH_COUNT:
        train_pts = np.float32([self.train_kp[m.queryIdx].pt for m in matches[:M] ]).reshape(-1,1,2)
        curr_pts = np.float32([self.curr_kp[m.trainIdx].pt for m in matches[:M] ]).reshape(-1,1,2)

        H, mask = cv2.findHomography(train_pts, curr_pts, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        h,w,d = self.train_img.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts, H)
        img_det = cv2.polylines(self.cv_image, [np.int32(dst)], True, (255,255,255), 3, cv2.LINE_AA)
      else:
        print "Not enough matches are found - %d/%d" % (M,MIN_MATCH_COUNT)
        matchesMask = None


      # parametros de dibujo
      draw_params = dict(matchColor = (0,255,0), # dibujar matches en verde
                     singlePointColor = None,
                     matchesMask = matchesMask, # dibujar solo inliers
                     flags = 2)
      img_det2 = cv2.drawMatches(self.train_img, self.train_kp, img_det, self.curr_kp, matches[:M], None, **draw_params)


      # publicamos la imagen al topico /image/detection
      try:
        self.pub_det.publish(self.bridge.cv2_to_imgmsg(img_det2, "bgr8"))
      except CvBridgeError as e:
        print('Homography')
        print(e)

  def clipAndCrop(self, event, x, y, flags, param):
    #referencia: http://www.pyimagesearch.com/2015/03/09/capturing-mouse-click-events-with-python-and-opencv/
    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
      self.referencePoints = [(x, y)]
      self.cropping = True
     # check to see if the left mouse button was released
    elif (event == cv2.EVENT_LBUTTONUP and self.cropping):
      # record the ending (x, y) coordinates and indicate that
      # the cropping operation is finished

      self.referencePoints.append((x, y))
      cropping = False

      img_clone = self.cv_image.copy()

      # draw a rectangle around the region of interest
      cv2.rectangle(img_clone, self.referencePoints[0], self.referencePoints[1], (0, 255, 0), 2)

      # calcular descriptores en imagen de entrenamiento
      self.train_img = img_clone[self.referencePoints[0][1]:self.referencePoints[1][1], self.referencePoints[0][0]:self.referencePoints[1][0]]
      train_gray = cv2.cvtColor(self.train_img, cv2.COLOR_BGR2GRAY)

      (self.train_kp, self.train_desc) = self.detector.detectAndCompute(train_gray, None)
      self.detect = True
      print('Imagen de entrenamiento capturada!')
      cv2.imshow('image', img_clone)


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

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
