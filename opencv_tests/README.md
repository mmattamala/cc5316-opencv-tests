# OpenCV Tests
Este repositorio incluye ejemplso básicos de uso de OpenCV con ROS

## Requerimientos 
Se asume que hay un nodo que publica imágenes al tópico `/camera/image_raw`. La forma más fácil de hacerlo es usando el nodo `usb_cam` para publicar imágenes desde una cámara usb y _remappeando_ el tópico al lanzar el nodo:
    
    rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw

# Nodos incluidos
El package incluye 6 nodos para realizar pruebas básicas de OpenCV.

- `image_republisher.py`: se subscribe a la imagen, la convierte a formato opencv, y la convierte de vuelta al formato de mensajes de ROS, publicandola en otro tópico.
- `image_channels.py`: Se subscribe a una imagen, la separa en canales RGB, y publica los tres canales por separado.
- `image_threshold.py`: Recibe una imagen, la separa en canales, y aplica un umbral (_threshold_) por canal. Posteriormente fusiona los canales de vuelta, reconstruye una imagen RGB _umbralizada_ y la publica.
- `image_threshold_with_hsv.py`: Lo mismo que el anterior, pero transforma la imagen de RGB al espacio HSV para aplicar los umbrales en ese espacio. Se publica el resultado de la _umbralización_
- `image_filters.py`: Aplica filtros 2D a la imagen. Hay ejemplos de deblurring con filtro cuadrado, Prewitt y Laplaciano de Gaussianas (Laplacian of Gaussians, LoG). Se publica el resultado de un sólo filtro.
- `image_corners.py`: Ejemplos de detección de esquinas con Harris, Shi-Tomasi (GoodFeaturesToTrack) y FAST. Se publican los resultados en tópicos distintos.




