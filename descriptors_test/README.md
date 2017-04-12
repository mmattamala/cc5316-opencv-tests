# Descriptors Tests
Este repositorio incluye ejemplos de calculo de descriptores con ROS

## Requerimientos
Se asume que hay un nodo que publica imágenes al tópico `/camera/image_raw`. La forma más fácil de hacerlo es usando el nodo `usb_cam` para publicar imágenes desde una cámara usb y _remappeando_ el tópico al lanzar el nodo:

    rosrun usb_cam usb_cam_node /usb_cam/image_raw:=/camera/image_raw

# Nodos incluidos

- `image_descriptors.py`: se subscribe a la imagen y calcula descriptores binarios y de punto flotante
- `image_tracks.py`: se subscribe a la imagen y calcula _tracks_ entre la imagen anterior y la actual
- `image_matching.py`: se subscribe a la imagen y permite guardar una imagen de entrenamiento con el mouse. Luego se calcula una homografía con el objeto detectado en la imagen actual.


