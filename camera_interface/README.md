# camera_interface

`camera_interface` er en ROS 2-pakke som håndterer oppstart og publisering av bilder fra et tilkoblet kamera. Den inneholder en enkel driver-node som publiserer bilder på et ROS-topic, samt en tilhørende launch-fil for å starte systemet.

## Innhold

- `camera_driver_node.py`: Python-node som bruker OpenCV til å hente bilder fra et kamera og publiserer dem som ROS-meldinger av typen `sensor_msgs/Image`.
- `launch/camera_driver.launch.py`: Launch-fil som starter `camera_driver_node` med mulighet for å sette parametere som kameraindeks og FPS.

## ROS Topics

- Publiserer:
  - `/image_raw` (`sensor_msgs/Image`): Råbilder fra kamera.

## Parametere

- `camera_index` (int): Indeks for kameraenheten (f.eks. 0 for intern webkamera).
- `fps` (float): Bildefrekvens i bilder per sekund.

## Kjøring

For å starte kamera-driveren:

```bash
ros2 launch camera_interface camera_driver.launch.py
