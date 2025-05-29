# ROS Mega Prosjekt


[![Package nodes diagram simulated on ROS and Rviz | Download Scientific ...](https://tse1.mm.bing.net/th?id=OIP.fs4oCKt3UvUHo7-Q3t0URQHaGJ\&cb=iwp1\&pid=Api)](https://www.researchgate.net/figure/Package-nodes-diagram-simulated-on-ROS-and-Rviz_fig6_316945804)


Dette prosjektet ble utviklet som del av AIS2105: Mekatronikk og Robotikk ved NTNU Ålesund. Systemet bruker ROS 2 og MoveIt for å styre en UR5-robotarm til å identifisere og peke på tre fargede kuber rekkefølgen rød, gul og blå.

## Funksjonalitet

Systemet gjør dette ved oppstart:

- Går til en definert hjem-posisjon
- Flytter seg for å ta oversiktsbilde over bordet
- Detekterer røde, gule og blå kuber via bildebehandling
- Estimerer posisjoner i 3D ved hjelp av tf2
- Planlegger og utfører bevegelser til hver kube i rekkefølgen: rød → gul → blå
- Varsler hvis noen av kubene ikke blir funnet

## Pakker

### `camera_interface`

Inneholder:

- `camera_driver_node`: Leser bilder fra kamera og publiserer dem på `/camera/image_raw`
- `camera_driver.launch.py`: Launch fil for å starte kamera

### `cube_detection`
  
  Inneholder:

- `cube_detector_node`: Detekterer kuber via HSV-fargefiltrering og publiserer senterpunkt og farge
- `pose_estimator_node`: Konverterer 2D-punkt til 3D-posisjon via transformasjon med `tf2`
- Estimerte posisjoner publiseres som `PoseStamped` på `/cube_pose`

### `ur_motion_planning`

  Inneholder:

- `planner_node.cpp`: Tar imot posisjoner fra `/cube_pose` og bruker `MoveGroupInterface` fra MoveIt til å planlegge bevegelser

### `system_integration`

  Inneholder:
  
- `system.launch.py`: Starter hele systemet 

## Avhengigheter

- ROS 2 (Jazzy)
- MoveIt
- OpenCV
- tf2
- Third_Party UR ROS driver (`ur_robot_driver`)

## Hvordan installere third_party repos

1. cd inn til SRC folder
2. Skriv følgende inn i terminalen:
```
vcs import < ../external.repos
```
3. Det vil installere UR config files som trengs for å bruke roboten
4. deretter kjør følgende i mega_ws/ (ikke i src)
```
rosdep install --from-paths src --ignore-src -r -y
```


## Kjøring

### Start hele systemet

```bash
ros2 launch system_integration system.launch.py
```

#### Individuell testing av pakker

Start Kamera:

```bash
ros2 launch camera_interface camera_driver.launch.py
```

Start Kube-deteksjon:

```bash
ros2 run cube_detection cube_detector_node
ros2 run cube_detection pose_estimator_node
```

Start Plnalegger:

```bash
ros2 run ur_motion_planning planner_node
```
