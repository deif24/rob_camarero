# Proyecto rob_camarero 

Este proyecto ha sido desarrollado por:
- David Martínez García
- Alejandro Gea Belda
- Manuel Muélledes Mateos

<p align="justify">
Este paquete de ROS tiene como objetivo desarrollar una aplicación para el robot turtlebot que le permita simular el comportamiento de un camarero en un restaurante. Para cumplir los objetivos propuestos, se ha implementado reconocimiento de voz y reconocimiento visual, además de hacer uso del stack de navegación proporcionado por ROS.

Para la simulación del proyecto se ha utilizado Gazebo y se ha creado un mapa que simula el entorno de un restaurante.
</p>

![mapa_restaurante](https://github.com/user-attachments/assets/8e5e63cb-b373-4c24-aaaf-59e1ee866252)

Para este paquete se han utilizado las siguientes bibliotecas:
- Turtlebot3:
  ```bash
  sudo apt install ros-noetic-dynamixel-sdk
  sudo apt install ros-noetic-turtlebot3-msgs
  sudo apt install ros-noetic-turtlebot3
  ```

- VOSK:<br>
  [Link a su web oficial](https://alphacephei.com/vosk/).
<p align="justify">
VOSK es una biblioteca de reconocimiento de voz offline así que para su correcto funcionamiento hay que bajarse el modelo correspondiente al idioma utilizado que se puede encontrar en la [web oficial](https://alphacephei.com/vosk/models).
</p>
  
- Mediapipe:<br>
  [Link a su repositorio](https://github.com/hrnr/m-explore).
  ```bash 
  ```
  

- SMACH:<br>
  ```bash
  sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer
  ```
- ROS Noetic:
  ```bash
  sudo apt install ros-noetic-desktop-full
  sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
  ```

# Guia de instalación
Para poder utilizarlo primero es necesario crearse un workspace donde contener el escondite.<br>
En caso de no tener uno se puede crear asi:
```bash
mkdir escondite_ws
cd escondite_ws
mkdir src
cd src
```
Dentro de tu workspace tendras que clonar este repositorio dentro de la carpeta src:
```bash
git clone https://github.com/AdrianGG22/escondite.git
```
Una vez descargado vuelves al directorio base del workspace, compilas el paquete y actualizas las variables de entorno:
```bash
catkin_make
source devel/setup.bash
```
Con todo esto el paquete ya estaría instalado y listo para su uso.

# Guia de uso
Primero se necesita ejecutar el gazebo con el turtlebot3, en este paquete se pueden elegir distintos mapas:
```bash
roslaunch escondite mapa_1.launch
roslaunch escondite mapa_2.launch
roslaunch escondite mapa_3.launch
roslaunch escondite mapa_4.launch
roslaunch escondite mapa_empty.launch #Este último, no tiene obstáculos de colores para localizar
```
En una terminal nueva ejecutamos el mapeado del turtlebot3:

```bash
# Al ejecutar el siguiente comando se puede añadir en Rviz el display
# de "Image" y poner el topic "/image_raw" para tener la cámara del robot
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
De nuevo, en una nueva terminal se ejecuta la navegacion del turtlebot3:
```bash
roslaunch turtlebot3_navigation move_base.launch
```
Por último, en una nueva terminal accedemos a la carpeta src del paquet para ejecutar el codigo "escondite.py":
```bash
cd src/escondite/src
python3 escondite.py
```
Tras todas estas ejecuciones el programa deberia perdirte elegir entre la deteccion de personas "p" o de colores "c", una vez lo elijas el robot empazara a moverse.
En caso de que hayas elegido la deteccion de perosonas estate seguro de que tengas una cámara conectada al dispositivo.
