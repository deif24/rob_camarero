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
  
- Turtlebot3:
  ```bash
  sudo apt install ros-noetic-dynamixel-sdk
  sudo apt install ros-noetic-turtlebot3-msgs
  sudo apt install ros-noetic-turtlebot3
  ```
  
- PyAudio
  ```bash
  sudo apt install portaudio19-dev
  sudo apt install python3-all-dev
  sudo apt install python3-pyaudio
  ```
  
- SMACH:<br>
  ```bash
  sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer
  ```
  
- VOSK:<br>
  [Link a su web oficial](https://alphacephei.com/vosk/).
<p align="justify">
VOSK es una biblioteca de reconocimiento de voz offline así que para su correcto funcionamiento hay que bajarse el modelo correspondiente al idioma utilizado que se puede encontrar en la web oficial.
</p>
  
- Mediapipe:<br>
  [Link a su repositorio](https://github.com/google-ai-edge/mediapipe).

# Guia de instalación
El primer paso para poder utilizar el paquete es crear un workspace de ros donde poder instalarlo
En caso de no tener uno se puede crear asi:
```bash
mkdir robot_camarero_ws
cd robot_camarero_ws
mkdir src
cd src
```
Dentro de la carpeta src de tu workspace tendras que clonar este repositorio:
```bash
git clone https://github.com/deif24/rob_camarero.git
```
Una vez descargado el paquete hay que volver al directorio base para compilarlo y actualizar las variables de entorno:
```bash
cd ~/robot_camarero_ws
catkin_make
source devel/setup.bash
```
Si la compilación no ha dado ningún error el paquete estaría instalado correctamente y preparado para su uso.

# Guia de uso
El primer paso es ejecutar el entorno simulado junto con el turtlebot 3, para ello se hará uso del siguiente archivo launch.
```bash
roslaunch rob_camarero turtlebot3_restaurante.launch
```
En una terminal nueva hay que ejecutar el stack de navegación junto con el mapa del restaurante.
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<ruta_al_ws>/src/rob_camarero/mapa_restaurante.yaml
```
Antes de ejecutar el programa principal hay que modificar la ruta del modeleo de VOSK del archivo voz.py a donde se haya descargado
```bash
cd ~/robot_camarero_ws/src/rob_camarero/simulado
nano voz.py
MODEL_PATH = "<ruta_al_modelo>/vosk-model-es-0.42" # Modificar esta línea de código
```
Una vez se tenga configurado correctamente el reconocimiento de voz, se puede ejecutar el programa principal.
```bash
roslaunch rob_camarero rob_camarero.launch
```
<p align="justify">
Una vez ejecutados todos los comandos anteriores se debería de tener abierto una ventana de Gazebo, de RViz así como la interfaz del programa y la cámara que estará realizando el reconocimiento de gestos. El programa principial debería de estar ejecutándose en el estado de reposo donde el turtlebot se encuentra girando sobre sí mismo esperando a encontrar una mano levantada o a que se pulse alguno de los botones de la interfaz referentes a las mesas.
</p>
