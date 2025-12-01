Proyecto Robot Car – Integración con ROS2

Este proyecto implementa un nodo ROS2 que se comunica con un carro robótico controlado por un ESP8266 mediante TCP/IP.
El microcontrolador controla los motores y el sensor ultrasónico HC-SR04, mientras que ROS2 envía comandos y recibe la distancia medida en tiempo real.

Video del proyecto:
[[ENLACE DEL VIDEO AQUÍ]](https://www.youtube.com/shorts/t6x-Yg9WhzE)
```txt
src/
└── robot_car_ros/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── robot_car_ros
    └── robot_car_ros/
        ├── __init__.py
        └── car_bridge_node.py
````
Requisitos previos

ROS2 Humble / Foxy / Jazzy instalado en Linux
Documentación oficial: https://docs.ros.org/

Python 3

Carro físico (no incluido en esta entrega):

ESP8266 (NodeMCU)

Firmware que recibe comandos F, B, L, R, S por TCP puerto 4210

Envío de distancia con formato:
```txt
DIST:<valor_en_cm>
```

El PC y el ESP8266 deben estar conectados a la misma red WiFi.


Instalación del paquete ROS2

Copiar la carpeta robot_car_ros dentro del workspace del usuario:
```txt
cd ~/ros2_ws/src
colocar aquí la carpeta robot_car_ros

```
Compilar:
```txt
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Ejecución del nodo ROS2

Para iniciar el nodo puente (TCP hacia ROS2):
```txt
ros2 run robot_car_ros car_bridge_node --ros-args -p car_ip:=<IP_DEL_CARRO>

```
Ejemplo:
```txt
ros2 run robot_car_ros car_bridge_node --ros-args -p car_ip:=192.168.4.1
```
Tópicos del proyecto
1. Enviar comandos al carro
```txt
ros2 topic pub /car_cmd std_msgs/msg/String "data: 'F'" -1
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

Comandos válidos:
```txt
F → Adelante
B → Atrás
L → Izquierda
R → Derecha
S → Stop
```
2. Leer la distancia del sensor ultrasónico
ros2 topic echo /car_distance


Ejemplo de salida:
```txt
data: 23.4
data: 21.8
data: 25.0
```
Autor

Nicholas Ruiz Zhilkin
Correo: nichoruizhilkin@gmail.com





