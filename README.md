# Proyecto Robot Car – Integración con ROS2
Este proyecto implementa un nodo ROS2 que se comunica con un carro robótico controlado por un ESP8266 mediante TCP/IP. El microcontrolador maneja los motores y el sensor ultrasónico HC-SR04, mientras que ROS2 envía comandos y recibe la distancia medida en tiempo real.

Video del proyecto:  
**[ENLACE DEL VIDEO AQUÍ]**

---

## 📦 Estructura entregada
src/
└── robot_car_ros/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│ └── robot_car_ros
└── robot_car_ros/
├── init.py
└── car_bridge_node.py

yaml
Copy code

---

# 📌 Requisitos previos
### ROS2 Humble / Foxy / Jazzy instalado en Linux  
Guía oficial: https://docs.ros.org/

### Python 3

### Carro físico (no incluido en esta entrega):
- ESP8266 (NodeMCU)
- Firmware que recibe comandos F/B/L/R/S por TCP puerto 4210
- Envío de distancia con formato `DIST:<valor_en_cm>`

El PC y el ESP8266 deben estar en la misma red WiFi.

---

# 🚀 Instalación del paquete ROS2
Copiar la carpeta `robot_car_ros` dentro del `src/` del workspace ROS2 del usuario:

```bash
cd ~/ros2_ws/src
# colocar aquí la carpeta robot_car_ros
Luego compilar:

bash
Copy code
cd ~/ros2_ws
colcon build
source install/setup.bash
▶️ Ejecución del nodo ROS2
Lanzar el nodo puente (TCP → ROS2):

bash
Copy code
ros2 run robot_car_ros car_bridge_node --ros-args -p car_ip:=<IP_DEL_CARRO>
Ejemplo:

bash
Copy code
ros2 run robot_car_ros car_bridge_node --ros-args -p car_ip:=192.168.4.1
📡 Tópicos ROS2 del proyecto
1. Enviar comandos al carro
bash
Copy code
ros2 topic pub /car_cmd std_msgs/msg/String "data: 'F'" -1
Comandos aceptados:

Comando	Acción
F	Adelante
B	Atrás
L	Izquierda
R	Derecha
S	Stop

2. Leer la distancia del sensor ultrasónico
bash
Copy code
ros2 topic echo /car_distance
Ejemplo de salida:

yaml
Copy code
data: 23.4
---
data: 21.8
---
data: 25.0
👤 Autor
nick-zh
Correo: nichoruizhilkin@gmail.com

yaml
Copy code

---

Si lo quieres también en versión “limpia” sin emojis, dímelo y te lo dejo igual pero totalmente plano.
