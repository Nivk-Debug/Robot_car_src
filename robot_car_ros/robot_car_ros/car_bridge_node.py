import socket
import threading
import time  

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class CarBridgeNode(Node):
    def __init__(self):
        super().__init__('car_bridge_node')

        # Parámetros:
        self.declare_parameter('car_ip', '192.168.4.1')
        self.declare_parameter('car_port', 4210)

        self.car_ip = self.get_parameter('car_ip').get_parameter_value().string_value
        self.car_port = self.get_parameter('car_port').get_parameter_value().integer_value

        self.sock = None
        self.buffer = ""

        self.connect_socket()

        # Suscriptor de comandos
        self.cmd_sub = self.create_subscription(
            String,
            'car_cmd',
            self.cmd_callback,
            10
        )

        # Publicador de distancia
        self.dist_pub = self.create_publisher(Float32, 'car_distance', 10)

        # Hilo para leer datos del carro (DIST:xx.xx)
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            'Nodo car_bridge_node listo. Suscrito a /car_cmd y publicando /car_distance'
        )

    def connect_socket(self):
        """Intenta conectar el socket TCP al carro."""
        try:
            self.get_logger().info(f'Conectando a {self.car_ip}:{self.car_port} ...')
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.car_ip, self.car_port))
            self.sock.settimeout(1.0)  # lectura con timeout
            self.get_logger().info('Conexión TCP establecida con el carro.')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar al carro: {e}')
            self.sock = None

    def cmd_callback(self, msg: String):
        """Envia comandos F/B/L/R/S al carro."""
        cmd = msg.data.strip().upper()
        if cmd not in ['F', 'B', 'L', 'R', 'S']:
            self.get_logger().warn(f'Comando inválido: {cmd}')
            return

        if self.sock is None:
            self.get_logger().warn('No hay socket TCP. Reintentando conectar...')
            self.connect_socket()
            if self.sock is None:
                return

        try:
            self.sock.send(cmd.encode('ascii'))
            self.get_logger().info(f'Enviado comando: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Error enviando comando: {e}')
            self.close_socket()

    def reader_loop(self):
        """Lee continuamente datos del carro y publica DIST en /car_distance."""
        while rclpy.ok():
            if self.sock is None:
                # Intentar reconectar cada cierto tiempo
                self.connect_socket()
                time.sleep(1.0)  
                continue

            try:
                data = self.sock.recv(128)
                if not data:
                    # conexión cerrada por el carro
                    self.get_logger().warn('Socket cerrado por el carro.')
                    self.close_socket()
                    continue

                text = data.decode('ascii', errors='ignore')
                self.buffer += text

                # Procesar líneas completas
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    self.process_line(line)

            except socket.timeout:
                
                continue
            except Exception as e:
                self.get_logger().error(f'Error leyendo del socket: {e}')
                self.close_socket()

    def process_line(self, line: str):
        """Procesa una línea del ESP
        if line.startswith('DIST:'):
            value_str = line[5:].strip()
            try:
                dist_cm = float(value_str)
                msg = Float32()
                msg.data = dist_cm
                self.dist_pub.publish(msg)
                self.get_logger().info(f'Distancia recibida: {dist_cm:.2f} cm')
            except ValueError:
                self.get_logger().warn(f'No se pudo parsear distancia: {line}')

    def close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def destroy_node(self):
        self.close_socket()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

