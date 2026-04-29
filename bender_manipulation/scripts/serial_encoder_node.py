#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class SerialEncoderReader(Node):
    def __init__(self):
        super().__init__('serial_encoder_node')

        # Parámetros de conexión
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            # timeout=0.1 es importante para que no bloquee el nodo si no hay datos
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Conectado al encoder en {port} a {baud} baudios.")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir el puerto serial: {e}")
            exit(1)

        # Publisher: Solo publicamos el ángulo leído
        self.pub_angle = self.create_publisher(Float32, 'shoulder/angle', 10)

        # Timer: Revisar el puerto serial cada 0.01 segundos (100 Hz)
        self.timer = self.create_timer(0.01, self.serial_callback)

    def serial_callback(self):
        """Revisa si hay datos en el buffer serial y publica."""
        if self.ser.in_waiting > 0:
            try:
                # Leemos la línea, decodificamos bytes a string y quitamos espacios/\n
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    # Convertimos a float y publicamos
                    angle_value = float(line)
                    
                    msg = Float32()
                    msg.data = angle_value
                    self.pub_angle.publish(msg)
                    
                    # Opcional: Debug para ver qué llega
                    # self.get_logger().info(f"Encoder: {angle_value}")

            except ValueError:
                # A veces llega basura por el serial, lo ignoramos
                pass
            except Exception as e:
                self.get_logger().warn(f"Error leyendo serial: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialEncoderReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
