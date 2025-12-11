import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class EmotionSender(Node):
    def __init__(self):
        super().__init__('emotion_sender')

        # Abrir puerto serial (ajusta el puerto a tu caso: COM3 en Windows, /dev/ttyUSB0 en Linux)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Suscripción al tópico con los ángulos como string
        self.subscription = self.create_subscription(
            String,
            'emotion',  # Tópico esperado
            self.send_emotion_callback,
            10
        )

    def send_emotion_callback(self, msg):
        try:
            # Agregar salto de línea al final para que Arduino/u otro dispositivo lo reciba bien
            data_str = msg.data.strip() + "\n"
            self.ser.write(data_str.encode('utf-8'))
            self.get_logger().info(f'📤 Enviando por Serial: {data_str.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando por Serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EmotionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
