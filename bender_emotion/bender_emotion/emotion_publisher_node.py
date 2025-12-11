# File: emotion_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import time


class EmotionPublisherNode(Node):
    def __init__(self):
        super().__init__('emotion_publisher_node')

        # Publicadores
        self.publisher_ = self.create_publisher(Float64MultiArray, '/goal_pos', 10)
        self.emotion_publisher_ = self.create_publisher(String, 'emotion', 10)

        # Suscriptor para recibir emociones desde otro nodo
        self.emotion_subscriber_ = self.create_subscription(
            String,
            'emotion_action',
            self.emotion_callback,
            10
        )

        # Diccionario de emociones y coordenadas
        self.emotions = {
            "off": [[150, 62.70, 219.73]],
            "talking": [[150, 62.70, 219.73]],
            "felicidad": [[150, 62.70, 219.73]],
            "sorpresa": [[150, 62.70, 219.73]],
            "tranqui": [[150, 62.70, 219.73]],
            "masbrillo": [[150, 62.70, 219.73]],
            "menosbrillo": [[150, 62.70, 219.73]],
            "brillo": [[150, 62.70, 219.73]],
            "tristeza": [[150, 79.10, 206.25]],
            "enojo": [[150, 74.12, 210.94]],
            "no": [
                [139, 62.70, 219.73],
                [161, 62.70, 219.73],
            ],
            "si": [
                [150, 74.12, 210.94],
                [150, 51.25, 225.88],
            ],
        }

        self.get_logger().info("Nodo 'emotion_publisher_node' iniciado. Esperando mensajes en 'emotion_action'...")

    def emotion_callback(self, msg: String):
        """Callback ejecutado al recibir una emoción por el tópico 'emotion_action'."""
        emotion = msg.data.strip().lower()
        self.get_logger().info(f"Recibido comando de emoción: {emotion}")
        self.publish_emotion(emotion)

    def publish_emotion(self, emotion: str):
        if emotion == "tranquilo":
            emotion = "tranqui"

        """Publica los mensajes correspondientes a la emoción y su nombre."""
        if emotion not in self.emotions:
            self.get_logger().error(f"Emoción desconocida: {emotion}")
            return
        
        poses = self.emotions[emotion]

        # Publicar nombre de la emoción
        name_msg = String()
        name_msg.data = emotion
        self.emotion_publisher_.publish(name_msg)
        self.get_logger().info(f"Publicado nombre de emoción: {emotion}")

        # Caso especial para “sí” y “no”
        if emotion in ["si", "no"]:
            for rep in range(2):
                for i, data in enumerate(poses):
                    msg = Float64MultiArray()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"{emotion} (ciclo {rep+1}) posición {i+1}: {data}")
                    time.sleep(0.4)
            # Volver a posición normal
            normal_pose = self.emotions["off"][0]
            msg = Float64MultiArray()
            msg.data = normal_pose
            self.publisher_.publish(msg)
            self.get_logger().info(f"Regresando a posición normal: {normal_pose}")

        # Caso brillo
        elif emotion.startswith("brillo"):
            poses = self.emotions["brillo"]
            msg = Float64MultiArray()
            msg.data = poses[0]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publicado brillo: {poses[0]}")
        else:
            msg = Float64MultiArray()
            msg.data = poses[0]
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publicado {emotion}: {poses[0]}")


def main(args=None):
    rclpy.init(args=args)
    node = EmotionPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Nodo interrumpido por el usuario.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

