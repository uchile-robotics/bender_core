#!/usr/bin/env python3
# File: emotion_action_server.py

# File: emotion_action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from custom_action_interfaces.action import Emotion
from std_msgs.msg import String
import asyncio

class EmotionActionServer(Node):
    def __init__(self):
        super().__init__('emotion_action_server')

        # Action Server
        self._action_server = ActionServer(
            self,
            Emotion,
            'perform_emotion',
            self.execute_callback
        )

        # Publisher hacia /emotion_action
        self.emotion_action_pub = self.create_publisher(String, '/emotion_action', 10)

        self.last_emotion = "none"  # Guarda la emoción anterior
        self.get_logger().info("EmotionActionServer iniciado. Esperando metas en 'perform_emotion'...")

    async def execute_callback(self, goal_handle):
        """Callback principal del Action Server."""
        new_emotion = goal_handle.request.emotion_name
        self.get_logger().info(f"Ejecutando emoción: {new_emotion}")

        # Publicar en el tópico /emotion_action
        emotion_msg = String()
        emotion_msg.data = new_emotion
        self.emotion_action_pub.publish(emotion_msg)
        self.get_logger().info(f"Publicado en /emotion_action: {new_emotion}")

        # Enviar feedback con la emoción anterior
        feedback_msg = Emotion.Feedback()
        feedback_msg.previous_emotion = self.last_emotion
        feedback_msg.current_status = f"Cambiando de '{self.last_emotion}' a '{new_emotion}'"
        goal_handle.publish_feedback(feedback_msg)

        if new_emotion in ["si", "no"]:
            await asyncio.sleep(1.5)
        else:
            await asyncio.sleep(3)
        # Actualizar emoción actual
        self.last_emotion = new_emotion
        goal_handle.succeed()

        # Resultado final
        result = Emotion.Result()
        result.success = True
        result.message = f"Emoción '{new_emotion}' ejecutada con éxito (antes estaba en '{feedback_msg.previous_emotion}')"

        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = EmotionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("EmotionActionServer detenido por el usuario.")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
