
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from uchile_srvs.action import Emotion
from std_msgs.msg import String
import time


class EmotionActionServer(Node):
    def __init__(self):
        super().__init__('emotion_action_server')

        self._action_server = ActionServer(
            self,
            Emotion,
            'perform_emotion',
            self.execute_callback
        )

        self.emotion_action_pub = self.create_publisher(
            String, '/emotion', 10
        )

        self.last_emotion = "none"
        self.get_logger().info(
            "EmotionActionServer iniciado. Esperando metas en 'perform_emotion'..."
        )

    def execute_callback(self, goal_handle):
        new_emotion = goal_handle.request.emotion_name
        self.get_logger().info(f"Ejecutando emoción: {new_emotion}")

        # Publicar emoción
        msg = String()
        msg.data = new_emotion
        self.emotion_action_pub.publish(msg)

        # Feedback
        feedback = Emotion.Feedback()
        feedback.current_status = (
            f"Cambiando de '{self.last_emotion}' a '{new_emotion}'"
        )
        goal_handle.publish_feedback(feedback)

        # Simular duración
        if new_emotion in ["si", "no"]:
            time.sleep(1.5)
        else:
            time.sleep(3.0)

        # Actualizar estado
        previous = self.last_emotion
        self.last_emotion = new_emotion
        goal_handle.succeed()

        result = Emotion.Result()
        result.success = True
        result.message = (
            f"Emoción '{new_emotion}' ejecutada con éxito "
            f"(antes estaba en '{previous}')"
        )

        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = EmotionActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
