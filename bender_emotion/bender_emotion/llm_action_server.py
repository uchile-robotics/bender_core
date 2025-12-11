#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from custom_action_interfaces.action import LlamaSpeak
from std_msgs.msg import String
import time
class LlamaBridgeActionServer(Node):

    def __init__(self):
        super().__init__('llama_bridge_action_server')

        # Action Server
        self._action_server = ActionServer(
            self,
            LlamaSpeak,
            'llama_speak',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers hacia el nodo LLM
        self.pub_input = self.create_publisher(String, '/input_text', 10)

        # Variables para almacenar respuestas del LLM
        self.received_emotion = None
        self.received_message = None

        # Subscribers del nodo LLM
        self.sub_emotion = self.create_subscription(
            String, '/emotion_action', self.emotion_callback, 10)

        self.sub_message = self.create_subscription(
            String, '/message_output', self.message_callback, 10)

        self.get_logger().info("Llama Bridge Action Server iniciado.")

    # ---------------------------------------------------------
    # Callbacks de TOPIC
    # ---------------------------------------------------------
    def emotion_callback(self, msg):
        self.received_emotion = msg.data

    def message_callback(self, msg):
        self.received_message = msg.data

    # ---------------------------------------------------------
    # GOAL + CANCEL
    # ---------------------------------------------------------
    def goal_callback(self, goal_request):
        if len(goal_request.user_text.strip()) == 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    # ---------------------------------------------------------
    # EXECUTE CALLBACK
    # ---------------------------------------------------------
    def execute_callback(self, goal_handle):

        user_text = goal_handle.request.user_text

        # Reset previous responses
        self.received_emotion = None
        self.received_message = None

        # Enviar texto al nodo LLM
        to_send = String()
        to_send.data = user_text
        self.pub_input.publish(to_send)

        self.get_logger().info(f"▶️ Enviado a LLM: {user_text}")

        # Feedback inicial
        feedback = LlamaSpeak.Feedback()
        feedback.status = "Procesando LLM..."
        goal_handle.publish_feedback(feedback)

        # Esperar respuesta LLM
        for _ in range(200):  # ~10 segundos
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = LlamaSpeak.Result()
                result.success = False
                result.emotion = "off"
                result.llm_output = "Cancelado"
                return result

            if self.received_emotion is not None and self.received_message is not None:
                break

            time.sleep(0.05)

        # Timeout
        if self.received_emotion is None or self.received_message is None:
            goal_handle.abort()
            result = LlamaSpeak.Result()
            result.success = False
            result.emotion = "off"
            result.llm_output = "No se recibió respuesta del LLM"
            return result

        # Guardar respuestas antes de limpiar
        emotion = self.received_emotion
        message = self.received_message

        # Limpiar buffers
        self.received_emotion = None
        self.received_message = None

        # Resultado final
        result = LlamaSpeak.Result()
        result.success = True
        result.emotion = emotion
        result.llm_output = message

        goal_handle.succeed()
        return result



def main(args=None):
    rclpy.init(args=args)
    node = LlamaBridgeActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
