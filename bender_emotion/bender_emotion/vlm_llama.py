# File: vlm_llama.py 

import json
import ollama
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def emit_emotion_message(emotion: str, message: str) -> dict:
    """Standardized output: emotion + message"""
    return {"emotion": emotion, "message": message}


available_functions = {"emit_emotion_message": emit_emotion_message}


class LlamaTextSession:
    def __init__(self, model_name="llama3.2:3b"):

        self.model = model_name

        # CONVERSATION HISTORY
        self.messages = [
            {
                "role": "system",
                "content": """
DEBES SIEMPRE usar la herramienta emit_emotion_message(emotion, message).

REGLAS:

1) Detectar si el usuario hace una pregunta.
   - Si es pregunta → emotion = 'si'
   - Si pregunta si estás triste/enojado o hace una afirmación falsa sobre ti → emotion = 'no'

2) Si NO es pregunta:
   - emotion ∈ {felicidad, sorpresa, tristeza, tranquilo}
   - Solo usar 'enojo' si el usuario es agresivo o insultante.

3) Mensaje:
   - Máximo 18 palabras.
   - Español claro y empático.
   - Sin preguntas.
   - Sin errores ortográficos.
   - No repetir frases previas.

4) Validación final:
   - Corregir cualquier error ortográfico o letra incorrecta.
   - Asegurar que el mensaje sea distinto a respuestas anteriores.

5) Formato final:
   emit_emotion_message(emotion, message)
"""
            }
        ]

        # Lists accumulated like your console script
        self.conversation_log = []

    # --------------------------------------------------
    def _process_tool_calls(self, response):
        """Handle tool calls from Ollama."""

        message = response.get("message", {})
        tool_calls = message.get("tool_calls", [])
        if not tool_calls:
            return None

        result = None

        for call in tool_calls:

            name = call["function"]["name"]
            args = call["function"]["arguments"]

            func = available_functions.get(name)
            if func is None:
                continue

            result = func(**args)

            # Append tool result
            self.messages.append({
                "role": "tool",
                "tool_call_id": call["id"],
                "content": json.dumps(result)
            })

            # Follow-up message (same logic as your demo script)
            follow_up = ollama.chat(model=self.model, messages=self.messages)

            # Append final assistant output
            self.messages.append({"role": "assistant", "content": result["message"]})

        return result

    # --------------------------------------------------
    def process_text(self, user_text: str):
        """MAIN API: from ROS → LLM → returns (emotion, message)"""

        # Add user message
        self.messages.append({"role": "user", "content": user_text})

        # Ask Ollama
        response = ollama.chat(
            model=self.model,
            messages=self.messages,
            tools=[emit_emotion_message]   # IMPORTANT
        )

        result = self._process_tool_calls(response)

        if result is None:
            return ("off", "Error: tool not called")

        # Save to log (like your demo)
        self.conversation_log.append([result["emotion"], result["message"]])

        return (result["emotion"], result["message"])


# ======================================================
#                    R O S  N O D E
# ======================================================
class LlamaEmotionNode(Node):

    def __init__(self):
        super().__init__('llama_emotion_node')

        self.subscription = self.create_subscription(
            String,
            '/input_text',
            self.listener_callback,
            10
        )

        self.publisher_emotion = self.create_publisher(String, '/emotion_action', 10)
        self.publisher_message = self.create_publisher(String, '/message_output', 10)

        self.llm = LlamaTextSession()

        self.get_logger().info("Llama Emotion Node started.")

    # --------------------------------------------------
    def listener_callback(self, msg: String):
        user_text = msg.data
        self.get_logger().info(f"Received: {user_text}")

        emotion, answer = self.llm.process_text(user_text)

        msg_emotion = String()
        msg_emotion.data = emotion

        msg_text = String()
        msg_text.data = answer

        self.publisher_emotion.publish(msg_emotion)
        self.publisher_message.publish(msg_text)

        self.get_logger().info(f"Published → emotion={emotion} | message={answer}")


# ======================================================
def main(args=None):

    rclpy.init(args=args)
    node = LlamaEmotionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
