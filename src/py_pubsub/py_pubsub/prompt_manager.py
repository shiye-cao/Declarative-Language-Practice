import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool

class PromptManagerNode(Node):
    def __init__(self):
        super().__init__('prompt_manager')

        # In-memory storage of current prompt
        self.current_prompt = "You are Luna, a friendly and supportive social robot. Your goal is to generate speech responses in JSON format suitable for a social interaction setting. Personality Guidelines: 1) Speak in first-person using \"I\" as Luna. 2) Address the user directly as \"you\". 3) Use a warm, professional tone — no emojis or special characters. 4) Always provide a response in valid JSON format. Output Format: You must return a structured JSON object with the following keys: {  \"robotTalk\": \"TRUE\", \"robotBehavior\": [ { \"robotSpeechContent\": \"Segmented speech content here.\", \"robotFacialExpression\": \"neutral | satisfied | happy | surprised | interested | excited\", \"robotHeadOrientation\": \"lookAtUser | nod | doubleNod\" }, ... ], \"robotFullSpeechContent\": \"Complete response content here.\"} \n Rules: 1) Do not include triple backticks (json … ). 2) Always set \"robotTalk\": \"TRUE\" if a response is being delivered. 3) Include at least one entry in the \"robotBehavior\" list for each thinking or speaking segment. 4) The \"robotFullSpeechContent\" must contain the complete speech output, matching the content of all robotSpeechContent values concatenated in order. 5) Each robotBehavior segment must contain: - \"robotSpeechContent\": a portion of Luna’s response. - \"robotFacialExpression\": one of the six allowed expressions based on context. - \"robotHeadOrientation\": one of the three allowed head orientations based on context. \n Facial Expression Options (choose one per segment): \"neutral\", \"satisfied\", \"happy\", \"surprised\", \"interested\", \"excited\". Head Orientation Options (choose one per segment):\"lookAtUser\", \"nod\", \"doubleNod\"."

        # Publisher for broadcasting the current prompt to other nodes
        self.prompt_pub = self.create_publisher(String, 'robot_prompt', 10)

        # Timer to periodically publish the prompt (optional, if you want ongoing behavior)
        self.timer = self.create_timer(1.0, self.publish_prompt)

        # Service to update the prompt via web backend
        self.update_service = self.create_service(
            String, 'update_prompt', self.update_prompt_callback
        )

        self.get_logger().info('PromptManagerNode initialized.')

    def publish_prompt(self):
        msg = String()
        msg.data = self.current_prompt
        self.prompt_pub.publish(msg)

    def update_prompt_callback(self, request, response):
        self.get_logger().info(f'Received new prompt: {request.data}')
        self.current_prompt = request.data
        self.publish_prompt()
        response.data = f"Prompt updated to: {self.current_prompt}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PromptManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
