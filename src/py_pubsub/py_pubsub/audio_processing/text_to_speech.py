"""
Text-to-Speech ROS2 Node: 

This node uses Google Cloud Text-to-Speech to synthesis speech and publish
the robot speech audio files to the ROS2 network.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from google.cloud import texttospeech
import os
import json
from std_msgs.msg import Empty

import zmq

class GoogleTextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')
        
        # Initialize
        self.declare_parameter("output_path", "/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/py_pubsub/output.mp3")

        self.declare_parameter('prompts_file', '/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/resources/prompts.json')
        self.declare_parameter('resource_dir', '/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/resources')
        
        self.prompts_file = self.get_parameter('prompts_file').get_parameter_value().string_value
        self.resource_dir = self.get_parameter('resource_dir').get_parameter_value().string_value

        self.get_logger().info(f"Using voice: {self.load_latest_voice_name()}")

        self.output_path = self.get_parameter("output_path").get_parameter_value().string_value

        self._setup_publishers()
        self._setup_subscribers()
        self._setup_speech_synthesizer()

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:12345")  # Adjust port as needed
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.timer = self.create_timer(0.1, self.receive_msg)  # Timer to check for messages

        
    def receive_msg(self):
        """Receive messages from the ZeroMQ socket."""
        try:
            message = self.socket.recv_string(flags=zmq.NOBLOCK)
            self.get_logger().info("[TextToSpeech]Received message from ZeroMQ")
            if message.startswith("conversation started"):
                self.get_logger().info("Received start message from ZeroMQ")
                self.conversation_start_change_voice_callback(Empty())
            elif message.startswith("initial prompt conversation started"):
                self.conversation_type = "initial_prompt"
                self.get_logger().info("Received start initial prompt conversation message from ZeroMQ")
                self.conversation_start_change_voice_callback(Empty())
            # elif message.startswith("conversation ended"):
            #     self.get_logger().info("Received end message from ZeroMQ")
            #     self.conversation_end_change_voice_callback(Empty())

        except zmq.Again:
            # No message received, continue
            pass


    def load_latest_voice_name(self):
        """Load the latest voice name from the prompts file."""
        try:
            files = [f for f in os.listdir(self.resource_dir) if f.startswith("prompts_") and f.endswith(".json")]
            if not files:
                self.get_logger().warning("No prompts files found.")
                return "en-US-Standard-H"
            files.sort(key=lambda f: os.path.getmtime(os.path.join(self.resource_dir, f)), reverse=True)
            latest_file = os.path.join(self.resource_dir, files[0])
            with open(latest_file, "r") as file:
                prompts = json.load(file)
                voice_name = prompts.get("voice", "en-US-Standard-H")
                self.get_logger().info(f"Loaded latest prompts: {latest_file}")
                self.get_logger().info(f"Using voice: {voice_name}")
                return voice_name
        except Exception as e:
            self.get_logger().error(f"Failed to load prompts file: {e}")
            return "en-US-Standard-H"

    def _setup_publishers(self) -> None:
        """Set up ROS2 publishers."""
        self.audio_output_publisher = self.create_publisher(
            UInt8MultiArray,  
            '/text_to_speech_output',
            10
        )

    def _setup_subscribers(self) -> None:
        """Set up ROS2 subscribers."""
        self.text_input_subscription = self.create_subscription(
            String,
            '/robot_speech_content',
            self.text_to_speech_callback,
            10
        )
    
    def _setup_speech_synthesizer(self) -> None:
        """Set up Google Cloud Text-to-Speech client and configuration."""
        self.client = texttospeech.TextToSpeechClient()
        voice_name = self.load_latest_voice_name()  # Load the latest voice name
        self.voice = texttospeech.VoiceSelectionParams(
            language_code="en-US",
            # name="en-US-Neural2-D",  # Default voice
            name=voice_name,  # Use the latest voice name loaded from prompts
            ssml_gender=texttospeech.SsmlVoiceGender.FEMALE
        )

        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            speaking_rate=0.95
        )

        self.get_logger().info("Google Text-to-Speech Node initialized")

    def conversation_start_change_voice_callback(self, msg: Empty):
        """Callback for conversation start to change voice."""
        self.get_logger().info("Conversation started, reloading voice.")
        # voice_name = self.load_latest_voice_name()
        # self.voice.name = self.voice_name
        self._setup_speech_synthesizer()
        self.get_logger().info(f"Changed voice to: {self.load_latest_voice_name()}")

    def text_to_speech_callback(self, msg):
        input_robot_speech_content = json.loads(msg.data)
        robot_speech_content = input_robot_speech_content.get('robot_speech_content', "")
        self.get_logger().info(f"Processing: {robot_speech_content}")

        # Synthesize speech
        synthesis_input = texttospeech.SynthesisInput(text=robot_speech_content)
        response = self.client.synthesize_speech(
            input=synthesis_input,
            voice=self.voice,
            audio_config=self.audio_config
        )

        # Save audio to file
        with open(self.output_path, "wb") as out:
            out.write(response.audio_content)
            self.get_logger().info(f"Audio written to {self.output_path}")

        # Convert the binary audio content (bytes) into a list of integers (individual bytes)
        audio_data = list(response.audio_content)  # Convert to list of integers (bytes)

        # Create a UInt8MultiArray message and populate its data field
        audio_message = UInt8MultiArray()  # Create the message with the correct type
        audio_message.data = audio_data  # List of integers representing the audio content

        # Publish the audio message
        self.audio_output_publisher.publish(audio_message)
        self.get_logger().info("Text-to-Speech audio published")

def main(args=None):
    rclpy.init(args=args)
    node = GoogleTextToSpeechNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Google Text-to-Speech Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()