import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import json
from openai import OpenAI

import zmq

from py_pubsub.conversation_logger import get_conversation_logger
# from py_pubsub.audio_processing.speech_to_text import SpeechToTextNode

FALL_BACK_RESPONSE_NORMAL = json.dumps({
    "robotTalk": "TRUE",
    "robotBehavior": [
        {
            "robotSpeechContent": "Sorry, I didn't get that. Can you repeat that please?",
            "robotFacialExpression": "neutral",
            "robotHeadOrientation": "lookAtUser"
        }
    ],
    "robotFullSpeechContent": "Sorry, I didn't get that. Can you repeat that please?"
})

class RobotBehaviorGenerator(Node):
    def __init__(self):
        super().__init__('robot_behavior_generator')
        
        # Parameters
        self.declare_parameter('openai_api_key', os.environ.get('OPENAI_API_KEY'))
        self.declare_parameter('openai_api_url', 'https://api.openai.com/v1/chat/completions')
        self.declare_parameter('prompts_file', '/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/resources/prompts.json')
        self.declare_parameter('latest_initial_prompt_file', '/home/icl/2026-hri-conversation-design-experimental/txt_files/initial_prompts/initial_prompt_latest.txt')
        # self.declare_parameter('prompts_file_latest', '/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/resources/prompts.json') # 0711
        self.declare_parameter('resource_dir', '/home/icl/2026-hri-conversation-design-experimental/src/py_pubsub/resources')
        
        self.api_key = self.get_parameter('openai_api_key').get_parameter_value().string_value
        self.api_url = self.get_parameter('openai_api_url').get_parameter_value().string_value
        self.prompts_file = self.get_parameter('prompts_file').get_parameter_value().string_value
        # self.prompts_file_latest = self.get_parameter('prompts_file_latest').get_parameter_value().string_value
        self.resource_dir = self.get_parameter('resource_dir').get_parameter_value().string_value
        self.conversation_type = "normal"  # Default conversation type

        # ROS2 Subscribers 0711
        self.conversation_active = False

        # ROS2 Subscribers
        self.user_input_subscription = self.create_subscription(
            String,
            '/user_speech',  # FROM: speech_to_text
            self.user_input_callback,
            10
        )

        self.user_intent_subscription = self.create_subscription(
            String,
            '/user_intent',
            self.user_intent_callback,
            10
        )
        
        self.robot_speech_subscription = self.create_subscription(
            String,
            '/robot_speech_content',  # FROM: robot_controller
            self.robot_speech_callback, 
            10
        )

        self.prompt_update_subscription = self.create_subscription(
            String,
            '/updated_prompt',
            self.update_prompt_callback,
            10
        )

        # ROS2 Publisher - only publish to robot_controller
        self.robot_behavior_publisher = self.create_publisher(
            String,
            '/robot_behavior',  # TO: robot_controller
            10
        )

        # Internal states
        self.prompts = self.load_prompts(self.prompts_file)
        self.conversation_history = []
        self.prev_robot_speech = ""
        
        self.get_logger().info("Robot Behavior Generator Node initialized")

        self.logger = get_conversation_logger()

        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:12345")  # Adjust port as needed
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.timer = self.create_timer(0.1, self.receive_msg)  # Timer to check for messages

        
    def receive_msg(self):
        """Receive messages from the ZeroMQ socket."""
        try:
            message = self.socket.recv_string(flags=zmq.NOBLOCK)
            self.get_logger().info("[BehaviorGenerator]Received message from ZeroMQ")
            
            if message.startswith("conversation started"):
                self.conversation_type = "normal"
                self.get_logger().info("Received start conversation message from ZeroMQ")
                self.conversation_start_callback(Empty())
            elif message.startswith("initial prompt conversation started"):
                self.conversation_type = "initial_prompt"
                self.get_logger().info("Received start initial prompt conversation message from ZeroMQ")
                self.conversation_start_callback(Empty())
            elif message.startswith("conversation ended"):
                self.conversation_type = "normal"
                self.get_logger().info("Received end message from ZeroMQ")
                self.conversation_end_callback(Empty())

        except zmq.Again:
            # No message received, continue
            pass


    def load_prompts(self, file_path):
        """Load predefined prompts from a JSON file."""
        try:
            with open(file_path, 'r') as file:
                prompts = json.load(file)
                self.get_logger().info("Prompts loaded successfully")
                return prompts
        except Exception as e:
            self.get_logger().error(f"Error loading prompts: {str(e)}")
            return {}
        
    def load_latest_prompts(self):
        files = [f for f in os.listdir(self.resource_dir) if f.startswith("prompts_") and f.endswith(".json")]
        if not files:
            self.get_logger().warning("No prompts files found.")
            return {}
        files.sort(key=lambda f: os.path.getmtime(os.path.join(self.resource_dir, f)), reverse=True)
        latest_file = os.path.join(self.resource_dir, files[0])
        with open(latest_file, "r") as file:
            prompts = json.load(file)
            self.get_logger().info(f"Loaded latest prompts: {latest_file}")
            return prompts
        
    # def conversation_start_callback(self, msg): #0711
    #     self.conversation_active = True
    #     self.prompts = self.load_prompts(self.prompts_file_latest)  # Load latest prompts
    #     self.get_logger().info("Conversation starts, prompts reloaded")

    def conversation_start_callback(self, msg):
        self.conversation_active = True
        self.prompts = self.load_latest_prompts()
        self.conversation_history = []
        self.prev_robot_speech = ""
        self.get_logger().info("Conversation starts, latest prompts loaded")

        response = self.generate_robot_behavior("start")
        self.publish_robot_behavior(response)
        
    def conversation_end_callback(self, msg):
        """Callback for conversation end event."""
        self.conversation_active = False
        self.get_logger().info(f"Conversation ended, current conversation active status: {self.conversation_active}")
        # save conversation history to log with timestamp
        if self.conversation_history:
            self.get_logger().info("Saving conversation history")
            # Save conversation history to logger 
            self.logger.save_conversation_history(self.conversation_history, 'robot_behavior_generator')
        # Clear conversation history
        self.conversation_history.clear()
        self.prev_robot_speech = ""
        self.get_logger().info(f"Cleared: {self.conversation_history}")
        self.get_logger().info("Conversation history cleared")

    def user_input_callback(self, msg):
        """Processes user input and generates appropriate robot behavior."""
        if not self.conversation_active:
            self.conversation_history.clear()
            self.prev_robot_speech = ""
            self.get_logger().warning("Conversation is not active, ignoring user input")
            return
        try:
            user_input = json.loads(msg.data)
            user_speech = user_input.get('transcribed_audio', "")
            self.get_logger().info(f"User Speech: {user_speech}")
            self.logger.log_user_speech(user_speech, 'robot_behavior_generator')

            # Generate and publish robot behavior
            response = self.generate_robot_behavior(user_speech)
            self.publish_robot_behavior(response)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse user input JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in user input callback: {e}")

    def user_intent_callback(self, msg):
        """Processes user intent and generates appropriate robot behavior."""
        if not self.conversation_active:
            self.conversation_history.clear()
            self.prev_robot_speech = ""
            self.get_logger().warning("Conversation is not active, ignoring user intent")
            return
        try:
            user_intent_data = json.loads(msg.data)
            user_intent_type = user_intent_data.get('user_intent_type', "")
            user_speech = user_intent_data.get('user_speech', "")
            is_wakeword = user_intent_data.get('is_wakeword', False)
            is_stop_only = user_intent_data.get('is_stop_only', False)
            
            self.logger.log_user_interruption(user_speech, user_intent_type, is_wakeword, 'robot_behavior_generator')

            self.get_logger().info(f"User Intent: {user_intent_type}, Speech: '{user_speech}'")
            
            # Handle different intent types
            if is_stop_only or is_wakeword or user_intent_type == "disruptive":
                self.get_logger().info(f"{user_intent_type} detected - no response generated")
                return  # Don't generate any behavior
            
            # Waiting for the actual speech
            elif user_intent_type == "cooperative-clarification":
                self.get_logger().info("Clarification request - generating response with baseline prompt")
                # Generate response using baseline prompt
                # response = self.generate_robot_behavior(user_speech)
                # self.publish_robot_behavior(response)
                return
                
            elif user_intent_type == "cooperative-agreement":
                self.get_logger().info("Agreement detected - no response generated")
                return  # Don't generate any behavior
                
            elif user_intent_type == "cooperative-assistance":
                self.get_logger().info("User assistance detected - no response generated")
                return  # Don't generate any behavior
                
            elif user_intent_type == "not_interruption":
                self.get_logger().info("Regular speech - generating normal response")
                # Normal conversation flow
                response = self.generate_robot_behavior(user_speech)
                self.publish_robot_behavior(response)
                
            else:
                self.get_logger().warning(f"Unknown intent type: {user_intent_type}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse user intent JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in user intent callback: {e}")

    def robot_speech_callback(self, msg):
        """Store the previous robot speech for context."""
        try:
            data = json.loads(msg.data)
            self.prev_robot_speech = data.get("robot_speech_content", "")
            self.get_logger().info(f"Robot speech received: {self.prev_robot_speech}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse robot speech JSON: {e}")
            # Fallback to raw message data
            self.prev_robot_speech = msg.data
        except Exception as e:
            self.get_logger().error(f"Error in robot speech callback: {e}")

    def update_prompt_callback(self, msg):
        try:
            new_prompts = json.loads(msg.data)
            self.prompts = new_prompts
            self.get_logger().info("Updated prompts via topic /updated_prompt")
        except Exception as e:
            self.get_logger().error(f"Failed to update prompts: {e}")

    def generate_robot_behavior(self, user_speech_content):
        """Generates robot behavior based on user speech content and conversation history."""
        self.get_logger().info(f"conversation active status: {self.conversation_active}")
        self.get_logger().info(f"Generating robot behavior for user speech, while conversation history is: {self.conversation_history}")
        if not self.conversation_active:
            self.get_logger().warning("Conversation is not active, ignoring user speech")
            self.conversation_history.clear()
            self.prev_robot_speech = ""
            return
        
        # Get system prompt
        if self.conversation_type == "normal":
            # Send thinking status to robot controller 
            self.publish_thinking_status()
            system_prompt = self.prompts.get("baseline", "") + "Always provide a response in valid JSON format. Output Format: You must return a structured JSON object with the following keys: {  \"robotTalk\": \"TRUE\", \"robotBehavior\": [ { \"robotSpeechContent\": \"Segmented speech content here.\", \"robotFacialExpression\": \"neutral | satisfied | happy | surprised | interested | excited\", \"robotHeadOrientation\": \"lookAtUser | nod | doubleNod\" }, ... ], \"robotFullSpeechContent\": \"Complete response content here.\"} \n Rules: 1) Do not include triple backticks (json … ). 2) Always set \"robotTalk\": \"TRUE\" if a response is being delivered. 3) Include at least one entry in the \"robotBehavior\" list for each thinking or speaking segment. 4) The \"robotFullSpeechContent\" must contain the complete speech output, matching the content of all robotSpeechContent values concatenated in order. 5) Each robotBehavior segment must contain: - \"robotSpeechContent\": a portion of Luna’s response. - \"robotFacialExpression\": one of the six allowed expressions based on context. - \"robotHeadOrientation\": one of the three allowed head orientations based on context. \n Facial Expression Options (choose one per segment): \"neutral\", \"satisfied\", \"happy\", \"surprised\", \"interested\", \"excited\". Head Orientation Options (choose one per segment):\"lookAtUser\", \"nod\", \"doubleNod\"."
        elif self.conversation_type == "initial_prompt":
            system_prompt = self.prompts.get("baseline", "") 
        model_choose = {
            "gpt-4o":"gpt-4o-2024-05-13",
            "gpt-4o-mini": "gpt-4o-mini-2024-07-18",
            "gpt-4.1-mini": "gpt-4.1-mini-2025-04-14"
        }
        model_type = self.prompts.get("model", "gpt-4.1-mini")
        model_to_use = "gpt-4.1-mini-2025-04-14"
        temperature = 0.5

        self.get_logger().info(f"Model to use: {model_to_use}")
        self.get_logger().info(f"Prompt: {system_prompt}")

        if self.conversation_type == "normal":
            # Format user input for GPT
            formatted_user_input = (
                f"[User Speech: {user_speech_content}]\n"
                f"[Previous Robot Speech: {self.prev_robot_speech}]\n"
            )
        elif self.conversation_type == "initial_prompt":
            latest_initial_prompt = ""
            if os.path.exists(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value):
                with open(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value, 'r') as f:
                    latest_initial_prompt = f.read().strip()

            # Format user input for GPT
            formatted_user_input = (
                f"[User Speech: {user_speech_content}]\n"
                f"[Previous ACE Speech: {self.prev_robot_speech}]\n"
                f"[Current Initial ACE Prompt: {latest_initial_prompt}]\n"
            )

        self.get_logger().info(formatted_user_input)

        # Generate robot behavior using OpenAI
        output = self.call_openai(system_prompt, formatted_user_input, model_to_use, temperature)
        
        # Update conversation history
        if user_speech_content:
            self.conversation_history.append({"role": "user", "content": formatted_user_input})
        
        self.trim_conversation_history()
        return output

    def publish_thinking_status(self):
        """Publish thinking status to robot controller."""
        thinking_behavior = {
            "robotBehavior": [{
                "robotSpeechContent": "",
                "robotFacialExpression": "thinking",
                "robotHeadMovement": "thinking",
                "status": "thinking"
            }]
        }
        # dont think if it is not disruptive or normal talking

        msg = String()
        msg.data = json.dumps(thinking_behavior)
        self.robot_behavior_publisher.publish(msg)
        self.get_logger().info("Published thinking status to robot controller")

    def trim_conversation_history(self):
        """Ensures the conversation history does not exceed 20 entries (10 exchanges)."""
        while len(self.conversation_history) > 20:
            self.conversation_history.pop(0)

    def publish_robot_behavior(self, response):
        """Publishes the generated robot behavior to robot controller."""
        try:
            # Validate JSON response
            response_dict = json.loads(response)

            # FOR INITIAL PROMPT DRAFTING AGENT ONLY! 
            initial_prompt_draft = response_dict.get("promptDraft", "")
            if initial_prompt_draft:
                self.get_logger().info(f"Initial Prompt Draft: {initial_prompt_draft}")

            # Save the initial prompt draft to a file
            if initial_prompt_draft:
                try:
                    # Create directory if it doesn't exist
                    os.makedirs("/home/icl/2026-hri-conversation-design-experimental/txt_files/initial_prompts", exist_ok=True)
                    
                    # Save the prompt draft to file
                    with open("/home/icl/2026-hri-conversation-design-experimental/txt_files/initial_prompts/initial_prompt_latest.txt", "w") as f:
                        f.write(initial_prompt_draft)
                    
                    self.get_logger().info("Initial prompt draft saved to file successfully")
                except Exception as e:
                    self.get_logger().error(f"Failed to save initial prompt draft to file: {e}")

            if self.conversation_type == "normal":
                robot_behavior = response_dict.get("robotBehavior", [])
            elif self.conversation_type == "initial_prompt":
                robot_behavior = response_dict.get("AceBehavior", [])

            if not robot_behavior:
                self.get_logger().warning("No robot behavior found in response")
                return
            elif self.conversation_type == "normal":
                # Log behavior details
                for behavior in robot_behavior:
                    robot_speech_content = behavior.get("robotSpeechContent", "Sorry, I didn't get that. Can you repeat that please?")
                    robot_facial_expression = behavior.get("robotFacialExpression", "neutral")
                    if robot_speech_content:
                        self.get_logger().info(f"Robot Behavior: {robot_speech_content} ({robot_facial_expression})")

                # Update conversation history
                self.conversation_history.append({"role": "assistant", "content": response})

                # Publish to robot controller
                msg = String()
                msg.data = json.dumps(response_dict)  # Send full response dict
                self.robot_behavior_publisher.publish(msg)
            elif self.conversation_type == "initial_prompt":
                # Log behavior details
                for behavior in robot_behavior:
                    robot_speech_content = behavior.get("robotSpeechContent", "Sorry, I didn't get that. Can you repeat that please?")
                    robot_facial_expression = behavior.get("robotFacialExpression", "neutral")
                    if robot_speech_content:
                        self.get_logger().info(f"Robot Behavior: {robot_speech_content} ({robot_facial_expression})")

                # Update conversation history
                self.conversation_history.append({"role": "assistant", "content": response})

                for item in robot_behavior:
                    if "AceSpeechContent" in item:
                        item["robotSpeechContent"] = item.pop("AceSpeechContent")

                key_map = {
                    "AceTalk": "robotTalk",
                    "AceBehavior": "robotBehavior",
                    "AceFullSpeechContent": "robotFullSpeechContent"
                }

                # rename response
                response_dict = (
                    {key_map.get(k, k): (
                        [{key_map.get(k2, k2): v2 for k2, v2 in i.items()} if isinstance(i, dict) else i for i in v]
                        if isinstance(v, list) else
                        {key_map.get(k2, k2): v2 for k2, v2 in v.items()} if isinstance(v, dict) else v
                    ) for k, v in response_dict.items()}
                )

                # Publish to robot controller
                msg = String()
                msg.data = json.dumps(response_dict)  # Send full response dict
                self.robot_behavior_publisher.publish(msg)
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding GPT response JSON: {e}")

            # Fallback behavior if response is invalid
            robot_speech_content = "Sorry, I didn't get that. Can you repeat that please?"
            robot_facial_expression = "neutral"
            self.get_logger().info(f"Robot Behavior: {robot_speech_content} ({robot_facial_expression})")
            
            if self.conversation_type == "normal":
                self.conversation_history.append({"role": "assistant", "content": FALL_BACK_RESPONSE_NORMAL})
                self.publish_robot_behavior(FALL_BACK_RESPONSE_NORMAL)
            elif self.conversation_type == "initial_prompt":
                latest_initial_prompt = ""
                if os.path.exists(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value):
                    with open(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value, 'r') as f:
                        latest_initial_prompt = f.read().strip()
                FALL_BACK_RESPONSE_ACE= json.dumps({
                    "AceTalk": "TRUE",
                    "promptDraft": latest_initial_prompt, 
                    "AceBehavior": [
                        {
                            "AceSpeechContent": "Sorry, I didn't get that. Can you repeat that please?",
                        }
                    ],
                    "AceFullSpeechContent": "Sorry, I didn't get that. Can you repeat that please?"
                })
                self.conversation_history.append({"role": "assistant", "content": FALL_BACK_RESPONSE_ACE})
                self.publish_robot_behavior(FALL_BACK_RESPONSE_NORMAL)

        except Exception as e:
            self.get_logger().error(f"Error publishing robot behavior: {e}")

            # Fallback behavior if response is invalid
            robot_speech_content = "Sorry, I didn't get that. Can you repeat that please?"
            robot_facial_expression = "neutral"
            self.get_logger().info(f"Robot Behavior: {robot_speech_content} ({robot_facial_expression})")

            if self.conversation_type == "normal":
                self.conversation_history.append({"role": "assistant", "content": FALL_BACK_RESPONSE_NORMAL})
                self.publish_robot_behavior(FALL_BACK_RESPONSE_NORMAL)
            elif self.conversation_type == "initial_prompt":
                latest_initial_prompt = ""
                if os.path.exists(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value):
                    with open(self.get_parameter('latest_initial_prompt_file').get_parameter_value().string_value, 'r') as f:
                        latest_initial_prompt = f.read().strip()
                FALL_BACK_RESPONSE_ACE= json.dumps({
                    "AceTalk": "TRUE",
                    "promptDraft": latest_initial_prompt, 
                    "AceBehavior": [
                        {
                            "AceSpeechContent": "Sorry, I didn't get that. Can you repeat that please?",
                        }
                    ],
                    "AceFullSpeechContent": "Sorry, I didn't get that. Can you repeat that please?"
                })
                self.conversation_history.append({"role": "assistant", "content": FALL_BACK_RESPONSE_ACE})
                self.publish_robot_behavior(FALL_BACK_RESPONSE_NORMAL)

    def call_openai(self, system_prompt, user_input, model, temperature):
        """Calls the OpenAI API with formatted input and system prompt."""
        if not self.api_key:
            self.get_logger().error("OpenAI API key not found")
            return self.get_error_response()
        
        # Prepare messages for OpenAI
        messages = [{"role": "system", "content": system_prompt}] + self.conversation_history
        messages.append({"role": "user", "content": user_input})
        
        try:
            client = OpenAI(api_key=self.api_key)
            completion = client.chat.completions.create(
                model=model,
                temperature=temperature,
                messages=messages
            )
            
            response_content = completion.choices[0].message.content
            self.get_logger().info("Generated response from OpenAI")
            self.get_logger().info(f"Response content: {response_content}")
            
            return response_content
            
        except Exception as e:
            self.get_logger().error(f"Error calling OpenAI API: {str(e)}")
            return self.get_error_response()

    def get_error_response(self):
        """Returns a default error response in the expected format."""
        return json.dumps({
            "robotBehavior": [{
                "robotSpeechContent": "Sorry, I didn't get that. Can you repeat that please?",
                "robotFacialExpression": "neutral",
                "robotHeadMovement": "neutral"
            }]
        })


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Robot Behavior Generator Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()