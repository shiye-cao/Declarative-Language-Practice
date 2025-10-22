import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import requests
import os
import json
import aiohttp
import asyncio
import re

class IntentClassifier(Node):
    def __init__(self):
        super().__init__('intent_classifier')
        # Get API credentials
        self.declare_parameter('openai_api_key', os.environ.get('OPENAI_API_KEY'))
        self.declare_parameter('openai_api_url', 'https://api.openai.com/v1/chat/completions')
        self.api_key = self.get_parameter('openai_api_key').get_parameter_value().string_value
        self.api_url = self.get_parameter('openai_api_url').get_parameter_value().string_value
        
        # ROS2 Subscribers
        self.user_interruption_subscription = self.create_subscription(
            String, # JSON: {is_wakeword, interruption_time, transcribed_audio, interruption_detected}
            '/user_interruption',  # FROM: speech_to_text
            self.user_interruption_callback,
            10
        )

        self.robot_speech_subscription = self.create_subscription(
            String, # JSON: {robot_speech_content}
            '/robot_speech_content', # FROM: robot_controller
            self.robot_speech_callback, 
            10
        )

        # ROS2 Publisher
        self.user_intent_publisher = self.create_publisher(
            String, # JSON {user_intent_type, user_speech, prev_robot_behavior, prev_robot_speech}
            '/user_intent', # TO: robot_behavior_generator
            10
        )

        self.interruption_detected_publisher = self.create_publisher(
            String,  # JSON {interruption_detected}
            '/interruption_detected',  # TO: robot_controller
            10
        )

        # Internal states
        self.is_wakeword_detected = False
        self.user_speech = ""
        self.is_interruption = False
        self.interruption_type = "not_interruption"
        self.robot_speech = ""
        self.user_interrupted_at = ""
        self.robot_speech_duration_left = 0.0
        self.robot_speech_duration_completed = 0.0

        # Backchanneling patterns
        self.backchanneling_patterns = [
            r'\b(uhm|umm|uh|um)\b',
            r'\b(hmm|hm|mm)\b', 
            r'\b(yup|yeah|yes|yep)\b',
            r'\b(okay|ok|alright)\b',
            r'\b(right|sure|exactly)\b',
            r'\b(wow|oh|ah)\b',
            r'\b(i see|got it|gotcha)\b',
            r'\b(mhm|mmhmm)\b'
        ]

        self.system_prompt = """
        You are designed to categorize user interruptions in a two-person conversation between a user and a voice assistant.
        You will be provided with a conversation transcript and the time when the user interruption occurs.
        The speech-to-text output may only contain the first part of the user utterance.
        The voice assistant's text-to-speech rate is 185 words per minute. Using this information, you must determine the nature of any interruptions.
        There are five main types of interruptions:
            1. Disruptive Interruption: The user seeks to take the turn from the speaker. If the user speech content contains "Luna" or "stop", it must be considered disruptive. Subtypes of disruptive interruption include:
                - Disagreement: The user disagrees and immediately expresses their own opinion.
                - Floor Taking: The user takes over the conversation, continuing or shifting the topic.
                - Topic Change: The user introduces an entirely new topic, breaking away from the current one.
                - Tangentialization: The user summarizes the speaker's point to conclude the topic, typically to prevent further information.
            2. Cooperative clarification: The user asks for clarification or requests additional information to better understand the speaker's message. Sentences containing how, why, what is in this category.
            3. Cooperative agreement: The user expresses agreement, support, understanding, or compliance with the speaker.
            4. Cooperative assistance: The user offers help by providing a word, phrase, or idea to complete the speaker's turn. If the user's interruption has similar contextual meaning with the robot speech, it must be considered cooperative assistance.
        Your task is to classify each user interruption into one of these 4 categories based on the transcript and conversation context. Your output must be one of the following: "disruptive"; "cooperative-clarification"; "cooperative-agreement"; "cooperative-assistance".
        Do not provide justifications.
        """

        self.get_logger().info("Intent Classifier Node initialized")
 
    def user_interruption_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.user_speech = data.get("transcribed_audio", "")
            self.is_interruption = data.get("interruption_detected", False)
            self.is_wakeword_detected = data.get("is_wakeword", False)
            self.user_interrupted_at = data.get("interruption_time", "")
            
            self.get_logger().info(f"User interruption received: {self.user_speech}")
            self.process_interruption()
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse user interruption JSON: {e}")

    def robot_speech_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.robot_speech = data.get("robot_speech_content", "")
            self.get_logger().info(f"Robot speech received: {self.robot_speech}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse robot speech JSON: {e}")
            self.robot_speech = msg.data

    def is_backchanneling(self, text):
        """Check if the user speech is just backchanneling (should not interrupt robot)."""
        text_lower = text.lower().strip()
        
        # Check if the entire text matches backchanneling patterns
        for pattern in self.backchanneling_patterns:
            if re.fullmatch(pattern, text_lower):
                return True
        
        # Check for very short responses that are likely backchanneling
        words = text_lower.split()
        if len(words) <= 2:
            for word in words:
                for pattern in self.backchanneling_patterns:
                    if re.match(pattern, word):
                        return True
        
        return False

    def process_interruption(self):
        """Process the interruption and decide whether to stop the robot."""
        
        # Check if this is backchanneling - if so, don't interrupt
        if self.is_backchanneling(self.user_speech):
            self.get_logger().info(f"Detected backchanneling: '{self.user_speech}' - not interrupting robot")
            # Still classify intent but don't stop robot
            user_intent_type = "cooperative-agreement"  # Most backchanneling is agreement
            self.publish_interruption_detected(False)  # Explicitly don't interrupt
            self.publish_intent_result(user_intent_type)
            return
        
        # For wakewords, always interrupt and STOP (don't generate new behavior)
        if self.is_wakeword_detected:
            self.get_logger().info(f"Wakeword detected: '{self.user_speech}' - stopping robot completely")
            self.publish_interruption_detected(True)
            user_intent_type = "disruptive"  # Wakewords are always disruptive
            # For wakewords, we want to STOP only, not generate new behavior
            self.publish_intent_result_wakeword_stop(user_intent_type)
            return
        
        # For other interruptions, classify intent and decide
        self.get_logger().info('Classifying intent for interruption...')
        user_intent_type = self.classify_intent_sync()
        
        # Stop robot for disruptive interruptions and cooperative clarification
        if user_intent_type == "disruptive":
            self.get_logger().info("Disruptive interruption detected - stopping robot and listening")
            self.publish_interruption_detected(True)
            self.publish_intent_result(user_intent_type)
        elif user_intent_type == "cooperative-clarification":
            self.get_logger().info("Cooperative clarification detected - stopping robot and listening")
            self.publish_interruption_detected(True)
            self.publish_intent_result(user_intent_type)
        elif user_intent_type == "cooperative-agreement":
            self.get_logger().info("Cooperative agreement - robot continues, no stopping")
            self.publish_interruption_detected(False)
            self.publish_intent_result(user_intent_type)
        elif user_intent_type == "cooperative-assistance":
            self.get_logger().info("Cooperative assistance - robot continues, no stopping")
            self.publish_interruption_detected(False)
            self.publish_intent_result(user_intent_type)
        else:
            # Handle error cases
            self.get_logger().warning(f"Unknown intent type: {user_intent_type} - defaulting to stop robot")
            self.publish_interruption_detected(True)
            self.publish_intent_result("disruptive")  # Default to disruptive for safety
        # else:
        #     # Not an interruption, just regular speech
        #     self.get_logger().info('intent_classifer didnt correctly classify, will be dealed as cooperative clarification')
        #     self.publish_interruption_detected(True)
        #     self.publish_intent_result("cooperative-clarification")

    def publish_intent_result_wakeword_stop(self, user_intent_type):
        """Special publisher for wakeword events - signals STOP ONLY, no new behavior generation."""
        msg = String()
        response_data = {
            'user_intent_type': user_intent_type,
            'user_speech': self.user_speech,
            'prev_robot_behavior': self.robot_speech,
            'is_wakeword': True,
            'is_stop_only': True,  # Special flag to indicate stop without new behavior
            'interruption_time': self.user_interrupted_at
        }
        msg.data = json.dumps(response_data)
        self.get_logger().info(f"Published STOP-ONLY wakeword intent: {user_intent_type}")
        self.user_intent_publisher.publish(msg)

    def publish_intent_result(self, user_intent_type):
        """Regular intent publisher for non-wakeword events."""
        msg = String()
        response_data = {
            'user_intent_type': user_intent_type,
            'user_speech': self.user_speech,
            'prev_robot_behavior': self.robot_speech,
            'is_wakeword': self.is_wakeword_detected,
            'is_stop_only': False,  # Normal processing, can generate new behavior
            'interruption_time': self.user_interrupted_at
        }
        msg.data = json.dumps(response_data)
        self.get_logger().info(f"Classified intent type: {user_intent_type}")
        self.user_intent_publisher.publish(msg)


    def publish_interruption_detected(self, should_interrupt):
        """Publish interruption signal to robot controller."""
        msg = String()
        response_data = {
            'interruption_detected': should_interrupt
        }
        msg.data = json.dumps(response_data)
        self.interruption_detected_publisher.publish(msg)
        self.get_logger().info(f"Published interruption signal: {should_interrupt}")

    def classify_intent_sync(self):
        data = {
            "model": "gpt-4o-mini-2024-07-18",
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {
                    "role": "user",
                    "content": f"assistant: {self.robot_speech}\nuser: {self.user_speech}\nuser interrupted at: {self.user_interrupted_at}"
                }
            ],
            "temperature": 0.5
        }
        
        try:
            response = requests.post(self.api_url,
                                    headers={"Authorization": f"Bearer {self.api_key}"},
                                    json=data)
            if response.status_code == 200:
                result = response.json()
                return result['choices'][0]['message']['content'].strip()
            else:
                self.get_logger().error(f"OpenAI API error: {response.status_code} - {response.text}")
                return "error"
        except Exception as e:
            self.get_logger().error(f"Error while calling OpenAI API: {str(e)}")
            return "error"

    async def run(self):
        while rclpy.ok():
            await asyncio.sleep(0.1)

    def destroy_node(self):
        # stop the event loop
        if hasattr(self, 'loop'):
            self.loop.call_soon_threadsafe(self.loop.stop())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IntentClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IntentClassifier Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()