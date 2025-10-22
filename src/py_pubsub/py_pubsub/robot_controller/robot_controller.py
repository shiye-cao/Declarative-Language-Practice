#!/usr/bin/env python3
"""
Robot Controller ROS2 Node

This node orchestrates robot behaviors by managing speech output, facial expressions,
and head movements. It processes behavior sequences from the robot behavior generator
and coordinates with hardware controllers to execute synchronized robot actions.

Author: [Your Name]
Date: [Current Date]
Version: 2.1
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from utils.srv import SetString
from collections import deque
from typing import Dict, List, Optional, Tuple, Any
import threading

from std_msgs.msg import Empty
import zmq

from py_pubsub.conversation_logger import get_conversation_logger

class BehaviorItem:
    """Represents a single robot behavior with speech, face, and head components."""
    
    def __init__(self, speech_content: str, facial_expression: str = "neutral", 
                 head_movement: str = "lookAtUser", status: str = "normal"):
        """
        Initialize a behavior item.
        
        Args:
            speech_content: Text to be spoken by the robot
            facial_expression: Facial expression to display
            head_movement: Head movement/orientation to perform
            status: Special status (normal, thinking, etc.)
        """
        self.speech_content = speech_content
        self.facial_expression = facial_expression
        self.head_movement = head_movement
        self.status = status
    
    def __str__(self) -> str:
        return f"BehaviorItem(speech='{self.speech_content[:50]}...', face='{self.facial_expression}', head='{self.head_movement}', status='{self.status}')"


class RobotController(Node):
    """
    Robot Controller Node for orchestrating multi-modal robot behaviors.
    
    This node manages the execution of robot behaviors including speech synthesis,
    facial expressions, and head movements. It maintains a behavior queue and 
    ensures proper synchronization between different modalities.
    
    Subscribers:
        - /robot_behavior (String): Behavior sequences from behavior generator
        - /interruption_detected (String): Interruption signals from speech processor  
        - /speech_completed (String): Speech completion signals from speaker controller
    
    Publishers:
        - /robot_speech_content (String): Speech content for TTS system
        - /stop_audio (String): Audio stop commands for interruption handling
        - /robot_face (String): Facial expression commands for face controller
        - /robot_head (String): Head movement commands for head controller
    
    Service Clients:
        - set_speech_content (SetString): Sets current speech content in shared state
    """
    
    def __init__(self):
        """Initialize the Robot Controller node."""
        super().__init__('robot_controller')
        
        # Initialize subscribers
        self._setup_subscribers()
        
        # Initialize publishers  
        self._setup_publishers()
        
        # Initialize service clients
        self._setup_service_clients()
        
        # Initialize state variables
        self._init_state_variables()
        
        self.get_logger().info('Robot Controller Node initialized successfully')

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
            self.get_logger().info("[TextToSpeech]Received message from ZeroMQ")
            # if message.startswith("conversation started"):
            #     self.get_logger().info("Received start message from ZeroMQ")
            #     self.conversation_start_change_voice_callback(Empty())
            # elif message.startswith("initial prompt conversation started"):
            #     self.conversation_type = "initial_prompt"
            #     self.get_logger().info("Received start initial prompt conversation message from ZeroMQ")
            #     self.conversation_start_change_voice_callback(Empty())
            if message.startswith("conversation ended"):
                self.get_logger().info("Received end message from ZeroMQ")
                self.conversation_end_callback(Empty())

        except zmq.Again:
            # No message received, continue
            pass


    def conversation_end_callback(self, msg):
        """Callback for conversation end event."""
        self._handle_interruption()

    def _setup_subscribers(self) -> None:
        """Set up ROS2 subscribers."""
        self.robot_behavior_subscription = self.create_subscription(
            String,
            '/robot_behavior',
            self.robot_behavior_callback,
            10
        )
        
        self.interruption_subscription = self.create_subscription(
            String,
            '/interruption_detected',
            self.interruption_callback,
            10
        )

        self.speech_completed_subscription = self.create_subscription(
            String,
            '/speech_completed',
            self.speech_completed_callback,
            10
        )

    def _setup_publishers(self) -> None:
        """Set up ROS2 publishers."""
        self.speech_publisher = self.create_publisher(
            String,
            '/robot_speech_content',
            10
        )

        self.stop_audio_publisher = self.create_publisher(
            String,
            '/stop_audio',
            10
        )
        
        self.robot_face_publisher = self.create_publisher(
            String,
            '/robot_face',
            10
        )

        self.robot_head_publisher = self.create_publisher(
            String,
            '/robot_head',
            10
        )

    def _setup_service_clients(self) -> None:
        """Set up ROS2 service clients."""
        self.set_speech_content_client = self.create_client(SetString, 'set_speech_content')
        
        # Wait for service with timeout
        timeout_sec = 10.0
        if not self.set_speech_content_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warning('set_speech_content service not available after waiting')
        else:
            self.get_logger().info('set_speech_content service is available')

    def _init_state_variables(self) -> None:
        """Initialize node state variables."""
        self.behavior_queue: deque[BehaviorItem] = deque()
        self.is_executing = False  # Flag to prevent concurrent behavior execution
        self.current_behavior: Optional[BehaviorItem] = None
        self._lock = threading.Lock()  # Thread safety for queue operations

    def robot_behavior_callback(self, msg: String) -> None:
        """
        Process incoming robot behavior messages.
        
        Args:
            msg: ROS2 String message containing behavior JSON data
        """
        try:
            data = json.loads(msg.data)
            
            # Handle different input formats
            if isinstance(data, dict) and "robotBehavior" in data:
                # Format: {"robotBehavior": [behaviors]}
                behavior_list = data["robotBehavior"]
            elif isinstance(data, list):
                # Format: [behaviors]
                behavior_list = data
            elif isinstance(data, dict):
                # Format: single behavior dict
                behavior_list = [data]
            else:
                self.get_logger().error(f"Invalid behavior data format: {type(data)}")
                return
            
            if not behavior_list:
                self.get_logger().info("Empty behavior list received")
                return
                
            self.get_logger().info(f"Received {len(behavior_list)} behavior(s)")
            self._queue_behaviors(behavior_list)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse behavior JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing behavior message: {e}")

    def interruption_callback(self, msg: String) -> None:
        """
        Handle interruption signals from speech processing.
        
        Args:
            msg: ROS2 String message containing interruption data
        """
        try:
            data = json.loads(msg.data)
            is_interrupted = data.get("interruption_detected", False)
            
            if is_interrupted:
                self.get_logger().info("Interruption detected - stopping current behavior")
                self._handle_interruption()
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse interruption JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing interruption message: {e}")

    def speech_completed_callback(self, msg: String) -> None:
        """
        Handle speech completion signals from speaker controller.
        
        Args:
            msg: ROS2 String message indicating speech completion
        """
        try:
            with self._lock:
                self.get_logger().info("Speech completion signal received")
                self.is_executing = False
                self.current_behavior = None
                
                # Process next behavior in queue if available
                self._process_next_behavior()
                
        except Exception as e:
            self.get_logger().error(f"Error processing speech completion: {e}")

    def _queue_behaviors(self, behavior_list: List[Dict[str, Any]]) -> None:
        """
        Queue new behaviors for execution.
        
        Args:
            behavior_list: List of behavior dictionaries
        """
        with self._lock:
            queued_count = 0
            
            for behavior_data in behavior_list:
                try:
                    behavior = self._parse_behavior(behavior_data)
                    if behavior:
                        # Handle thinking behaviors immediately
                        if behavior.status == "thinking":
                            self._execute_thinking_behavior(behavior)
                        else:
                            self.behavior_queue.append(behavior)
                            queued_count += 1
                        
                except Exception as e:
                    self.get_logger().error(f"Failed to queue behavior: {e}")
            
            if queued_count > 0:
                self.get_logger().info(f"Queued {queued_count} behavior(s)")
            
            # Start processing if not already executing
            if not self.is_executing and self.behavior_queue:
                self._process_next_behavior()

    def _parse_behavior(self, behavior_data: Dict[str, Any]) -> Optional[BehaviorItem]:
        """
        Parse behavior data into a BehaviorItem.
        
        Args:
            behavior_data: Dictionary containing behavior information
            
        Returns:
            BehaviorItem if parsing successful, None otherwise
        """
        speech_content = behavior_data.get("robotSpeechContent", "")
        facial_expression = behavior_data.get("robotFacialExpression", "neutral")
        head_movement = behavior_data.get("robotHeadMovement", 
                                         behavior_data.get("robotHeadOrientation", "lookAtUser"))
        status = behavior_data.get("status", "normal")
        
        # For thinking behaviors, speech content can be empty
        if status != "thinking" and (not speech_content or not isinstance(speech_content, str)):
            self.get_logger().warning("Behavior missing or invalid speech content")
            return None
        
        # Validate facial expression and head movement are strings
        if not isinstance(facial_expression, str):
            facial_expression = "neutral"
        if not isinstance(head_movement, str):
            head_movement = "lookAtUser"
        if not isinstance(status, str):
            status = "normal"
        
        return BehaviorItem(speech_content, facial_expression, head_movement, status)

    def _execute_thinking_behavior(self, behavior: BehaviorItem) -> None:
        """
        Execute thinking behavior immediately without queuing.
        
        Args:
            behavior: BehaviorItem with thinking status
        """
        self.get_logger().info("Executing thinking behavior")
        
        # Publish thinking face and head movements
        self._publish_stop_audio()
        self._publish_robot_face("thinking")
        self._publish_robot_head("thinking")
        
        # No speech or speech completion for thinking behaviors

    def _process_next_behavior(self) -> None:
        """Process the next behavior in the queue."""
        if not self.behavior_queue:
            self.get_logger().info("All behaviors completed")
            return
        
        if self.is_executing:
            self.get_logger().debug("Already executing behavior, skipping")
            return
        
        # Get next behavior from queue
        behavior = self.behavior_queue.popleft()
        self.current_behavior = behavior
        self.is_executing = True
        
        self.get_logger().info(f"Executing behavior: {behavior}")
        
        # Execute all behavior components
        self._execute_behavior(behavior)

    def _execute_behavior(self, behavior: BehaviorItem) -> None:
        """
        Execute a single behavior item.
        
        Args:
            behavior: BehaviorItem to execute
        """
        try:
            # Publish speech content (if any)
            if behavior.speech_content:
                self._publish_speech_content(behavior.speech_content)
                
                # Set speech content in shared state
                self._set_speech_content(behavior.speech_content)
            
            # Publish facial expression
            self._publish_robot_face(behavior.facial_expression)
            
            # Publish head movement
            self._publish_robot_head(behavior.head_movement)
            
            # If no speech content, mark as completed immediately
            if not behavior.speech_content:
                with self._lock:
                    self.is_executing = False
                    self.current_behavior = None
                    self._process_next_behavior()
            
        except Exception as e:
            self.get_logger().error(f"Failed to execute behavior: {e}")
            # Reset execution state on error
            with self._lock:
                self.is_executing = False
                self.current_behavior = None

    def _handle_interruption(self) -> None:
        """Handle interruption by stopping current behavior and resetting state."""
        with self._lock:
            # Log remaining robot speech before clearing
            remaining_speech = []
            
            # Add current behavior speech if it exists and has content
            if self.current_behavior and self.current_behavior.speech_content:
                remaining_speech.append(self.current_behavior.speech_content)
            
            # Add queued behavior speech
            for behavior in self.behavior_queue:
                if behavior.speech_content:
                    remaining_speech.append(behavior.speech_content)
            
            # Log remaining speech if any
            if remaining_speech:
                self.logger.log_remaining_robot_speech(remaining_speech, node_name="robot_controller")
                self.get_logger().info(f"Logged {len(remaining_speech)} remaining speech items due to interruption")
            
            # Clear behavior queue
            cleared_count = len(self.behavior_queue)
            self.behavior_queue.clear()
            
            # Reset execution state
            self.is_executing = False
            self.current_behavior = None
            
            self.get_logger().info(f"Cleared {cleared_count} queued behavior(s)")
        
        # Send stop commands and reset robot state
        self._publish_stop_audio()
        self._publish_robot_face("reset")
        self._publish_robot_head("lookAtUser")

    def _publish_speech_content(self, speech_content: str) -> None:
        """
        Publish speech content to TTS system.
        
        Args:
            speech_content: Text to be spoken
        """
        try:
            msg = String()
            msg.data = json.dumps({"robot_speech_content": speech_content})
            self.speech_publisher.publish(msg)
            self.logger.log_robot_speech(speech_content, 'robot_controller')
            self.get_logger().info(f"Published speech: '{speech_content[:50]}{'...' if len(speech_content) > 50 else ''}'")
        except Exception as e:
            self.get_logger().error(f"Failed to publish speech content: {e}")

    def _publish_stop_audio(self) -> None:
        """Publish stop audio command."""
        try:
            msg = String()
            msg.data = json.dumps({"stop_audio": True})
            self.stop_audio_publisher.publish(msg)
            self.get_logger().info("Published stop audio command")
        except Exception as e:
            self.get_logger().error(f"Failed to publish stop audio: {e}")

    def _publish_robot_face(self, facial_expression: str) -> None:
        """
        Publish facial expression command.
        
        Args:
            facial_expression: Facial expression to display
        """
        try:
            msg = String()
            msg.data = json.dumps({"face_status": facial_expression})
            self.robot_face_publisher.publish(msg)
            self.get_logger().info(f"Published face expression: '{facial_expression}'")
        except Exception as e:
            self.get_logger().error(f"Failed to publish face expression: {e}")

    def _publish_robot_head(self, head_movement: str) -> None:
        """
        Publish head movement command.
        
        Args:
            head_movement: Head movement/orientation to perform
        """
        try:
            msg = String()
            msg.data = json.dumps({"head_status": head_movement})
            self.robot_head_publisher.publish(msg)
            self.get_logger().info(f"Published head movement: '{head_movement}'")
        except Exception as e:
            self.get_logger().error(f"Failed to publish head movement: {e}")

    def _set_speech_content(self, speech_content: str) -> None:
        """
        Set current speech content in shared state service.
        
        Args:
            speech_content: Current speech content
        """
        if not self.set_speech_content_client.service_is_ready():
            self.get_logger().warning("set_speech_content service not ready")
            return
        
        try:
            request = SetString.Request()
            request.data = speech_content
            future = self.set_speech_content_client.call_async(request)
            future.add_done_callback(self._set_speech_content_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to call set_speech_content service: {e}")

    def _set_speech_content_callback(self, future) -> None:
        """
        Handle response from set_speech_content service call.
        
        Args:
            future: Future object containing service response
        """
        try:
            response = future.result()
            if response:
                self.get_logger().debug(f"Speech content set: {response.message}")
            else:
                self.get_logger().warning("set_speech_content service call failed")
        except Exception as e:
            self.get_logger().error(f"Error in set_speech_content callback: {e}")

    def get_status(self) -> Dict[str, Any]:
        """
        Get current node status for debugging.
        
        Returns:
            Dictionary containing current node status
        """
        with self._lock:
            return {
                "is_executing": self.is_executing,
                "queue_length": len(self.behavior_queue),
                "current_behavior": str(self.current_behavior) if self.current_behavior else None
            }


def main(args=None):
    """Main function to run the Robot Controller node."""
    rclpy.init(args=args)
    
    try:
        node = RobotController()
        
        # Log initial status
        node.get_logger().info("Robot Controller started - ready to receive behaviors")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("Shutting down Robot Controller Node...")
    except Exception as e:
        print(f"Error running Robot Controller Node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()