"""
Robot Speaker Controller Node

This node is responsible for handling speaker playback in the social robot system.
It receives audio messages (as byte arrays), plays them through the robot's speakers,
handles interruptions (stop commands), and notifies the system when playback completes.

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String
import json
import time
import threading
from pydub import AudioSegment
import io
import pygame
from utils.srv import SetString
from typing import Optional


class AudioManager:
    """
    Handles initialization of the audio system, audio playback, stopping playback,
    and cleanup of audio resources using pygame and pydub.
    """
    def __init__(self, logger):
        self.logger = logger
        self.is_initialized = False
        self.is_playing = False
        self.should_stop = False
        self._init_pygame()
    
    def _init_pygame(self):
        """Attempt to initialize the pygame mixer for audio output. Raises on failure."""
        try:
            pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=512)
            pygame.mixer.init()
            self.is_initialized = True
            self.logger.info("Pygame mixer initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize pygame mixer: {e}")
            raise
    
    def play_audio(self, audio_data: bytes) -> bool:
        """
        Plays the provided WAV audio data through the speakers, blocking until done
        or until playback is interrupted.

        Args:
            audio_data: Raw bytes of a WAV-format audio file.

        Returns:
            True if playback finished naturally, False if stopped early.
        """
        if not self.is_initialized:
            self.logger.error("Audio manager not initialized")
            return False
        
        try:
            # Convert audio data to AudioSegment
            audio = AudioSegment.from_file(io.BytesIO(audio_data), format="wav")

            audio_pygame = io.BytesIO()
            audio.export(audio_pygame, format="wav")
            audio_pygame.seek(0)
            
            # Begin playback
            pygame.mixer.music.load(audio_pygame)
            pygame.mixer.music.play()
            
            self.is_playing = True
            self.should_stop = False
            
            # Loop until done playing, or an interruption signal is received
            while pygame.mixer.music.get_busy() and not self.should_stop:
                time.sleep(0.01)  # Small sleep to prevent CPU spinning

            # Handle whether we exited loop from natural finish or a force stop
            if self.should_stop:
                pygame.mixer.music.stop()
                self.logger.info("Audio playback stopped by request")
                return False
            else:
                self.logger.info("Audio playback completed naturally")
                return True
                
        except Exception as e:
            self.logger.error(f"Error playing audio: {e}")
            return False
        finally:
            self.is_playing = False
            self.should_stop = False
    
    def stop_audio(self):
        """Immediately stop audio playback."""
        if self.is_playing:
            self.should_stop = True
            pygame.mixer.music.stop()
            self.logger.info("Audio stop requested")
    
    def cleanup(self):
        """Clean up pygame resources."""
        if self.is_initialized:
            pygame.mixer.quit()


class RobotSpeakerControllerNode(Node):
    """
    Robot Speaker Controller Node for handling audio playback for the robot's speech system.
    
    Subscribers:
        - /text_to_speech_output (UInt8MultiArray): Audio data from TTS
        - /stop_audio (String): Commands to stop current audio playback
    
    Publishers:
        - /speech_completed (String): Signals when speech playback is complete
    
    Service Clients:
        - set_is_talking: updates robot's talking status
    """
    
    def __init__(self):
        super().__init__('robot_speaker_controller')
        
        self.audio_manager = AudioManager(self.get_logger())
        
        # State tracking
        self.speech_start_time = 0.0
        self.playback_thread: Optional[threading.Thread] = None
        
        # Setup ROS2 components
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_service_clients()
        
        self.get_logger().info("Enhanced Speaker Controller Node initialized successfully")
    
    def _setup_subscriptions(self):
        """Initialize ROS2 subscriptions."""
        self.audio_input_subscription = self.create_subscription(
            UInt8MultiArray,
            '/text_to_speech_output',
            self.audio_callback,
            10
        )
        
        self.stop_audio_subscription = self.create_subscription(
            String,
            '/stop_audio',
            self.stop_audio_callback,
            10
        )
    
    def _setup_publishers(self):
        """Initialize ROS2 publishers."""
        self.speech_completed_publisher = self.create_publisher(
            String,
            '/speech_completed',
            10
        )
    
    def _setup_service_clients(self):
        """Initialize ROS2 service clients."""
        # Service for setting is_talking status
        self.set_is_talking_client = self.create_client(SetString, 'set_is_talking')
        if not self.set_is_talking_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('set_is_talking service not available')
    
    def audio_callback(self, msg):
        """
        Handle incoming audio data for playback.
        
        Args:
            msg: UInt8MultiArray containing audio data
        """
        # Stop any currently playing audio
        if self.playback_thread and self.playback_thread.is_alive():
            self.audio_manager.stop_audio()
            self.playback_thread.join(timeout=1.0)  # Wait for thread to finish
        
        # Convert message data to bytes
        audio_data = bytes(msg.data)
        
        # Start new playback in separate thread
        self.playback_thread = threading.Thread(
            target=self._play_audio_threaded,
            args=(audio_data,),
            daemon=True
        )
        self.playback_thread.start()
    
    def _play_audio_threaded(self, audio_data: bytes):
        """
        Play audio in a separate thread with proper state management.
        
        Args:
            audio_data: Raw audio data to play
        """
        try:
            # Set robot as talking
            self._set_is_talking("true")
            
            # Record start time
            self.speech_start_time = time.time()
            
            # Play audio (this will block until complete or stopped)
            completed_naturally = self.audio_manager.play_audio(audio_data)
            
            # Publish completion event only if speech completed naturally
            if completed_naturally:
                self._publish_speech_completed()
            
        except Exception as e:
            self.get_logger().error(f"Error in audio playback thread: {e}")
        finally:
            # Always set robot as not talking when done
            self._set_is_talking("false")
    
    def stop_audio_callback(self, msg):
        """
        Handle stop audio commands.
        
        Args:
            msg: String message with stop command
        """
        self.get_logger().info("Received stop audio command")
        self.audio_manager.stop_audio()
    
    def _set_is_talking(self, status: str):
        """
        Update the robot's talking status via service call.
        
        Args:
            status: "true" or "false" string
        """
        if not self.set_is_talking_client.service_is_ready():
            self.get_logger().warning("set_is_talking service not ready")
            return
        
        request = SetString.Request()
        request.data = status
        
        future = self.set_is_talking_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_service_response(f, "is_talking", status)
        )
    
    def _handle_service_response(self, future, service_name: str, value: str):
        """
        Handle service call responses.
        
        Args:
            future: The future object from the service call
            service_name: Name of the service for logging
            value: Value that was set
        """
        try:
            response = future.result()
            self.get_logger().debug(f"Set {service_name} to {value}: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to set {service_name}: {e}")
    
    def _publish_speech_completed(self):
        """Publish speech completion event."""
        msg = String()
        msg.data = "speech_completed"
        self.speech_completed_publisher.publish(msg)
        self.get_logger().info("Published speech completion event")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        # Stop any ongoing audio
        if self.audio_manager:
            self.audio_manager.stop_audio()
        
        # Wait for playback thread to finish
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=2.0)
        
        # Clean up audio manager
        if self.audio_manager:
            self.audio_manager.cleanup()
        
        # Call parent cleanup
        super().destroy_node()


def main(args=None):
    """Main function to run the Robot Speaker Controller node."""
    rclpy.init(args=args)
    
    node = None
    try:
        node = RobotSpeakerControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down Robot Speaker Controller...")
    except Exception as e:
        print(f"Error running Robot Speaker Controller: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()