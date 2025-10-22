import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.srv import SetString
import json

class RobotFaceController(Node):
    def __init__(self):
        super().__init__('robot_face_controller')

        # ROS2 Subscriber:
        self.robot_face_subscription = self.create_subscription(
            String,  # JSON {robot_face}
            '/robot_face',  # FROM: RobotController
            self.robot_face_callback,
            10
        )
        self.speech_completed_subscription = self.create_subscription(
            String,
            '/speech_completed',  # FROM: SpeakerController
            self.speech_completed_callback,
            10
        )

        # ROS2 Service Client for setting face status
        self.set_face_status_client = self.create_client(SetString, 'set_face_status')
        while not self.set_face_status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_face_status service...')
        
        self.get_logger().info('Robot Face Controller Node initialized.')
        # Track current face to avoid redundant updates
        self.current_face = "reset"

    def robot_face_callback(self, msg):
        try:
            face_data = json.loads(msg.data)
            face_status = face_data.get("face_status", "reset")
            # Immediately update if the new status is different from the current one
            if face_status != self.current_face:
                self.current_face = face_status
                self.set_face_status(face_status)
                self.get_logger().info(f"Updated face status to: {face_status}")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid face_status message format.")

    def speech_completed_callback(self, msg):
        self.get_logger().info("Received speech completion signal.")
        self.set_face_status("reset")
        
    def set_face_status(self, status: str):
        request = SetString.Request()
        request.data = status
        future = self.set_face_status_client.call_async(request)
        future.add_done_callback(self.set_face_status_callback)

    def set_face_status_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Set 'face_status' to {response.message}")
        except Exception as e:
            self.get_logger().error(f"Failed to set 'face_status': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotFaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Robot Face Controller Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()