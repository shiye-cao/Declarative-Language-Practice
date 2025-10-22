import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import serial
import threading

class RobotHeadController(Node):
    def __init__(self):
        super().__init__('robot_head_controller')
        # Parameters for serial port configuration. These can be made configurable.
        self.serial_port = '/dev/ttyACM0'  # Adjust to match your system (e.g. COM3 on Windows)
        self.baud_rate = 9600
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Opened serial port: {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise

        # Subscribe to head command topic. The message data is the string name of the head movement.
        self.subscription = self.create_subscription(
            String,
            '/robot_head',
            self.head_command_callback,
            10
        )

        self.subscription  # Prevent unused variable warning
        # Start a background thread to read any responses from Arduino.
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    # Mapping from behavior name to the Arduino command string (with "0," prefix)
    command_mapping = {
        'lookAtUser': '0,1',
        'lookAtScreen': '0,2',
        'lookAway': '0,3',
        'nod': '0,6',
        'doubleNod': '0,7',
        'thinking': '0,10',
    }

    def head_command_callback(self, msg: String) -> None:
        """
        Callback that receives a head command string and sends the corresponding serial command.
        It looks up the command in a predefined mapping and sends it to the Arduino.
        """
        command_key = json.loads(msg.data).get("head_status", "lookAtUser")
        # command_key = self.command_mapping[head_status]
        if command_key in self.command_mapping:
            command_str = self.command_mapping[command_key] + "\n"
            self.get_logger().info(f"Sending command: {command_str.strip()}")
            try:
                self.ser.write(command_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Failed to write to serial: {e}")
        else:
            self.get_logger().error(f"Unknown command: {command_key}")

    def read_serial(self) -> None:
        """
        Continuously reads lines from the Arduino and logs them.
        This can be used to capture status messages (e.g., confirmation messages) from the Arduino.
        """
        while rclpy.ok():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f"Arduino response: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotHeadController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down head controller node.')
    finally:
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()