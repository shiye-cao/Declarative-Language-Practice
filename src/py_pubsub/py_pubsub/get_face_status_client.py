import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class GetFaceStatusClient(Node):
    def __init__(self):
        super().__init__('get_face_status_client')
        self.client = self.create_client(Trigger, 'get_face_status')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_face_status service...')
        self.get_logger().info('Service is available.')
        self.send_request()

    def send_request(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)
        
    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service response: {response.message}')
            else:
                self.get_logger().info('Service call failed.')
        except Exception as e:
            self.get_logger().error(f'Service call failed with error: {e}')

def main():
    rclpy.init()
    node = GetFaceStatusClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
