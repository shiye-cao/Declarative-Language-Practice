import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import json

class HeadNodRecognitionNode(Node):
    def __init__(self):
        super().__init__('user_nod_recognition')
        
        self.user_nod = self.create_publisher(
            String,
            '/user_nod',
            10
        )
        
        # Setup video writer and capture
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('/home/intuitive-computing/Desktop/nodcontrol.avi', fourcc, 20.0, (640, 480))
        self.cap = cv2.VideoCapture(0)

        # Get the path to the haarcascade file
        package_share_directory = get_package_share_directory('py_pubsub')
        cascade_path = os.path.join(package_share_directory, 'resources', 'haarcascade_frontalface_alt.xml')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15), maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Movement thresholds
        self.max_headGestureDetector_movement = 10
        self.movement_threshold = 15
        self.gesture_threshold = 50
        self.gesture_show = 40

        # Tracking variables
        self.face_found = False
        self.x_movement = 0
        self.y_movement = 0
        self.gesture = False
        self.nod_published = False

        self.init_tracking()
        self.timer = self.create_timer(0.05, self.process_frame)

    def get_coords(self, p1):
        try:
            return int(p1[0][0][0]), int(p1[0][0][1])
        except:
            return int(p1[0][0]), int(p1[0][1])

    def init_tracking(self):
        while not self.face_found:
            ret, frame = self.cap.read()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(frame_gray, 1.3, 5)
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                self.face_found = True
                self.face_center = np.array([[[x + w / 2, y + h / 3]]], np.float32)
            cv2.imshow('image', frame)
            self.out.write(frame)
            cv2.waitKey(1)

        self.p0 = self.face_center
        self.old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        cv2.circle(frame, self.get_coords(p1), 4, (0, 0, 255), -1)
        cv2.circle(frame, self.get_coords(self.p0), 4, (255, 0, 0))

        a, b = self.get_coords(self.p0), self.get_coords(p1)
        self.x_movement += abs(a[0] - b[0])
        self.y_movement += abs(a[1] - b[1])

        if not self.gesture:
            cv2.putText(frame, f'x_movement: {self.x_movement}', (50, 50), self.font, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f'y_movement: {self.y_movement}', (50, 100), self.font, 0.8, (0, 0, 255), 2)

        if self.x_movement > self.gesture_threshold:
            self.gesture = 'No'
        if self.y_movement > self.gesture_threshold:
            self.gesture = 'Yes'

        if self.gesture and self.gesture_show > 0:
            cv2.putText(frame, f'Gesture Detected: {self.gesture}', (50, 50), self.font, 1.2, (0, 0, 255), 3)
            self.publish_gesture(self.gesture)
            self.gesture_show -= 1
        if self.gesture_show == 0:
            self.gesture = False
            self.x_movement = 0
            self.y_movement = 0
            self.gesture_show = 40
            self.nod_published = False

        self.p0 = p1
        self.old_gray = frame_gray.copy()
        self.out.write(frame)
        cv2.imshow('image', frame)
        cv2.waitKey(1)

    def publish_gesture(self, gesture):
        if gesture == "Yes" and not self.nod_published:
            msg = String()
            response_data = {
                'user_node': True
            }
            msg.data = json.dumps(response_data)
            self.user_nod.publish(msg)
            self.get_logger().info(f"User noded")
            self.nod_published = True


def main(args=None):
    rclpy.init(args=args)
    node = HeadNodRecognitionNode()
    rclpy.spin(node)
    node.cap.release()
    node.out.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()