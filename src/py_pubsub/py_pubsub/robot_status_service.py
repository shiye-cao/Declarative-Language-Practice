import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from utils.srv import SetString

import json
import requests
import os
import asyncio
import aiohttp
import time
import datetime

import zmq


class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_status_service')
        self.is_talking_srv = self.create_service(SetString, 'set_is_talking', self.set_is_talking_callback)
        self.face_status_srv = self.create_service(SetString, 'set_face_status', self.set_face_status_callback)
        self.head_status_srv = self.create_service(SetString, 'set_head_status', self.set_head_status_callback)
        self.speech_content_srv = self.create_service(SetString, 'set_speech_content', self.set_speech_content_status_callback)
        self.speech_duration_srv = self.create_service(SetString, 'set_speech_duration', self.set_speech_duration_status_callback)

        self.get_is_talking_srv = self.create_service(Trigger, 'get_is_talking', self.get_is_talking_callback)
        self.get_face_status_srv = self.create_service(Trigger, 'get_face_status', self.get_face_status_callback)
        self.get_head_status_srv = self.create_service(Trigger, 'get_head_status', self.get_head_status_callback)
        self.get_speech_content_status_srv = self.create_service(Trigger, 'get_speech_content', self.get_speech_content_status_callback)
        self.get_speech_duration_status_srv = self.create_service(Trigger, 'get_speech_duration', self.get_speech_duration_status_callback)

        # Initialize the robot's status variables
        self.is_talking = "false"
        self.face_status = "neutral"
        self.head_status = "user"
        self.speech_content = "none"
        self.speech_duration = 0.0
        self.conversation_type = "normal"  # Default conversation type

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
            elif message.startswith("initial prompt conversation started"):
                self.conversation_type = "initial_prompt"
                self.get_logger().info("Received start initial prompt conversation message from ZeroMQ")
            elif message.startswith("conversation ended"):
                self.conversation_type = "normal"
                self.get_logger().info("Received end message from ZeroMQ")
        except zmq.Again:
            # No message received, continue
            pass
    
    def set_is_talking_callback(self, request, response):
        self.is_talking = request.data
        response.success = True
        response.message = f"is_talking updated to {self.is_talking}"
        if self.is_talking == "true":
            if self.conversation_type == "normal":
                self.send_face_request("startTalking")
        elif self.is_talking == "false":
            # Stop talking animation and return to the current face status
            self.send_face_request("reset")

        # self.get_logger().info(response.message)
        return response
    
    def send_face_request(self, expression):
        FACE_URL = "http://localhost:3000/express"
        
        try:
            data = {'expression': expression}
            response = requests.post('http://localhost:3000/express', json=data)

        except Exception as e:
            self.get_logger().error(f"Error while updating face status: {e}")

    def set_face_status_callback(self, request, response):
        new_face = request.data
        self.get_logger().info(f"Face status updating to {new_face}")

        # Only update if the face status is different
        if new_face != self.face_status:
            self.face_status = new_face
            self.send_face_request(new_face)
    
        response.success = True
        response.message = f"Face status updated to {self.face_status}"
        return response
    
    def set_speech_content_status_callback(self, request, response):
        self.speech_content = request.data
        response.success = True
        response.message = f"Speech content updated to {self.speech_content}."
        return response
    
    def set_speech_duration_status_callback(self, request, response):
        new_speech = request.data
        self.speech_duration = new_speech.get('speech_duration', self.speech_duration)
        response.success = True
        response.message = f"Speech duration updated to {self.speech_duration}."
        return response
    
    def set_head_status_callback(self, request, response):
        self.head_status = request.data
        response.success = True
        response.message = f"head_status updated to {self.head_status}"
        self.get_logger().info(response.message)
        return response
    
    def get_is_talking_callback(self, request, response):
        response.success = True
        response_data = {"is_talking": self.is_talking}
        response.message = json.dumps(response_data)
        self.get_logger().info(response.message)
        return response
    
    def get_face_status_callback(self, request, response):
        response.success = True
        response_data = {"face_status": self.face_status}
        response.message = json.dumps(response_data)
        self.get_logger().info(response.message)
        return response
    
    def get_head_status_callback(self, request, response):
        response.success = True
        response_data = {"head_status": self.head_status}
        response.message = json.dumps(response_data)
        self.get_logger().info(response.message)
        return response
    
    def get_speech_content_status_callback(self, request, response):
        response.success = True
        response_data = {"speech_content": self.speech_content}
        response.message = json.dumps(response_data)
        self.get_logger().info(response.message)
        return response
    
    def get_speech_duration_status_callback(self, request, response):
        response.success = True
        response_data = {"speech_duration": self.speech_duration}
        response.message = json.dumps(response_data)
        self.get_logger().info(response.message)
        return response
    

def main():
    rclpy.init()
    robot_status_service = RobotStatusService()
    rclpy.spin(robot_status_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()