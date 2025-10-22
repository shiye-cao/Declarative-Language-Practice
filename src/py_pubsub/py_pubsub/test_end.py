import zmq
import json
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://localhost:12345")
print("Publisher socket bound to tcp://localhost:12345")

# message = "conversation started"
socket.send_string(f"test")

time.sleep(0.25)  # Allow time for subscribers to connect
socket.send_string("conversation ended")
print("Sent message:", "conversation ended")
