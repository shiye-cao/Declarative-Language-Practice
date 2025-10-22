import os
import cv2
import time
import asyncio
import base64
import numpy as np
from dotenv import load_dotenv
from hume import AsyncHumeClient
from hume.expression_measurement.stream import Config
from hume.expression_measurement.stream.socket_client import StreamConnectOptions
from hume.expression_measurement.stream.types import StreamFace

# Load Hume API key
load_dotenv()
HUME_API_KEY = os.getenv("HUME_API_KEY")

# Streaming config with Face model enabled
model_config = Config(face=StreamFace())
stream_options = StreamConnectOptions(config=model_config)

async def stream_webcam():
    # Initialize Hume Streaming client
    client = AsyncHumeClient(api_key=HUME_API_KEY)

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot access webcam.")
        return

    async with client.expression_measurement.stream.connect(options=stream_options) as socket:
        print("Connected to Hume Streaming API.")
        try:
            while True:
                # Capture frame
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read webcam frame.")
                    break

                # Encode frame to JPEG and then base64
                _, buffer = cv2.imencode(".jpg", frame)
                img_base64 = base64.b64encode(buffer).decode("utf-8")

                # Send frame and await prediction
                result = await socket.send_base64(img_base64)
                print("Expression Result:")
                print(result)

                # Control frame rate (every 0.5s ~ 2 FPS)
                await asyncio.sleep(0.5)

        finally:
            cap.release()
            print("Webcam released.")

if __name__ == "__main__":
    asyncio.run(stream_webcam())
