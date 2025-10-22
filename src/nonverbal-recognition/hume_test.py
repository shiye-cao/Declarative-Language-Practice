import os
import cv2
import time
import asyncio
import tempfile
from datetime import datetime
from dotenv import load_dotenv
from hume import AsyncHumeClient
from hume.expression_measurement.batch import Face, Models

# Load Hume API Key
load_dotenv()
HUME_API_KEY = os.getenv('HUME_API_KEY')

client = AsyncHumeClient(api_key=HUME_API_KEY)

async def analyze_frame(image_path):
    face_config = Face()
    models_chosen = Models(face=face_config)
    job_id = await client.expression_measurement.batch.start_inference_job(
        files=[image_path], models=models_chosen
    )
    print(f"[{datetime.now()}] Job ID: {job_id}")
    await poll_for_completion(client, job_id)
    predictions = await client.expression_measurement.batch.get_job_predictions(id=job_id)
    print(f"Predictions:\n{predictions}\n")

async def poll_for_completion(client, job_id, timeout=60):
    try:
        await asyncio.wait_for(poll_until_complete(client, job_id), timeout=timeout)
    except asyncio.TimeoutError:
        print("Timed out waiting for job to complete.")

async def poll_until_complete(client, job_id):
    delay = 1
    while True:
        await asyncio.sleep(delay)
        job_details = await client.expression_measurement.batch.get_job_details(job_id)
        status = job_details.state.status
        print(f"Job status: {status}")
        if status == "COMPLETED":
            return
        elif status == "FAILED":
            raise Exception("Job failed.")
        delay = min(delay * 2, 10)

def capture_and_send(interval=5):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open webcam.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame.")
                break

            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp_file:
                img_path = tmp_file.name
                cv2.imwrite(img_path, frame)
                print(f"[{datetime.now()}] Captured frame: {img_path}")
                asyncio.run(analyze_frame(img_path))
                os.remove(img_path)

            time.sleep(interval)

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_and_send(interval=5)  # Capture every 5 seconds
