# Conversational Robot ROS2

## Overview
This project implements a conversational interaction platform for a social robot using the ROS2 Humble framework. _To be filled up..._


## Setup and Installation
1. Install ROS2 Humble on Ubuntu 22.04 by following the [ROS2 installation instructions](https://docs.ros.org/en/humble/Installation.html).
2. Clone this repository into your ROS2 workspace.
3. Setup environment variables:
```
OPENAI_KEY: Your OpenAI API key.
GOOGLE_APPLICATION_CREDENTIALS: Path to your Google Cloud Text-to-Speech credentials JSON.

export OPENAI_API_KEY="YOUR_API_KEY"
export GOOGLE_APPLICATION_CREDENTIALS="PATH_TO_YOUR_CREDENTIALS_JSON_FILE"
```
4. Navigate to the root of the project file.
```
cd ~/2026-hri-information
```
5. Set up a virtual environment (if needed) and activate it.
```
source venv/bin/activate
```
6. Activate ROS2 commands.
```
source /opt/ros/humble/setup.bash
```
7. Build packages (package name used here is "py_pubsub") and setup.
```
colcon build --packages-select py_pubsub utils
```
```
source install/setup.bash
```
*if packages are missing
sudo pip install xxx
```
rosdep install --from-paths src --ignore-src -r -y 
```
8. Run the system by using the launch file.
```
ros2 launch py_pubsub py_pubsub_launch.py
```
If you want to save the system logs, use the following format:
```
ros2 launch py_pubsub py_pubsub_launch.py 2>&1 | tee  ~/2026-hri-information/test_logs/0608_1.txt
```
You must change the file name everytime you run in order to not overwrite the previous log. The format used here is : month/day_number.txt

9. If robot's head movement is not responsive, try this command:
```
sudo usermod -a -G dialout $USER
```
9. If you use more python packages, follow the [ROS2 Using Python Packages](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html).
   You can include third-party python packages by using corresponding rosdep keys that can be checked in this [github repository](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml).


## ROS2 System Nodes
### speech_to_text_node
Handles audio input and transcribes speech into text while detecting wakewords and general speech activity using Google SpeechToText api.
* Publishers:
  * `wakeword_detected`: indicates if a wakeword (e.g., "Luna") is detected
  * `speech_detected`: indicates if speech activity is detected
  * `transcribed_audio`: publishes the transcribed audio
* Subscribers: None

### llm_interruption_handler_node
Classifies interruption types (cooperative-agreement, cooperative-assistance, cooperative-clarification, disruptive) using an LLM, and publishes results.
* Publishers:
  * `interruption_type`: classification of user interruptions
* Subscribers:
  * `wakeword_detected`: from `speech_to_text_node`
  * `speech_detected`: from `speech_to_text_node`
  * `transcribed_audio`: from `speech_to_text_node`
  * `robot_speech_content`: from `robot_behavior_node`
  * `robot_speech_timing`: from `text_to_speech_node`

### robot_behavior_node
Generates robot behaviors and responses based on classified interruptions and context.
* Publishers:
  * `robot_behavior`: JSON object describing robot gestures, facial expressions, and speech. Example robot_behavior is shown below.
```
{
  "robotSpeechContent": "Let’s collaborate closely, combining my analytical strengths with your instincts, intuition, and experience to make the best possible decisions.",
  "itemList": "null;null;null;null;null;null;null;null;null;null",
  "wordCount": "21",
  "duration": "6.8",
  "robotFacialExpression": "excited",
  "robotHeadOrientation": "nod"
}
```
* Subscribers:
  * `interruption_type`: from `llm_interruption_handler_node`

### text_to_speech_node
Synthesizes and plays the robot's speech using Google TextToSpeech api.
* Publishers:
  * `robot_play`: Signals when the robot begins and ends speech playback.
  * `robot_speech_timing`: Updates remaining and completed speech durations during playback.
* Subscribers:
  * `robot_speech_content`: from `robot_behavior_node`

### robot_face_node
Handles robot facial expressions based on its behavior or context.
* Publishers:
  * `robot_face`: current facial expression (e.g., neutral, happy, etc.)
* Subscribers:
  * `robot_behavior`: receives facial expression from `robot_behavior_node`

### robot_head_node
Handles robot facial expressions based on its behavior or context.
* Publishers:
  * `robot_head`: current head movement (e.g., lookAtUser, lookAtScreen, etc.)
* Subscribers:
  * `robot_behavior`: receives head movement from `robot_behavior_node`

### task_interface
Coordinates task-specific updates for the interation (e.g., updating task state or updating item list)
* Publishers:
  * `update_item_list`: Publishes updates to the task state (e.g., progress updates).
* Subscribers:
  * `robot_behavior`: receives updated item list from `robot_behavior_node`

### main_node
Triggers all other nodes, manages global states, and logs robot behaviors.
* Publishers:
  * `previous_robot_behavior`: JSON (for logging).
  * `storing_robot_behavior`: String (log of the robot's behavior and interactions).
* Subscribers:
  * `robot_behavior`: from  `robot_behavior_node`.
  * `robot_speech_timing`: from `text_to_speech_node`.
  * `interruption_type`: from `llm_interruption_handler_node`.# conversation-design-experimental
