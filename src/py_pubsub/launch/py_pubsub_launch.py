import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot status service
        Node(
            package='py_pubsub',
            executable='robot_status_service',
            name='robot_status_service',
            output='screen'
        ),
        # Speech to text node
        Node(
            package='py_pubsub',
            executable='speech_to_text',
            name='speech_to_text',
            output='screen'
        ),
        # Interruption detector node
        Node(
            package='py_pubsub',
            executable='intent_classifier',
            name='intent_classifier',
            output='screen'
        ),
        # Robot behavior generator node
        Node(
            package='py_pubsub',
            executable='robot_behavior_generator',
            name='robot_behavior_generator',
            output='screen'
        ),
        # Robot controller node
        Node(
            package='py_pubsub',
            executable='robot_controller',
            name='robot_controller',
            output='screen'
        ),
        # Text to speech node
        Node(
            package='py_pubsub',
            executable='text_to_speech',
            name='text_to_speech',
            output='screen'
        ),
        # Speaker controller node
        Node(
            package='py_pubsub',
            executable='robot_speaker_controller',
            name='robot_speaker_controller',
            output='screen'
        ),
        # Face controller node
        Node(
            package='py_pubsub',
            executable='robot_face_controller',
            name='robot_face_controller',
            output='screen'
        ),
        # Head controller node
        Node(
            package='py_pubsub',
            executable='robot_head_controller',
            name='robot_head_controller',
            output='screen'
        ),
    ])
