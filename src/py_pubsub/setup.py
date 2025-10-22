from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the resource index for ROS 2 package discovery
        ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
        # Install the package.xml in the share directory
        ('share/' + package_name, ['package.xml']),
        ('share/py_pubsub/launch', ['launch/py_pubsub_launch.py']),
        ('share/py_pubsub/resources', ['resources/haarcascade_frontalface_alt.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shiye Cao',
    maintainer_email='scao14@jhu.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_text = py_pubsub.audio_processing.speech_to_text:main',
            'intent_classifier = py_pubsub.intent_classifier:main',
            'robot_status_service = py_pubsub.robot_status_service:main',
            'robot_controller = py_pubsub.robot_controller.robot_controller:main',
            'robot_speaker_controller = py_pubsub.robot_controller.robot_speaker_controller:main',
            'robot_face_controller = py_pubsub.robot_controller.robot_face_controller:main',
            'robot_head_controller = py_pubsub.robot_controller.robot_head_controller:main',
            'robot_behavior_generator = py_pubsub.robot_behavior_generator:main',
            'text_to_speech = py_pubsub.audio_processing.text_to_speech:main',
            'get_is_talking_client = py_pubsub.get_is_talking_client:main',
            'get_face_status_client = py_pubsub.get_face_status_client:main',
        ],
    },
)
