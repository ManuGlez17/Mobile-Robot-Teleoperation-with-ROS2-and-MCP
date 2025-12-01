from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_package',
            executable='image_publisher.py',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='camera_package',
            executable='person_detector.py',
            name='person_detector',
            output='screen'
        )
    ])
