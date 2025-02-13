from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
        ),
        Node(
            package='yolo_detection',
            executable='yolo_detector',
        ),
            ]
        )