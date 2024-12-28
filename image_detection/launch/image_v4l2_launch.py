from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node

def generate_launch_description():

    camera = launch_ros.actions.Node(
        package="v4l2_camera", executable="v4l2_camera_node",
        parameters=[
            {"image_size": [640,480]},
        ],
    )

    image = launch_ros.actions.Node(
        package="image_detection", executable="image_detector",
    )

    return LaunchDescription(
        camera,
        image
    )