"""Launch only the standalone vive_pose publisher (left/right controller poses)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vive_ros",
                executable="vive_pose",
                name="vive_pose",
                output="screen",
            ),
        ]
    )
