"""Launch the main vive_node and the corrective static map<->world transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vive_node = Node(
        package="vive_ros",
        executable="vive_node",
        name="vive_node",
        output="screen",
        parameters=[
            {"world_offset": [0.0, 0.0, 2.265]},
            {"world_yaw": 0.0},
        ],
    )

    # Corrective static transform map -> world
    # (matches the original ROS 1 launch file).
    world_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_map_tf",
        arguments=[
            "--x", "0.889",
            "--y", "0.027",
            "--z", "0.0",
            "--yaw", "0.88",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
    )

    return LaunchDescription([vive_node, world_map_tf])
