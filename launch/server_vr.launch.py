"""Launch SteamVR's vrserver with the env vars expected by the lighthouse driver.

Usually you do NOT run this inside Docker (SteamVR cannot start headless / inside
common ROS images without GPU + display + Steam install). It is provided for
parity with the original ROS 1 package on a Linux desktop install.
"""

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    home = os.environ.get("HOME", "/root")
    openvr = os.environ.get("OPENVR", os.path.join(home, "libraries", "openvr"))
    steam = os.environ.get("STEAM", os.path.join(home, ".local", "share", "Steam"))
    steamvr = os.environ.get(
        "STEAMVR",
        os.path.join(home, ".steam", "steam", "steamapps", "common", "SteamVR"),
    )

    extra_lib_paths = ":".join(
        [
            os.path.join(steam, "ubuntu12_32", "steam-runtime", "amd64", "lib", "x86_64-linux-gnu"),
            os.path.join(steamvr, "bin", "linux64"),
            os.path.join(steamvr, "drivers", "lighthouse", "bin", "linux64"),
        ]
    )
    ld_library_path = ":".join(
        filter(None, [os.environ.get("LD_LIBRARY_PATH", ""), extra_lib_paths])
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("OPENVR", openvr),
            SetEnvironmentVariable("STEAM", steam),
            SetEnvironmentVariable("STEAMVR", steamvr),
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
            Node(
                package="vive_ros",
                executable="launch_servervr.sh",
                name="server_vr",
                output="screen",
            ),
        ]
    )
