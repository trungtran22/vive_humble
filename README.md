# vive_ros (ROS 2 Humble)

HTC Vive driver for ROS 2 (Humble), built on top of [Valve's OpenVR
SDK](https://github.com/ValveSoftware/openvr) (tested with `v2.5.1`).

This is a ROS 2 port of the original
[`robosavvy/vive_ros`](https://github.com/robosavvy/vive_ros) ROS 1 package.
Most of the credit belongs to robosavvy and to Jonathan Osterberg, who
contributed `vive_pose.cpp`. The package now provides:

| Executable | Purpose |
|------------|---------|
| `vive_node` | Publishes HMD / controllers / trackers / lighthouses on `tf` and on `/vive/...` topics. Exposes a `/vive/set_origin` service and a `/vive/set_feedback` rumble subscription. |
| `vive_pose` | Standalone publisher of left/right controller pose, trigger, menu/grip buttons and touchpad axes. |

> **Note on the optional `USE_IMAGE` branch.** The original ROS 1 code shipped a
> compile-time flag (`USE_IMAGE`) that pulled in OpenVR's `hellovr_opengl` /
> `hellovr_vulkan` sample renderer, plus `cv_bridge` / OpenCV, to draw ROS
> stereo images into the HMD. That branch was disabled by default upstream and
> has been dropped in this ROS 2 port to keep the dependency surface
> small. Open an issue if you need it back.

## Topics & frames

`vive_node` publishes:

- `/vive/hmd_pose` — `geometry_msgs/PoseStamped`
- `/vive/controller1_pose`, `/vive/controller2_pose` — `geometry_msgs/PoseStamped`
- `/vive/controller_<serial>/joy` — `sensor_msgs/Joy` (one per controller)
- `tf`: `world` → `world_vive` → {`hmd`, `controller_<sn>`, `tracker_<sn>`, `lighthouse_<sn>`}

`vive_node` subscribes to:

- `/vive/set_feedback` — `sensor_msgs/JoyFeedback` (rumble)

`vive_node` services:

- `/vive/set_origin` — `std_srvs/Empty` (calibrates the world offset/yaw using controller 1's current pose).

`vive_pose` publishes:

- `/vive/controller/{left,right}/pose` — `geometry_msgs/PoseStamped`
- `/vive/controller/{left,right}/trigger` — `std_msgs/Float32`
- `/vive/controller/{left,right}/buttons/{menu,grip}` — `std_msgs/Int32`
- `/vive/controller/{left,right}/touchpad` — `std_msgs/Float32MultiArray`

## Quick start with Docker

The provided Dockerfile builds OpenVR `v2.5.1` from source and the workspace on
top of `osrf/ros:humble-desktop`.

```bash
# from the repo root
docker build -t vive_ros:humble -f docker/Dockerfile .

# allow the container to reach your X server (one-shot)
xhost +local:root

# launch the main node (mounts /dev, /tmp/.X11-unix and your Steam install)
docker compose -f docker/docker-compose.yml up vive_ros
```

Inside the container the workspace is at `/ros2_ws`, OpenVR at `/opt/openvr`,
and the entrypoint sources both `/opt/ros/humble/setup.bash` and the overlay.
You can drop into a shell with:

```bash
docker compose -f docker/docker-compose.yml run --rm vive_ros bash
# then
ros2 launch vive_ros vive.launch.py        # main node + corrective TF
ros2 launch vive_ros controller_pose.launch.py   # standalone pose publisher
ros2 run vive_ros vive_node
ros2 run vive_ros vive_pose
```

> **About SteamVR.** The container only ships the OpenVR SDK; it does **not**
> bundle Steam or SteamVR. To actually talk to the headset, SteamVR (i.e.
> `vrserver`) needs to be running. The docker-compose file mounts your host's
> Steam install at `~/.steam` and `~/.local/share/Steam` into the container
> so the OpenVR client can reach `vrserver`. The simplest setup is to start
> SteamVR on the host and run the ROS nodes inside the container.

## Native (host) install

If you prefer to build natively against an existing OpenVR checkout:

```bash
# 1. Build OpenVR v2.5.1
mkdir -p ~/libraries && cd ~/libraries
git clone https://github.com/ValveSoftware/openvr.git -b v2.5.1
cmake -S openvr -B openvr/build -DCMAKE_BUILD_TYPE=Release
cmake --build openvr/build -j

# 2. Build the workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s <path-to-this-repo> vive_ros
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
    --cmake-args -DOPENVR=$HOME/libraries/openvr -DCMAKE_BUILD_TYPE=Release

# 3. Run
source install/setup.bash
ros2 launch vive_ros vive.launch.py
```

`OPENVR` can also be passed as an environment variable.

### Hardware permissions

```bash
sudo cp 60-HTC-Vive-perms.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Parameters

`vive_node`:

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `world_offset` | `double[3]` | `[0, 0, 0]` | Offset applied to the corrective `world → world_vive` transform. The `vive.launch.py` example sets this to `[0, 0, 2.265]`. |
| `world_yaw` | `double` | `0.0` | Yaw component of the corrective transform. The `set_origin` service overwrites this from controller 1's current pose. |

`vive_pose`:

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `frame_id` | `string` | `"base"` | `header.frame_id` written into the published controller poses. |

## Credits

- Original ROS 1 implementation: [robosavvy/vive_ros](https://github.com/robosavvy/vive_ros) by Vitor Matos.
- `vive_pose` contribution: Jonathan Osterberg.
- ROS 2 / Docker port: this repository.
