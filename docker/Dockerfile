# syntax=docker/dockerfile:1.6
#
# vive_ros: ROS 2 Humble + OpenVR v2.5.1
# ---------------------------------------------------------------------------
# Build the package against a self-contained OpenVR install. SteamVR itself
# is NOT installed inside the image; you typically run vrserver on the host
# and only run the ROS nodes in this container, or you mount your existing
# SteamVR installation into the container at runtime.
#
# Build:
#     docker build -t vive_ros:humble -f docker/Dockerfile .
#
# Run (with udev devices, X display, network):
#     docker compose -f docker/docker-compose.yml up vive_ros
# ---------------------------------------------------------------------------

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG OPENVR_VERSION=v2.5.1
ENV OPENVR_VERSION=${OPENVR_VERSION}
ENV OPENVR=/opt/openvr

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ca-certificates \
        pkg-config \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        libudev-dev \
        libusb-1.0-0-dev \
        libsdl2-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libglew-dev \
        libvulkan-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# OpenVR v2.5.1
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch ${OPENVR_VERSION} \
        https://github.com/ValveSoftware/openvr.git ${OPENVR} \
    && cmake -S ${OPENVR} -B ${OPENVR}/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED=ON \
        -DBUILD_UNIVERSAL=OFF \
    && cmake --build ${OPENVR}/build --parallel "$(nproc)" \
    && mkdir -p ${OPENVR}/lib/linux64 \
    && cp -v ${OPENVR}/build/libopenvr_api.so* ${OPENVR}/lib/linux64/ || true \
    && cp -v ${OPENVR}/bin/linux64/libopenvr_api.so /usr/local/lib/ || true \
    && ldconfig

# ---------------------------------------------------------------------------
# vive_ros workspace
# ---------------------------------------------------------------------------
ENV ROS_WS=/ros2_ws
WORKDIR ${ROS_WS}

# Copy package sources into the workspace.
COPY . ${ROS_WS}/src/vive_ros

# rosdep (the package.xml has been kept simple, but run rosdep anyway in case
# extra deps are added later).
RUN apt-get update \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build \
        --symlink-install \
        --cmake-args -DOPENVR=${OPENVR} -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------------
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
