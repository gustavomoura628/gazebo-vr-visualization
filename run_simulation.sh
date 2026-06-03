#!/bin/bash
set -e

docker build -t go2w_people_search:latest .

# Allow X11 connections from Docker
xhost +local:docker 2>/dev/null || true

# GPU access for Ignition rendering.
# - NVIDIA (--gpus all): REQUIRED. The sim server renders sensors with OGRE2,
#   which needs a real GL3.3+ context. The Intel iGPU EGL path can't provide one
#   in-container (eglInitialize / "OpenGL 3.3 not supported"), so OGRE2 would
#   fall back to OGRE1, which silently breaks GPU lidar (all gpu_lidar -> 0).
#   The NVIDIA RTX (via nvidia-container-toolkit) gives OGRE2 its GL context.
# - /dev/dri (Intel): kept for the Gazebo GUI's GLX rendering on the X display.
DRI_DEVICE=""
[ -d /dev/dri ] && DRI_DEVICE="--device=/dev/dri:/dev/dri"

docker run -it --rm \
    --net=host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    $DRI_DEVICE \
    --env="ROS_DOMAIN_ID=0" \
    --env="IGN_IP=127.0.0.1" \
    --env="DISPLAY" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro" \
    go2w_people_search:latest \
    ros2 launch aog_simulation unitree_adapter_simulation.xml

xhost -local:docker 2>/dev/null || true
