#!/bin/bash
set -e

docker build -t go2w_people_search:latest .

# Allow X11 connections from Docker
xhost +local:docker 2>/dev/null || true

# GPU passthrough for Ignition rendering. WITHOUT this, ignition's ogre2 engine
# cannot create a GL context inside the container ("libGL error: failed to open
# /dev/dri/cardN"), which silently breaks BOTH the GUI *and* the server-side
# camera sensor rendering (no frames on /oakd/rgb/preview/image_raw). Mounting
# /dev/dri lets ogre2 use the host GPU. If your machine has no /dev/dri this
# line will need adjusting (NVIDIA EGL or LIBGL_ALWAYS_SOFTWARE fallback).
DRI_DEVICE=""
[ -d /dev/dri ] && DRI_DEVICE="--device=/dev/dri:/dev/dri"

docker run -it --rm \
    --net=host \
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
