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
    `# Pin EGL to the NVIDIA vendor ICD. Without this, glvnd EGL picks Mesa (dri2),` \
    `# fails ("failed to create dri2 screen"), and OGRE2 falls back to software/iGPU.` \
    `# Combined with --headless-rendering (baked in the image) this puts the gz` \
    `# server's sensor rendering on the RTX. Verify with nvidia-smi ( "ign gazebo` \
    `# server" should appear using GPU memory).` \
    --env="__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json" \
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
