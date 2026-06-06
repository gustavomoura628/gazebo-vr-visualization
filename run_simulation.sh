#!/bin/bash
set -e

docker build -t go2w_people_search:latest .

# The sim runs SERVER-ONLY (no Gazebo GUI -- see the `-s` flag baked in the Dockerfile)
# and renders sensors OFFSCREEN via EGL on the NVIDIA GPU. So it needs NO X11/display at
# all -- no DISPLAY, no /tmp/.X11-unix, no xhost. That's deliberate: the Gazebo GUI's
# GLX/X11 window is the #1 thing that breaks across machines (Wayland, locked DISPLAY,
# NVIDIA-GLX vs X mismatch -> "GLXWindow::create: wrong server or screen" crash). You
# view the robot through the WebRTC/VR bridge (run_teleop_bridge.sh), not a Gazebo window.
#
# Requirements (see README): an NVIDIA GPU + driver + nvidia-container-toolkit.
#   --gpus all                                : expose the GPU to the container
#   __EGL_VENDOR_LIBRARY_FILENAMES=...nvidia  : pin EGL to the NVIDIA ICD, else glvnd
#       picks Mesa -> "failed to create dri2 screen" -> software render / zeroed lidar.

docker run -it --rm \
    --net=host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json" \
    --env="ROS_DOMAIN_ID=0" \
    --env="IGN_IP=127.0.0.1" \
    go2w_people_search:latest \
    bash -lc '
      echo "================= PREFLIGHT (GPU / EGL / render) ================="
      echo "DISPLAY=${DISPLAY:-<unset, headless>}"
      echo "__EGL_VENDOR_LIBRARY_FILENAMES=${__EGL_VENDOR_LIBRARY_FILENAMES:-<unset>}"
      echo "--- nvidia-smi (is the GPU reachable INSIDE the container?) ---"
      nvidia-smi -L 2>&1 || echo "!! nvidia-smi FAILED -> --gpus all / nvidia-container-toolkit not working"
      echo "--- EGL vendor ICDs (need 10_nvidia.json) ---"
      ls -1 /usr/share/glvnd/egl_vendor.d/ 2>&1
      echo "--- NVIDIA EGL lib present? (need libEGL_nvidia) ---"
      ls /usr/lib/x86_64-linux-gnu/libEGL_nvidia.so* 2>&1 | head -1 || echo "!! libEGL_nvidia missing -> NVIDIA_DRIVER_CAPABILITIES must include graphics"
      echo "================================================================="
      echo "[run] launching sim (server-only, headless EGL). If you see"
      echo "[run] GLXWindow / dri2 / OGRE1 errors below, rendering is NOT on the GPU."
      exec ros2 launch aog_simulation unitree_adapter_simulation.xml
    '
