#!/bin/bash
# view_camera.sh — open rqt_image_view on the sim camera, inside a chosen container.
#
# The sim's ROS graph lives inside the docker container (separate IPC namespace),
# so host-side rqt can't see the topics over Fast DDS. Easiest reliable path is to
# run rqt *inside* the container, which already has X11 wired up by run_simulation.sh.
#
# Usage:
#   ./view_camera.sh                         # defaults to /oakd/rgb/preview/image_raw
#   ./view_camera.sh /oakd/rgb/preview/depth # view a different image topic
#
# If more than one container is running you'll get a numbered menu to pick from.

set -u

TOPIC="${1:-/oakd/rgb/preview/image_raw}"
IMAGE="go2w_people_search:latest"

# 1) Gather candidate containers — prefer the go2w image, fall back to all running.
mapfile -t CANDIDATES < <(docker ps --filter "ancestor=${IMAGE}" --format '{{.Names}}')
if [ "${#CANDIDATES[@]}" -eq 0 ]; then
    echo "No container running '${IMAGE}'. Showing all running containers instead."
    mapfile -t CANDIDATES < <(docker ps --format '{{.Names}}')
fi

if [ "${#CANDIDATES[@]}" -eq 0 ]; then
    echo "No running containers found. Start the sim first:  ./run_simulation.sh"
    exit 1
fi

# 2) Pick the container.
if [ "${#CANDIDATES[@]}" -eq 1 ]; then
    CONTAINER="${CANDIDATES[0]}"
    echo "Using the only running container: ${CONTAINER}"
else
    echo "Select the simulation container:"
    PS3="container # > "
    select c in "${CANDIDATES[@]}"; do
        if [ -n "${c:-}" ]; then
            CONTAINER="$c"
            break
        fi
        echo "Invalid selection, try again."
    done
fi

echo "Opening rqt_image_view on '${CONTAINER}'  (topic: ${TOPIC})"

# 3) Inside the container: source ROS, install the plugin on first run, launch.
#    Container runs as root (workspace stage), so no sudo needed.
docker exec -it "${CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash 2>/dev/null || true
    if ! ros2 pkg prefix rqt_image_view >/dev/null 2>&1; then
        echo '>>> rqt_image_view not installed in this container — installing (first run only)...'
        apt-get update && apt-get install -y ros-humble-rqt-image-view
    fi
    exec ros2 run rqt_image_view rqt_image_view '${TOPIC}'
"
