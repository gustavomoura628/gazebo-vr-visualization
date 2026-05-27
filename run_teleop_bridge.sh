#!/bin/bash
# run_teleop_bridge.sh — start the Quest teleop bridge inside the running sim container.
#
# The bridge (web_teleop/teleop_bridge.py) is baked into the image. This starts it
# in a chosen container and serves the web UI + MJPEG + control on port 8080.
# Open http://<laptop-ip>:8080 on the Quest browser (or any device on the WiFi).
#
# Usage:  ./run_teleop_bridge.sh
# Ctrl-C stops the bridge (not the sim).

set -u
IMAGE="go2w_people_search:latest"
PORT=8080

mapfile -t CANDIDATES < <(docker ps --filter "ancestor=${IMAGE}" --format '{{.Names}}')
if [ "${#CANDIDATES[@]}" -eq 0 ]; then
    echo "No container running '${IMAGE}'. Start the sim first:  ./run_simulation.sh"
    exit 1
fi

if [ "${#CANDIDATES[@]}" -eq 1 ]; then
    CONTAINER="${CANDIDATES[0]}"
    echo "Using the only running container: ${CONTAINER}"
else
    echo "Select the simulation container:"
    PS3="container # > "
    select c in "${CANDIDATES[@]}"; do
        [ -n "${c:-}" ] && { CONTAINER="$c"; break; }
        echo "Invalid selection, try again."
    done
fi

IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "Starting teleop bridge on '${CONTAINER}'."
echo "  Open in the Quest browser:  http://${IP:-<laptop-ip>}:${PORT}"
echo "  (Ctrl-C stops the bridge, not the sim.)"
echo

docker exec -it "${CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    # Unlock the Create3 base's full speed (lifts the ~0.3 m/s safe-speed throttle
    # to the ~0.46 m/s max and disables the backup limit). Harmless if it's already set.
    echo 'Setting Create3 safety_override=full...'
    for i in 1 2 3 4 5; do
        ros2 param set /motion_control safety_override full 2>&1 | grep -q successful \
            && { echo '  safety_override=full set.'; break; }
        echo '  (motion_control not ready, retrying...)'; sleep 2
    done
    exec python3 /web_teleop/teleop_bridge.py
"
