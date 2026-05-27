#!/bin/bash
set -e

docker build -t go2w_people_search:latest .

# Allow X11 connections from Docker
xhost +local:docker 2>/dev/null || true

docker run -it --rm \
    --net=host \
    --env="ROS_DOMAIN_ID=0" \
    --env="IGN_IP=127.0.0.1" \
    --env="DISPLAY" \
    --env="XAUTHORITY=/tmp/.Xauthority" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${XAUTHORITY:-$HOME/.Xauthority}:/tmp/.Xauthority:ro" \
    go2w_people_search:latest \
    ros2 launch aog_simulation unitree_adapter_simulation.xml

xhost -local:docker 2>/dev/null || true
