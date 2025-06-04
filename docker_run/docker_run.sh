#!/bin/bash

# Enable GUI display forwarding
xhost +local:docker

# Run the Docker Container with GUI, AI, and ROS 2 support
docker run \
    -it \
    --rm \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v "$(pwd)":/workspace \
    -v $(pwd)/../overlay_ws/src:/overlay_ws/src \
    --name agent \
    ai_agent-spot:main \
    bash



