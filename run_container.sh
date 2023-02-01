#!/bin/bash
xhost +

export UID=$(id -u)
export GID=$(id -g)
docker run -it --privileged \
    --user $UID:$GID \
    -v /home/tahsin/aerial_manipulation/quad_manip_ws:/quad_manip_ws:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --workdir="/quad_manip_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -p 14556:14556/udp \
    --runtime=nvidia \
    --gpus all \
    --workdir=/quad_manip_ws/src/aerial_manipulator/PX4-Autopilot \
    --name=aerial-manipulation-melodic-container aerial-manipulation-melodic bash

xhost -