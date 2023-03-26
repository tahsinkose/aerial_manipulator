#!/bin/bash
xhost +

docker run -it --privileged \
    --user ${USER} \
    -v ${PWD}:/quad_manip_ws/src/aerial_manipulator:rw \
    -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --workdir="/quad_manip_ws" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -p 14556:14556/udp \
    --gpus all \
    --workdir=/quad_manip_ws/src/aerial_manipulator/PX4-Autopilot \
    --name=aerial-manipulation-melodic-container aerial-manipulation-melodic bash

xhost -