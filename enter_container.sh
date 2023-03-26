#!/bin/bash

xhost +

docker exec -it --privileged \
    --user ${USER} \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    aerial-manipulation-melodic-container bash

xhost -