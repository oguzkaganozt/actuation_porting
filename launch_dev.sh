#! /bin/bash

xhost +

docker run --rm -it --name zephyr-dev \
    --network host \
    -v "$HOME/.ccache:/root/.ccache" \
    -v "$(pwd):/actuation" \
    -w "/actuation" \
    -e CCACHE_DIR=/root/.ccache \
    -e CYCLONEDDS_URI="file:///actuation/cyclonedds.xml" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID=2 \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    zephyr-dev
