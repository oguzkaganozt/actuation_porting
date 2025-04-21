#! /bin/bash

# Run docker with user mapping
docker run --rm -it \
    -v "$HOME/.ccache:/root/.ccache" \
    -v "$(pwd):/actuation" \
    --name zephyr-dev \
    -w "/actuation" \
    -e CCACHE_DIR=/root/.ccache \
    -e CYCLONEDDS_URI="file:///actuation/cyclonedds.xml" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID=2 \
    --network host \
    zephyr-dev
