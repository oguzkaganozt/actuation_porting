#! /bin/bash

# Check if xhost is available
if command -v xhost >/dev/null 2>&1; then
    xhost +
else
    echo "Warning: xhost command not found. X11 forwarding may not work properly."
fi

docker run --rm -it --name actuation-devcontainer \
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
    ghcr.io/oguzkaganozt/devcontainer:latest
