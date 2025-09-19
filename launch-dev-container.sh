#! /bin/bash

COLOR_YELLOW="\e[33m"
COLOR_RESET="\e[0m"

# Check if xhost is available
if command -v xhost >/dev/null 2>&1; then
    xhost +
else
    echo -e "${COLOR_YELLOW}Warning: xhost command not found on host machine. X11 forwarding may not work properly.${COLOR_RESET}"
    echo -e "${COLOR_YELLOW}----------------------------------------------------------${COLOR_RESET}"
fi

docker run --rm -it --name actuation-devcontainer \
    --network host \
    --privileged \
    -v "$HOME/.ccache:/root/.ccache" \
    -v "$(pwd):/actuation" \
    -w "/actuation" \
    -e CCACHE_DIR=/root/.ccache \
    -e CYCLONEDDS_URI="file:///actuation/cyclonedds.xml" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID=1 \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ghcr.io/oguzkaganozt/devcontainer:latest
