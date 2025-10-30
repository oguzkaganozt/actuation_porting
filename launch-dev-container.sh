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
    --privileged \
    --network host \
    -v "$HOME/.ccache:/root/.ccache" \
    -v "$(pwd):/actuation" \
    -w "/actuation" \
    -e CCACHE_DIR=/root/.ccache \
    ghcr.io/oguzkaganozt/devcontainer:latest
