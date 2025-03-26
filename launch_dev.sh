#! /bin/bash

# Run docker with user mapping
docker run --rm -it \
    -v "$HOME/.ccache:/root/.ccache" \
    -v "$(pwd):/actuation" \
    --name zephyr-dev \
    -w "/actuation" \
    -e CCACHE_DIR=/root/.ccache \
    --privileged \
    zephyr-dev
