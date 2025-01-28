#! /bin/bash

# Create user and group
docker run --rm zephyr-dev useradd -m -u "$(id -u)" -g "$(id -g)" -d "/home/$(id -un)" "$(id -un)" || true

# Run docker
docker run --rm -it -u "$(id -un)" -v "$HOME:/home/$(id -un)" \
    -v "$(pwd):/actuation" \
    -w "/actuation" \
    --net=host \
    --name zephyr-dev \
    zephyr-dev
