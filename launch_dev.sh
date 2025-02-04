#! /bin/bash

# Run docker with user mapping
docker run --rm -it \
    -v "$HOME:/home/$(id -un)" \
    -v "$(pwd):/actuation" \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    --name zephyr-dev \
    --user "$(id -u):$(id -g)" \
    -w "/actuation" \
    -e CCACHE_DIR=/home/"$(id -un)"/.ccache \
    --net=host \
    zephyr-dev
