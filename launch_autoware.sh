#!/bin/bash

xhost +

# Run docker with user mapping
docker run -it --rm -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME"/autoware_map/sample-map-planning:/root/autoware_map/sample-map-planning \
    ghcr.io/autowarefoundation/autoware:universe-20250207 /bin/bash