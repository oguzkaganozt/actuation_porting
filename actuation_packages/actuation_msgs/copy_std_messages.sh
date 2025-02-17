#! /bin/bash
DOCKER_IMAGE="ghcr.io/autowarefoundation/autoware:universe-devel-20250207"
ROS2_MSG_PATH="/opt/ros/humble/share"
AWF_MSG_PATH="/opt/autoware/share"
SRC_PATH=""
DEST_PATH="$PWD/msg"

determine_msg_path() {
    local msg_name=$1
    if [[ "$msg_name" == *"autoware"* ]]; then
        SRC_PATH="$AWF_MSG_PATH"
    else
        SRC_PATH="$ROS2_MSG_PATH"
    fi
}

# Get messages to copy from arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 <message_name1> [message_name2 ...]"
    exit 1
fi

# Create container so that we can copy the messages
docker run --rm -d --name "autoware_container" "$DOCKER_IMAGE" tail -f /dev/null

# Convert messages into .idl format using ros2idl
for msg in "$@"; do
    msg_name=$(basename "$msg")
    determine_msg_path "$msg_name"
    docker cp "autoware_container:$SRC_PATH/$msg" "$DEST_PATH/$msg_name"
    echo "Copied $msg_name from $SRC_PATH to $DEST_PATH"
done

