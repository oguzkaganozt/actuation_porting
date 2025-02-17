#! /bin/bash
MSG_PATH="/home/oguzkaganozt/codebase/message-porting/common_interfaces"
DEST_PATH="$PWD/msg"

# Get messages to copy from arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 <message_name1> [message_name2 ...]"
    exit 1
fi

# Convert messages into .idl format using ros2idl
for msg in "$@"; do
    msg_name=$(basename "$msg")
    ros2idl convert "$MSG_PATH/$msg" "$DEST_PATH/$msg_name"
    echo "Converted and copied $msg_name"
done

