#! /bin/bash
MSG_PATH="/home/oguzkaganozt/codebase/message-porting/share"
DEST_PATH="$PWD/msg"

# Get messages to copy from arguments
if [ $# -eq 0 ]; then
    echo "Usage: $0 <message_name1> [message_name2 ...]"
    exit 1
fi

# Copy messages from autoware_msgs to actuation_msgs
for msg in "$@"; do
    msg_name=$(basename "$msg")
    cp -r "$MSG_PATH/$msg" "$DEST_PATH/$msg_name"
    echo "Copied $msg_name"
done

