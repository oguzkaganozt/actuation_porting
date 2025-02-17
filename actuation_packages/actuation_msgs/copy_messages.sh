#! /bin/bash
ROS2_MSG_PATH="/opt/ros/humble/share"
AWF_MSG_PATH="/opt/autoware/share"
MESSAGE_MODULES="autoware_planning_msgs autoware_control_msgs autoware_vehicle_msgs autoware_perception_msgs tier4_debug_msgs builtin_interfaces visualization_msgs tier4_planning_msgs autoware_adapi_v1_msgs geometry_msgs std_msgs"
DEST_PATH="$PWD/msg"

determine_msg_paths() {
    local msg_paths=""
    local path=""
    
    for msg_name in $@; do
        if [[ "$msg_name" == *"autoware"* || "$msg_name" == *"tier4"* ]]; then
            path="$AWF_MSG_PATH/$msg_name"
            msg_paths="$msg_paths$path "
        else
            path="$ROS2_MSG_PATH/$msg_name"
            msg_paths="$msg_paths$path "
        fi
    done
    
    echo -e "$msg_paths"
}

copy_messages() {
    local msg_paths=$(determine_msg_paths "$MESSAGE_MODULES")
    for msg_path in $msg_paths; do
        find "$msg_path/msg" -name "*.idl" -exec cp {} $DEST_PATH \;
    done
    echo -e "$msg_paths"
}

mkdir -p $DEST_PATH
msg_paths=$(copy_messages)
echo -e "Copied messages to $DEST_PATH: $msg_paths"
