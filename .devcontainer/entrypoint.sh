#! /bin/bash
ROS2_MSG_PATH="/opt/ros/humble/share"
AWF_MSG_PATH="/opt/autoware/share"
DEST_PATH="$PWD/actuation_packages/actuation_msgs/msg"
source actuation_packages/actuation_msgs/messages.env

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

add_include_guard_to_messages() {
    for idl_file in $(find "$DEST_PATH" -name "*.idl"); do
        filename=$(basename "$idl_file" .idl)
        filename_upper=$(echo "$filename" | tr '[:lower:]' '[:upper:]')
        
        # Create temporary file
        temp_file=$(mktemp)
        
        # Add header guard and content
        echo "#ifndef ${filename_upper}_IDL_" > "$temp_file"
        echo "#define ${filename_upper}_IDL_" >> "$temp_file"
        echo "" >> "$temp_file"
        cat "$idl_file" >> "$temp_file"
        echo "" >> "$temp_file"
        echo "#endif  // ${filename_upper}_IDL_" >> "$temp_file"
        
        # Replace original file with modified content
        mv "$temp_file" "$idl_file"
    done
}

rm -rf $DEST_PATH
mkdir -p $DEST_PATH
msg_paths=$(copy_messages)
echo -e "Copied messages to $DEST_PATH: $msg_paths"
add_include_guard_to_files

pip3 install -r zephyr/scripts/requirements-base.txt
pip3 install -r zephyr/scripts/requirements-build-test.txt

west init -l zephyr_app > /dev/null 2>&1
west update
west zephyr-export

exec "/bin/bash"