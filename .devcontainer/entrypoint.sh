#! /bin/bash
DEST_PATH="$PWD/actuation_packages/actuation_msgs/msg"

add_include_guard_to_messages() {
    for idl_file in $(find "$DEST_PATH" -name "*.idl"); do
        # Check if file already has include guard
        if grep -q "^#ifndef.*_IDL_" "$idl_file"; then
            continue  # Skip this file if include guard exists
        fi

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

fix_include_paths() {
    for idl_file in $(find "$DEST_PATH" -name "*.idl"); do
        # Use sed to directly modify include lines in place
        sed -i 's|#include "[^"]*\/\([^/"]*\.idl\)"|#include "\1"|g' "$idl_file"
    done
}

add_include_guard_to_messages
fix_include_paths

pip3 install -r zephyr/scripts/requirements-base.txt
pip3 install -r zephyr/scripts/requirements-build-test.txt

west init -l actuation_autoware > /dev/null 2>&1
west update
west zephyr-export

exec "/bin/bash"