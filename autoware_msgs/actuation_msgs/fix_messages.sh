#! /bin/bash

DEST_PATH = $1

fix_include_paths() {
    for idl_file in $(find "$DEST_PATH" -name "*.idl"); do
        # Use sed to directly modify include lines in place
        sed -i 's|#include "[^"]*\/\([^/"]*\.idl\)"|#include "\1"|g' "$idl_file"
    done
}

fix_include_paths
