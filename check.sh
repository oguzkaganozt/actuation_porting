#!/bin/bash

./build.sh | grep -A 2 error: > build_error.log

if [ -s build_error.log ]; then
    cat build_error.log
    echo -e "\033[31mBuild failed. Please check build_error.log for details.\033[0m"
    exit 1
fi

echo -e "\033[32mBuild successful.\033[0m"
exit 0