#! /bin/bash
# shellcheck disable=SC1091

COLOR_BLUE="\e[34m"
COLOR_GREEN="\e[32m"
COLOR_RESET="\e[0m"

source "/opt/ros/humble/setup.bash"
source "/opt/autoware/setup.bash"

echo -e "${COLOR_BLUE}Checking Zephyr dependencies...${COLOR_RESET}"
pip3 install -r zephyr/scripts/requirements-base.txt > /dev/null 2>&1
pip3 install -r zephyr/scripts/requirements-build-test.txt > /dev/null 2>&1

echo -e "${COLOR_BLUE}Initializing and updating Zephyr project...${COLOR_RESET}"
west init -l actuation_module > /dev/null 2>&1
west update > /dev/null 2>&1

echo -e "${COLOR_BLUE}Exporting Zephyr SDK...${COLOR_RESET}"
west zephyr-export > /dev/null 2>&1

echo -e "${COLOR_GREEN}Dev container ready!${COLOR_RESET}"

exec "/bin/bash"
