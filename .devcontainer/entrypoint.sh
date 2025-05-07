#! /bin/bash
# shellcheck disable=SC1091

COLOR_BLUE="\e[34m"
COLOR_GREEN="\e[32m"
COLOR_YELLOW="\e[33m"
COLOR_RESET="\e[0m"

# Source ROS and Autoware
source "/opt/ros/humble/setup.bash"
source "/opt/autoware/setup.bash"

# Check if zephyr and cyclonedds submodules are cloned and not empty
if [ ! -d "zephyr" ] || [ ! -d "cyclonedds" ] || [ -z "$(ls -A zephyr 2>/dev/null)" ] || [ -z "$(ls -A cyclonedds 2>/dev/null)" ]; then
    echo -e "${COLOR_YELLOW}Zephyr and/or CycloneDDS submodules are not cloned or are empty.${COLOR_RESET}"
    echo -e "${COLOR_YELLOW}Please clone the submodules then run the container in order to build the project.${COLOR_RESET}"
    echo -e "${COLOR_YELLOW}----------------------------------------------------------${COLOR_RESET}"
fi

# Install Zephyr dependencies
echo -e "${COLOR_BLUE}Checking Zephyr dependencies...${COLOR_RESET}"
pip3 install -r zephyr/scripts/requirements-base.txt > /dev/null 2>&1
pip3 install -r zephyr/scripts/requirements-build-test.txt > /dev/null 2>&1

# Initialize and update Zephyr project
echo -e "${COLOR_BLUE}Initializing and updating Zephyr project...${COLOR_RESET}"
west init -l actuation_module > /dev/null 2>&1
west update > /dev/null 2>&1

# Export Zephyr SDK
echo -e "${COLOR_BLUE}Exporting Zephyr SDK...${COLOR_RESET}"
west zephyr-export > /dev/null 2>&1

# Ready to go!
echo -e "${COLOR_GREEN}Dev container ready!${COLOR_RESET}"
exec "/bin/bash"
