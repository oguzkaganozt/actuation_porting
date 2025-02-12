#! /bin/bash

west init -l zephyr_app > /dev/null 2>&1
west update
west zephyr-export

exec "/bin/bash"