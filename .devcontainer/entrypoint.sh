#! /bin/bash

pip3 install -r zephyr/scripts/requirements-base.txt
pip3 install -r zephyr/scripts/requirements-build-test.txt

west init -l zephyr_app > /dev/null 2>&1
west update
west zephyr-export

exec "/bin/bash"