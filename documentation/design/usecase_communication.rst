..
 # Copyright (c) 2022-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

###########################
Validation of communication
###########################

The demo can be used as a validation for the correctness of communication
between an Autoware pipeline running on the Primary Compute and a Zephyr
application running on the Safety Island. It validates that the content of ROS2
messages from Autoware can be received and read by a non-ROS application running
on Zephyr on a Cortex-R cluster. It also validates the network connection
between the two platforms.

Additional code path has been added to both Zephyr app (to forward control
commands to host that can be validated by the Packet Analyzer) and Zephyr tests
(to validate this forwarding of control commands). The Zephyr app's behavior of
publishing control commands via DDS or forwarding control commands to the host
can be set with the ``-b`` option when using the build script.
