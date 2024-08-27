..
 # Copyright (c) 2022-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

#################
Actuation Service
#################

The Actuation Service provides the Safety Island feature of the Actuation Demo.

It spawns a Pure Pursuit node and waits for termination of the program.

********************
Zephyr configuration
********************

The project configuration options are split into several ``prj.conf`` files in
order to avoid duplication of the options when used for different purposes. The
files are:

**prj_actuation.conf**:
  Contains the core options. It configures the Zephyr libc used, the common
  networking options (IPv4, net sockets, UDP and network buffers), the threads
  and the memory.

**boards/<BOARD>_actuation.conf**:
  Board-specific configurations that get used along with ``prj_actuation.conf``.

**prj_net.conf**:
  Contains options specific to the network configuration. It configures the L2
  layer and static IP addresses.

The core configuration file is meant to be used by setting the cmake
``CONF_FILE`` variable. The other configuration files are meant to be used by
setting the cmake ``OVERLAY_CONFIG`` variable. The selection process of the
configuration files is explained in `Zephyr's Initial Configuration
<https://docs.zephyrproject.org/3.5.0/build/kconfig/setting.html#the-initial-configuration>`_.

*******************
Devicetree overlays
*******************

The Zephyr app uses `devicetree overlays
<https://docs.zephyrproject.org/3.5.0/build/dts/howtos.html#set-devicetree-overlays>`_
to enable the board-specific devices defined in the platform's DTS. They are
placed in ``boards/`` and meant to be discovered automatically by the Zephyr
build system.

****************
Build variations
****************

The Zephyr app can be built to either publish control commands via DDS or
forward them over a BSD TCP socket by setting the ``CONTROL_CMDS_FWD`` build
time cmake variable to either ``dds`` (the default) or ``bsd_socket``. When
built with ``bsd_socket``, the Zephyr app starts a TCP server thread on port
49152 that does the following:

- Listen for TCP Connect request from the Packet Analyzer (at most 1 connection)
- Publish control commands on the spawned client socket if a client is connected
- Perform a parallel blocking read against the client socket to listen for a
  tear off request from the client
- Reset the client socket upon the tear off request OR a send failure OR a read
  failure, and continue to listen for connection request from the Packet
  Analyzer
