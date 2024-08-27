..
 # Copyright (c) 2023-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

###############
Packet Analyzer
###############

The Packet Analyzer validates actuation commands it receives via TCP sockets
against recorded reference commands.

******
Design
******

The Actuation Service sends control commands to Packet Analyzer only when the
Zephyr app is built with ``-b`` parameter of the build script.

High-level diagram:

.. image:: ../images/packet_analyzer.*
   :align: center

The Actuation Service starts a TCP server that waits for a TCP Connect from the
Packet Analyzer. When the Packet Analyzer connects to the Actuation
Service, it sends control commands over a TCP socket. A python based tool is
used to validate the same by comparing the commands with control
commands stored in ControlCommand.csv. The Packet Analyzer does the following:

- Initializes a TCP client
- Connects to the TCP server started by the Actuation Service on port 49152
- Looks for a sync control command to find the start of the packet chain (the
  first control command in ControlCommand.csv is used for this purpose)
- Compares incoming control commands with control commands stored in
  ControlCommand.csv sequentially
- Reports if a mismatch is found
- Prints the statistics (Observed Frequency, Avg Jitter and Std Deviation)
  collected during the packet chain processing

The tool also checks the sec and nanosec fields in the incoming packet to see if
the control commands are received in the same order they were sent.

************
Requirements
************

The Packet Analyzer package needs python=3.8 on the host
and contains ``pyproject.toml`` to manage build system requirements.

.. _design_analyzer_installation:

************
Installation
************

Please follow the steps below to install the package dependencies in a python
virtual environment. This environment can then be used to launch the analyzer
or the test module. Enter ``deactivate`` to exit the virtual environment.

  .. code-block:: console

    cd packet_analyzer
    python -m venv .venv
    source .venv/bin/activate .
    pip install .

*******
Running
*******

From the python virtual environment, the Packet Analyzer can be started by
executing ``start_analyzer -a localhost``.

*******
Testing
*******

The Packet Analyzer package has a testing module that can be used in the
following ways:

  - manually by executing ``test_packet_analyzer -a localhost`` in another
    terminal from a python virtual environment as shown in
    :ref:`design_analyzer_installation` after starting the Packet Analyzer.
  - via python unittest by executing
    ``cd packet_analyzer && python -m unittest -v test_packet_analyzer.py``
    from a python virtual environment as show in
    :ref:`design_analyzer_installation`.

.. note::

  Both ``start_analyzer`` and ``test_packet_analyzer`` modules have command line
  options to enable loglevel, run the module in a loop or select the recording
  to use. Use ``--help`` to get more information on how to set these options.
