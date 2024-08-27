..
 # Copyright (c) 2023, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

################
Actuation Player
################

The Actuation Player reads recorded trajectory and kinematic state files
produced by ``actuation_format_bag``. It sends their content as DDS messages.

*******
Package
*******

This package follows the ROS2 package design. It can be built and tested by
using `colcon
<https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_.

*****
Usage
*****

The Player can be launched by:

.. code-block:: console

    Usage: ./actuation_player [OPTIONS]
        -h, --help            Print this help message and exit
        -l, --loop            Run in a loop
        -d DIV, --divider DIV Divide the replay speed (floating point value)
        -p PATH, --path PATH  Path to the directory containing the recordings

The default target directory is "../share/" relative to the directory containing
the executable.

The Player expects to find ``cyclonedds.xml`` in this default directory. It gets
used as the CycloneDDS configuration file. To provide a different configuration
file, set the ``CYCLONEDDS_URI`` environment variable to point to it.

The Player expects to find ``KinematicSate.csv`` and ``Trajectory.csv`` in the
target directory. Use the "-p" option to change the target directory.

*******
Testing
*******

The unit test creates DDS participants around ``actuation_player`` to simulate
the Pure Pursuit algorithm running in the Actuation Service. It reads the DDS
messages from the Player and compares them against static expected data. It also
checks for correct handling of incorrect input.
