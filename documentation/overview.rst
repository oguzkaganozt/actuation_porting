..
 # Copyright (c) 2022-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

.. |Cortex(R)-R82| replace::
  Cortex\ :sup:`®`-R82

.. _Cortex(R)-R82:
  https://developer.arm.com/Processors/Cortex-R82

########
Overview
########

************
Introduction
************

Automotive compute platforms responsible for hosting services such as Advanced
Driver Assistance Systems (ADAS) and Autonomous Drive (AD) stacks require
increasingly complex and more powerful systems for these demanding workloads. To
aid with achieving the additional safety goals required in the automotive
environment, these systems benefit from the addition of a Safety Island, a
separate compute sub-system to help with functional and system monitoring.

This example shows how an AD software stack can be run in a compute environment
composed of a high-performance Primary Compute platform coupled with a
higher reliability Arm\ :sup:`®` Cortex\ :sup:`®`-R based Safety Island.

The Safety Island Actuation Demo features an Actuation Service application
running on the Safety Island that receives inputs from the Primary Compute. In
this reference implementation, the Primary Compute runs an Autoware pipeline,
and the Actuation Service takes the form of a Zephyr application showcasing the
`Pure Pursuit
<https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/pure-pursuit.html>`_
algorithm from Autoware.Auto. The two communicate via `Data Distribution Service
<https://www.dds-foundation.org/what-is-dds-3/>`_ (DDS) messages over a network
interface.

The goal of the Actuation Service is to check the input messages and to generate
actuation commands to be sent towards the actuators. A "Message Converter"
`Robot Operating System <https://docs.ros.org/en/humble/index.html>`_ (ROS)
package is created as part of this reference implementation, and is run on the
Primary Compute, translating the message structure to improve readability by the
Actuation Service.

The Actuation Demo git repository holds all the components necessary to run the
demo, and provides helper scripts to build and run it.

This documentation provides user guides for two deployment options:

- :ref:`user_guide/ava_s32z:AVA + S32Z deployment` on physical platforms, with
  the Primary Compute provided by an `AVA Developer Platform
  <https://www.adlinktech.com/Products/Computer_on_Modules/COM-HPC-Server-Carrier-and-Starter-Kit/AVA_Developer_Platform>`_
  from ADLINK, and the Safety Island function hosted on an NXP `S32Z Real-Time
  Processor
  <https://www.nxp.com/products/processors-and-microcontrollers/s32-automotive-platform/s32z-and-s32e-real-time-processors:S32Z-E-REAL-TIME-PROCESSORS>`_
  development board.

- :ref:`user_guide/avh:Arm Virtual Hardware deployment` on virtualized
  platforms, with the Primary Compute provided by a virtualized Arm\ :sup:`®`
  Neoverse\ :sup:`™` based platform, and the Safety Island function hosted on an
  `Arm Virtual Hardware
  <https://www.arm.com/products/development-tools/simulation/virtual-hardware>`_,
  |Cortex(R)-R82|_ based platform.

*********************
Solution Architecture
*********************

High-level diagram:

.. image:: images/actuation_overview.*
  :align: center

Use Cases
=========

- **Validation of communication**: Validating that the communication between an
  Autoware pipeline (on the Primary Compute) and the Actuation Service (on the
  Safety Island) is working. It validates that the content of ROS2 messages from
  Autoware can be received and read by a non-ROS application running on Zephyr
  on a Cortex\ :sup:`®`-R CPU. It also validates the network connection between
  the two platforms.

- **Safe execution platform**: Showing Autoware nodes running on an RTOS
  operating with the benefits of a Cortex\ :sup:`®`-R CPU.

- **Code porting example**: Preserving the ROS interface to enable Autoware
  modules running within a different OS environment.

Features
========

- **Autoware pipeline**: The Autoware pipeline for the demo. More details can be
  found in :ref:`design/feature_autoware:Autoware pipeline`.

- **Actuation Service**: The Zephyr application of the Actuation Demo. More
  details can be found in :ref:`design/feature_actuation_service:Actuation
  Service`.

- **Message Converter**: The ROS package converting DDS message structures. More
  details can be found in :ref:`design/feature_converter:Message Converter`.

********************
Repository Structure
********************

The high-level structure of the ``actuation-demo`` repository is as follows:

- **actuation_packages**: ROS packages used for the demo.

  - **actuation_demos**: Contains the launch script(s) of the available demo(s).

  - **actuation_format_bag**: Provides a helper tool to generate CSV files from
    rosbag data.

  - **actuation_message_converter**: Does the DDS message conversion.

  - **actuation_msgs**: Provides IDLC messages used in the demo(s).

  - **actuation_player**: Plays back recorded data as DDS messages.

- **cyclonedds**: Submodule targeting an upstream commit of the CycloneDDS
  repository.

- **Dockerfiles**: Contains Dockerfile(s) to automate setting up containerized
  environments.

- **documentation**: Directory that provides documentation for the
  ``actuation-demo`` repository.

- **packet_analyzer**: Python based application running on the host validating
  Control Commands produced by the Actuation Service.

- **zephyr**: Submodule targeting an upstream commit of the Zephyr repository.

- **zephyr_app**: The Zephyr application providing the functionality of the
  Actuation Service.

******************
Repository License
******************

The software is provided under an Apache-2.0 license (more details in
:ref:`license_file_link:Apache License`).

*****************************
Contributions and Bug Reports
*****************************

This project has not put in place a process for contributions or bug reports.

********************
Feedback and support
********************

To request support please contact Arm at support@arm.com. Arm licensees may also
contact Arm via their partner managers.

*************
Maintainer(s)
*************

- Ambroise Vincent <ambroise.vincent@arm.com>
