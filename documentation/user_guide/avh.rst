..
 # Copyright (c) 2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

.. |Cortex(R)-R82| replace::
  Cortex\ :sup:`®`-R82

.. _Cortex(R)-R82:
  https://developer.arm.com/Processors/Cortex-R82

###############################
Arm Virtual Hardware deployment
###############################

************
Introduction
************

This page documents the steps needed to run the Safety Island Actuation Demo
with the Primary Compute on an `AWS EC2 <https://aws.amazon.com/ec2/>`_
Graviton3 instance, connected over a Virtual Private Network (VPN) to the Safety
Island on an `Arm Virtual Hardware
<https://www.arm.com/products/development-tools/simulation/virtual-hardware>`_
(AVH) |Cortex(R)-R82|_ based board.

.. note::

  All command examples on this page from the HTML document format can be copied
  by clicking the copy button.
  In the PDF document format, be aware that special characters are added when
  lines get wrapped.

************************
Zephyr application build
************************

On a local machine (validated with Ubuntu 20.04), a suitable environment is
needed to build the Zephyr application. The Actuation Demo repository hosts a
Dockerfile for building a Docker image which contains Zephyr dependencies
(adapted from the `Zephyr documentation
<https://docs.zephyrproject.org/3.5.0/develop/getting_started/index.html#install-dependencies>`_)
and the `Zephyr SDK
<https://docs.zephyrproject.org/3.5.0/develop/getting_started/index.html#install-zephyr-sdk>`_.

If Docker is not installed, follow the `Install Docker Engine
<https://docs.docker.com/engine/install/ubuntu/>`_ instructions in order to
install Docker, and then the `Manage Docker as a non-root user
<https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_
paragraph of the post-installation steps page.

Ensure that the additional packages that will be needed are installed locally:

.. code-block:: console

  $ sudo apt update
  $ sudo apt install --no-install-recommends git

Clone the Actuation Demo repository and its submodules:

.. code-block:: console
  :substitutions:

  $ git clone https://gitlab.arm.com/automotive-and-industrial/safety-island/actuation-demo.git -b |actuation version|
  $ cd actuation-demo
  $ git submodule init
  $ git submodule update

Build the Docker image and get into the build environment:

.. code-block:: console

  $ docker build --build-arg USER=${USER} --build-arg UID=$(id -u) --build-arg BASE_IMAGE=ros:humble-ros-base-jammy -t actuation_zephyr -f Dockerfiles/Dockerfile Dockerfiles/
  $ docker run -it --rm -v .:/actuation-demo -w /actuation-demo actuation_zephyr

Now inside the Docker container, install the remaining dependencies and build
the Zephyr application:

.. code-block:: console

  $ pip3 install -r zephyr/scripts/requirements-base.txt
  $ west init -l actuation_autoware
  $ west update
  $ west zephyr-export
  $ ./build.sh -d -z -t fvp_baser_aemv8r_smp

The resulting Zephyr binary is located at
``build/actuation_autoware/zephyr/zephyr.elf``.

***********************
Virtual machines launch
***********************

AVH launch
==========

Follow the instructions of the `Arm Virtual Hardware User Guide
<https://developer.arm.com/documentation/107660/0600/Overview/Access-and-Costs?lang=en>`_
in order to create an account.

Create a Cortex\ :sup:`®`-R82 device with LAN91C111 networking, upload the
previously compiled ``zephyr.elf`` file as the custom firmware and start the
device. See the `Upload Firmware in Web UI
<https://developer.arm.com/documentation/107660/0600/Device-Firmware/Upload-Firmware-in-Web-UI?lang=en>`_
page for details.

EC2 launch
==========

If needed, follow the `Create Your AWS Account
<https://aws.amazon.com/getting-started/guides/setup-environment/module-one/>`_
tutorial in order to create an AWS account.

If needed, read the `Launch an instance using the new launch instance wizard
<https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/ec2-launch-instance-wizard.html?icmpid=docs_ec2_console>`_
page in order to learn how to create and configure an AWS EC2 instance.

Launch an instance:

- running Ubuntu Server 22.04 on a 64-bit Arm architecture

- with 8 vCPUs and 16GB of RAM on Graviton3 (type "m7g.2xlarge")

- selecting or creating a key pair for login

- enabling "Auto-assign a public IP" and allowing SSH traffic from a sensible
  range of IP addresses

- configuring 32GB of gp3 storage

**************
EC2 connection
**************

Read the `Connect to your Linux instance using an SSH client
<https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/connect-linux-inst-ssh.html#connect-linux-inst-sshClient>`_
paragraph in order to find the SSH command to connect to the instance. Add
``-X`` as an argument to the SSH command in order to enable X11 forwarding. Use
this command to connect.

On the AVH website, in the "Connect" tab of the previously created device, use
the "Download OVPN File" button to download the VPN configuration file.

Copy it to the EC2 instance using SCP. If needed, follow the `Transfer files to
Linux instances using an SCP client
<https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/connect-linux-inst-ssh.html#linux-file-transfer-scp>`_
paragraph for instructions on how to do that. The following steps assume that
``~/avh.ovpn`` is the destination path for the configuration file.

*****************************
Autoware installation and run
*****************************

Repository setup
================

Inside the EC2 instance, follow the `Install Docker Engine
<https://docs.docker.com/engine/install/ubuntu/>`_ instructions in order to
install Docker, and then the `Manage Docker as a non-root user
<https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user>`_
paragraph of the post-installation steps page.

Clone the Actuation Demo repository:

.. code-block:: console
  :substitutions:

  $ git clone https://gitlab.arm.com/automotive-and-industrial/safety-island/actuation-demo.git -b |actuation version|
  $ cd actuation-demo

VPN connection
==============

Install the OpenVPN package and use the VPN configuration
file to connect to the local network of the Safety Island, leaving the VPN
client running in the background:

.. code-block:: console

  $ sudo apt update
  $ sudo apt install --no-install-recommends openvpn
  $ sudo -b openvpn --config ~/avh.ovpn

The expected output ends with:

.. code-block:: text

  [...] TUN/TAP device tap0 opened
  [...] net_iface_mtu_set: mtu 1500 for tap0
  [...] net_iface_up: set tap0 up
  [...] net_addr_v4_add: X.X.X.X/X dev tap0
  [...] Initialization Sequence Completed

.. warning::

  If the name of the interface created is not "tap0", update the CycloneDDS
  configuration file to reflect it with ``sed -i 's/tap0/tapX/g'
  cyclonedds-avh.xml``, replacing "tapX" with the name observed in the output of
  the openvpn command.

Runtime environment
===================

Build the Docker image and get into the runtime environment:

.. note::

  An SSH connection with X11 forwarding must have been established first for the
  X11-related files to have been created.

.. code-block:: console

  $ docker build --build-arg USER=${USER} --build-arg UID=$(id -u) -t actuation_autoware --target autoware -f Dockerfiles/Dockerfile Dockerfiles/
  $ docker run -it -v .:/actuation-demo -w /actuation-demo --net host -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw -e XAUTHORITY=${HOME}/.Xauthority -e DISPLAY=${DISPLAY} actuation_autoware

Now inside the Docker container, build the ROS2 packages specific to the
Actuation Demo:

.. code-block:: console

  $ ./build.sh -a

Set up the execution environment:

.. code-block:: console

  $ source install/setup.bash
  $ export ROS_DOMAIN_ID=1
  $ export CYCLONEDDS_URI=$(pwd)/cyclonedds-avh.xml

.. note::

  The `ROS Domain IDs
  <https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html>`_
  ``1`` and ``2`` are expected not to be used by other machines on the
  sub-network.

Set up the demo by following the `Preparation
<https://github.com/autowarefoundation/autoware-documentation/blob/445a776ca7207e305371daf43376b7704ba9073d/docs/tutorials/ad-hoc-simulation/planning-simulation.md#preparation>`_
paragraph of the Autoware documentation.

Run the demo with:

.. code-block:: console

  $ ros2 launch actuation_demos planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

.. note::

  Periodic error logs are expected from this command.

By default, it launches ``rviz2`` for visualization.

Follow `steps 2 to 4 of the Autoware documentation
<https://github.com/autowarefoundation/autoware-documentation/blob/445a776ca7207e305371daf43376b7704ba9073d/docs/tutorials/ad-hoc-simulation/planning-simulation.md#2-set-an-initial-pose-for-the-ego-vehicle>`_
in order to interact with the simulator and run the demo.

.. note::

  The steering wheel overlay may look oversized due to X11 forwarding.

Clean up
========

Remember to either stop or terminate the EC2 and AVH instances after the demo in
order to avoid accumulating costs.
