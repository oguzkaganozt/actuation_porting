..
 # Copyright (c) 2022-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

..
  # Trailing whitespace on purpose
.. |cspell:disable-line| replace:: \ 

#########################
Changelog & Release Notes
#########################

******
latest
******

New Features
============

- New ``fvp_baser_aemv8r_smp`` Zephyr target for the Actuation Service.

- New AVH platform deployment, leveraging the ``fvp_baser_aemv8r_smp`` Zephyr
  target support.

Third-party repositories
------------------------

.. code-block:: yaml
    :substitutions:

    name:   cyclonedds
    url:    https://github.com/eclipse-cyclonedds/cyclonedds.git
    |cspell:disable-line|branch: master
    commit: f7688ce709e53f408e30706ebc27bd052c03d693

    name:   zephyr
    url:    https://github.com/zephyrproject-rtos/zephyr.git
    branch: main
    commit: 339cd5a45fd2ebba064ef462b71c657336ca0dfe

    name:   meta-ewaol
    url:    https://gitlab.com/soafee/ewaol/meta-ewaol.git
    branch: kirkstone-dev

Changed
=======

- Removed Autoware submodule. The demo now relies on pre-compiled binaries.

****
v2.0
****

New Features
============

- Added new components to the repository. They allow creating a lighter
  deployment of the Safety Island Actuation Demo, without running the Autoware
  pipeline on the Primary Compute and using a recording of the messages instead.

  - An Actuation Player component that reads back recorded messages and sends
    them as DDS messages.

  - A Packet Analyzer that validates actuation commands against reference ones.

Third-party repositories
------------------------

.. code-block:: yaml
    :substitutions:

    name:   autoware
    url:    https://github.com/autowarefoundation/autoware.git
    branch: release/2023.10
    commit: 78e5f575b258598e6460e6f04cc00211e7e7e604

    name:   cyclonedds
    url:    https://github.com/eclipse-cyclonedds/cyclonedds.git
    |cspell:disable-line|branch: master
    commit: f7688ce709e53f408e30706ebc27bd052c03d693

    name:   zephyr
    url:    https://github.com/zephyrproject-rtos/zephyr.git
    branch: main
    commit: 339cd5a45fd2ebba064ef462b71c657336ca0dfe

    name:   meta-ewaol
    url:    https://gitlab.com/soafee/ewaol/meta-ewaol.git
    branch: kirkstone-dev

Changed
=======

- Simplified and improved the steps of the user guide to reproduce the demo.

  - New Dockerfile to simplify the reproduce steps for the user.

  - Steps using the IDE to flash the development board replaced with using a
    command line interface.

- Updated the version of third party repositories.

  - Autoware updated to the 2023.10 release, which updates the underlying ROS2
    version from Galactic to Humble.

  - Zephyr updated to the 3.5.0 release. The targeted commit is ahead of the
    release in order to include patches providing better support for the S32Z
    board.

  - CycloneDDS updated to support the latest Zephyr version.

- Started using distinct ROS domain IDs for the Autoware pipeline on the Primary
  Compute and the Actuation Service on the Safety Island.

Limitations
===========

- A devicetree overlay ``actuation_autoware/boards/s32z270dc2_rtu0_r52.overlay`` is used
  to set the MAC address of the NXP S32Z270DC2_R52 board. This is done as a
  workaround as the NXP S32Z270DC2_R52 platform uses the same MAC address for
  every build (issue tracked in `Zephyr Project#61478
  <https://github.com/zephyrproject-rtos/zephyr/issues/61478>`_).

- The AVA Developer Platform and S32Z need to be on the same sub-network.

- Rendering issues with the ``rviz2`` program used by the Autoware demo have
  been observed on specific GPU and driver combinations. In particular, machines
  with `NVIDIA Optimus <https://en.wikipedia.org/wiki/Nvidia_Optimus>`_
  technology have been seen to error with ``libGL error: failed to create
  drawable`` lines leading to a crash of the program. There are no known
  workarounds apart from using a different machine to do the render.

Known Issues
============

None

Resolved Issues
===============

- The known issue of the 1.0 release requiring to re-flash the demo after each
  run has been resolved.

- Official support has been added to Zephyr for the S32 Debug Probe. The need
  for launching the S32 Design Studio IDE and the workaround involving user
  action to set system registers to the correct value have been removed.

****
v1.0
****

New Features
============

- First release.

  - Pure Pursuit as the Zephyr application, autoware.universe as the main pipeline.

Third-party repositories
------------------------

.. code-block:: yaml
    :substitutions:

    name:   autoware
    url:    https://github.com/autowarefoundation/autoware.git
    branch: main
    commit: 3a9bbd0142b453563469b8a3a6d232e98a51280a

    name:   cyclonedds
    url:    https://github.com/eclipse-cyclonedds/cyclonedds.git
    |cspell:disable-line|branch: master
    commit: 87b31771ad4dda92afccc6ad1cb84cb7f752b66b

    name:   zephyr
    url:    https://github.com/zephyrproject-rtos/zephyr.git
    branch: main
    commit: 07c6af3b8c35c1e49186578ca61a25c76e2fb308

    name:   meta-ewaol
    url:    https://gitlab.com/soafee/ewaol/meta-ewaol.git
    branch: kirkstone-dev

Changed
=======

- Initial release.

Limitations
===========

- No official support for the NXP S32 Debug Probe to debug the S32Z board.

- The AVA Developer Platform and S32Z need to be on the same sub-network.

Known Issues
============

- The S32Z board needs to be flashed before each run of the demo. Issue tracked
  in `CycloneDDS#1682
  <https://github.com/eclipse-cyclonedds/cyclonedds/issues/1682>`_.
