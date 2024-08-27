..
 # Copyright (c) 2022-2024, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

#################
Autoware pipeline
#################

The Autoware pipeline of the Actuation Demo is the `Planning simulation
<https://github.com/autowarefoundation/autoware-documentation/blob/445a776ca7207e305371daf43376b7704ba9073d/docs/tutorials/ad-hoc-simulation/planning-simulation.md>`_
from the "Ad hoc" tutorial of Autoware. It can be launched with the
``actuation_demos`` package, which contains the appropriate launch scripts.

The package has been copied from the ``autoware_launch`` package of Autoware,
taking a subset of files from it. Its "control" launch file has been modified to
accommodate for the specific needs of the Actuation Demo.
