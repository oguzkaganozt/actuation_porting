..
 # Copyright (c) 2022-2023, Arm Limited.
 #
 # SPDX-License-Identifier: Apache-2.0

#################
Message Converter
#################

This is the design document for the ``actuation_message_converter`` package.

*******************
Purpose / Use cases
*******************

This package allows communication between the main pipeline (from
autoware.universe) and the Pure Pursuit service (from Autoware.Auto) running in
the Safety Island.

Data structure conversion
=========================

CycloneDDS provides a simple compiler for IDL files, generating C headers
implementing the data structures. ROS2 uses its own IDL compiler that generates
different data structures than the CycloneDDS compiler (specifically, the
sequences and strings are implemented differently), as well as additional C
files containing initialization code. The difference in data structures makes it
impossible to simply use the ROS2 message types on the Autoware side and the
CycloneDDS message types on the pure_pursuit side.

Message type conversion
=======================

This package translates the message types between the Autoware pipeline and Pure
Pursuit. See the :ref:`inputs_outputs_api` paragraph below for details.

Domain separation
=================

DDS participants discover all the other endpoints available in their DDS Domain.
It results in allocating memory for each of those remote endpoints and keeping
it allocated until their termination. This represents a very significant amount
of memory for the constrained resources of the Safety Island sub-system. This
package is used to bridge two different Domain IDs, one dedicated to the main
pipeline and one for the Pure Pursuit service.

******
Design
******

This package generates both ROS2 message types and CycloneDDS message types from
the message definitions used. Both are used by the node to translate an incoming
message in one type to an output message in the other.

This node receives messages from Autoware nodes, and upon reception creates
CycloneDDS messages to be sent using the CycloneDDS API to the pure_pursuit app.
The same thing would then be done for translating back the messages emitted by
the pure_pursuit app and sent to the right Autoware packages.

Assumptions / Known limits
==========================

The node is not generic. The code needs to be modified to accommodate for a
change in the flow of messages.

The ``idlc`` compiler from CycloneDDS 0.8 doesn't support default values for the
message fields (it seems support was added on more recent versions). It is not a
problem when the default values are 0, which is the case most of the time, but
other default values need to be carefully handled when creating those messages,
and messages that depend on them. Currently that is only the case for the ``w``
field of the ``Quaternion`` message, ultimately used in ``Trajectory`` and
``VehicleKinematicState``, that should be default-valued to 1.

The ``idlc`` compiler from CycloneDDS doesn't prevent multiple includes of the
same file, leading to a re-definition of the ``Quaternion`` struct in our case.
It needs to be worked around. The least intrusive way is by having a local
``Quaternion`` definition in ``Transform``.

.. _inputs_outputs_api:

Inputs / Outputs / API
======================

.. list-table:: Conversion table
   :header-rows: 1

   * - rosidl (Primary Compute)
     - idlc (Safety Island)
   * - Odometry
     - VehicleKinematicState
   * - Trajectory
     - Trajectory
   * - AckermannControlCommand
     - VehicleControlCommand

Inner-workings / Algorithms
===========================

Message compilation
-------------------

The Autoware messages to be translated are compiled to IDLC with the CycloneDDS
compiler. Those messages, and their dependencies, are:

- VehicleControlCommand (from the Safety Island)

  + Time

- Trajectory (CI->SI)

  + TrajectoryPoint

    + Pose

      + Point
      + Quaternion

    + Duration

  + Header

    + Time

- VehicleKinematicState (to the Safety Island)

  + TrajectoryPoint

    + Pose

      + Point
      + Quaternion

    + Duration

  + Transform

    + Quaternion
    + Vector3

  + Header

    + Time

Error detection and handling
============================

The node aborts on any error code from the CycloneDDS API during initialization.
A message is printed on any error code after CycloneDDS has been initialized.

***********************
Security considerations
***********************

- Spoofing

  The origin of the incoming messages does not matter.

- Tampering

  This package handles variable-length data structures: Incoming C strings are
  not parsed by the package, but rather left to CycloneDDS for handling. The
  length of incoming arrays is checked against the internal maximum length; if
  the incoming length is greater, the extra elements are silently ignored.

- Repudiation

  The actions of external actors should not affect this package.

- Information disclosure

  There is no information to disclose.

- Denial of Service

  The package relies on the DDS middleware to mitigate for Denial of Service
  threats.

- Elevation of Privilege

  There is no privileges to elevate.

*******
Testing
*******

This package translates messages format from ROSIDL to IDLC (from Autoware to
the Safety Island) or from IDLC to ROSIDL (from the Safety Island to Autoware),
depending on the message.

The unit test creates DDS participants around ``actuation_message_converter`` to
simulate a full loop of Autoware -> **Converter** -> SI -> **Converter** ->
Autoware. Where the Converter in bold is the functionality under test and the
rest is the test-bench.

The goal is to evaluate the conversion, but also to ensure that the rest of the
pipeline is working as intended.
