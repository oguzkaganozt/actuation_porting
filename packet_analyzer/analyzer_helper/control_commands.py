# Copyright (c) 2023, Arm Limited.
#
# SPDX-License-Identifier: MIT

import os
import csv
import math
import itertools
import numpy as np
from dataclasses import dataclass


"""
This helper module contains:
 ControlCommand class for storing control command
 RxControlCommand class (inherits from ControlCommand) for storing control
   commands received from the Actuation Service
 RefControlCommands class that opens a pre-calculated control commands stored
   as ./ControlCommand.csv, and stores control commands as ControlCommand and
   offers a iter interface.
"""


@dataclass
class ControlCommand(object):
  """ Class for managing control commands

  The "sec"/"nanosec" found in the control command from Actuation Service
  denotes the beginning of processing by Autoware Services where as the
  "timestamp" in the csv file is the time at which the messages were recorded.
  It gives information on the order of the messages when playing them back and
  hence will not be part of the ControlCommand class
  :param ts: timestamp
  :type ts: np.uint64
  :param accel: acceleration
  :type accel: np.float32
  :param vel: velocity
  :type vel: np.float32
  :param f_angle: front wheel angle
  :type f_angle: np.float32
  :param r_angle: rear wheel angle
  :type r_angle: np.float32
  """
  ts: np.uint64
  accel: np.float32
  vel: np.float32
  f_angle: np.float32
  r_angle: np.float32

  def __eq__(self, other):
    accepted_tol = 0.0001
    return \
      math.isclose(self.accel, other.accel, rel_tol=accepted_tol) and \
      math.isclose(self.vel, other.vel, rel_tol=accepted_tol) and \
      math.isclose(self.f_angle, other.f_angle, rel_tol=accepted_tol) and \
      math.isclose(self.r_angle, other.r_angle, rel_tol=accepted_tol)

  def __str__(self):
    return f"accel={self.accel}, vel={self.vel}, f_angle={self.f_angle}, " + \
           f"r_angle={self.r_angle}"


@dataclass
class RxControlCommand(ControlCommand):
  """ Class for incoming control commands from Actuation Service

  Control commands from Actuation Service include "sec"/"nanosec" which denotes
    the  beginning of processing by Autoware Services.
  :param sec: seconds
  :type sec: np.int32
  :param nanosec: seconds
  :type nanosec: np.uint32
  """
  sec: np.int32
  nanosec: np.uint32

  def __str__(self):
    return "ControlCommand(sec={:10d}, nsec={:10d}, accel={}," \
           " vel: {}, f_angle {}, r_angle: {}" \
           "".format(self.sec, self.nanosec, self.accel, self.vel,
                     self.f_angle, self.r_angle)


class RefControlCommands(object):
  """ A control commands storage implementation

   This class:
   - instantiates an object that stores control commands based
     on 'ControlCommand.csv'
   - provide an iterator interface to move to next control command tuple
   - provide a get_sync_command interface which returns sync control command

    :param control_command_path: Path containing ControlCommand.csv
    :type control_command_path: str
  """
  _ref_control_command_file = "./ControlCommand.csv"
  _header_len = 3
  _control_command_len = 5

  def __init__(self, control_command_path):
    # Find the abs path for ./ControlCommand.csv. This file is maintained in the
    # root of the actuation repo
    self._commands_index = 0
    self._commands_len = 0
    self._sync_command = None
    self._ref_control_commands = []
    ref_file = RefControlCommands._ref_control_command_file
    csv_path = \
      os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"../packet_analyzer/{control_command_path}/{ref_file}")

    # Read contents of 'ControlCommand.csv' and store the same as list
    with open(csv_path) as csv_file:
      # Skip header portion from the csv file and iterate over the contents
      csv_reader = csv.reader(csv_file, delimiter=',')
      csv_reader = itertools.islice(csv_reader,
                                    RefControlCommands._header_len, None)
      for line in csv_reader:
        if len(line) != RefControlCommands._control_command_len:
          log.error("Malformed Ref Control Command, Rx: " + str(line))
          exit(1)

        # Control command order is ts (np.uint64), acceleration (np.float32),
        # speed (np.float32), # f_angle (np.float32), (r_angle) np.float32.
        # Note: ts read from csv file is used to calculate jitter
        control_command = ControlCommand(np.uint64(line[0]),
                                         float.fromhex(line[1]),
                                         float.fromhex(line[2]),
                                         float.fromhex(line[3]),
                                         float.fromhex(line[4]))

        # Actuation Service uses a pre-determined control packet structure
        # to mark the start of the packet chain. This is stored as the
        # first entry in the csv file.
        if not self._sync_command:
          self._sync_command = control_command
        else:
          self._ref_control_commands.append(control_command)
    self._commands_len = len(self._ref_control_commands)

  def __iter__(self):
    self._commands_index = 0
    return self

  def __next__(self):
    if self._commands_index < self._commands_len:
      result = self._ref_control_commands[self._commands_index]
      self._commands_index += 1
      return result
    else:
      raise StopIteration

  def get_sync_command(self):
    return self._sync_command
