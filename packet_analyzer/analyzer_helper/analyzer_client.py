# Copyright (c) 2023, Arm Limited.
#
# SPDX-License-Identifier: MIT

import sys
import time
import errno
import fcntl
import struct
import socket
import select
import logging
import numpy as np
from enum import Enum
from analyzer_helper import control_commands

"""
AnalyzerClient class is responsible for validating the Control Commands produced
by the Actuation Service running in the Safety Island. The Actuation Service
runs a TCP server that sends the Control Commands (upon registration from the
Packet Analyzer) over BSD TCP socket which will be validated against
pre-calculated Control Commands stored as ./ControlCommand.csv.
Each control command consists of the following:
- second (np.int32)
- nanosec (np.uint32)
- acceleration (np.float32)
- velocity (np.float32)
- front wheel angle (np.float32)
- rear wheel angle (np.float32)
"""


class ActuationServiceVars(object):
  """ Class for storing Actuation Service variables
  """
  ACTUATION_SERVICE_PORT = 49152
  CONTROL_COMMAND_PAYLOAD_LEN = 24
  PACKET_ANALYZER_FIN = b'packet_analyzer_fin'
  PACKET_ANALYZER_FIN_ACK = b'packet_analyzer_fin_ack'


class AnalyzerResult(Enum):
  """ Class for managing error codes
  """
  SUCCESS = 0
  PKT_OUT_OF_ORDER = 1
  COMMAND_MISMATCH = 2
  USER_INTERRUPTED = 3
  FAILURE = 4


class AnalyzerClient(object):
  """ An analyzer sever class implementation

    :param host: host
    :type host: str
    :param port: TCP port number
    :type port: int
    :param control_command_path: Path containing ControlCommand.csv
    :type control_command_path: str
  """

  def __init__(self, host, port, control_command_path='./data', run_loop=False):
    """ Initialize TCP client

    :param host: host
    :param port: TCP port number
    :param control_command_path: Path containing ControlCommand.csv
    """
    self._host = host
    self._port = port
    self._run_loop = run_loop

    self._count = 0
    self._deltas = []
    self._user_interrupted = False
    self._cli_socket_connected = False
    self._ref_control_commands = \
      control_commands.RefControlCommands(control_command_path)
    self._sync_cmd = self._ref_control_commands.get_sync_command()
    self._cli_socket = \
      socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
    # timeout used for select call to check if _cli_socket is ready for a read
    self._timeout = 0.1

  def __del__(self):
    """ Close TCP socket
    """
    self._cli_socket.close()

  def _get_payload(self, retries=0):
    """ Retrieve one packet from socket
    Uses python socket to receive the Control Command from Actuation Service
    running on Safety Island.

    :param retries: Number of tries to perform till _user_interrupted is ack-ed
    :return: payload in byte object format
    :return: seconds at which packet was received by n/w stack
    :return: microsecs at which packet was received by n/w stack
    """
    buf_len = ActuationServiceVars.CONTROL_COMMAND_PAYLOAD_LEN
    buf = bytearray(buf_len)
    mview = memoryview(buf)

    while (buf_len):
      read_ready, _, _ = select.select([self._cli_socket], [], [],
                                       self._timeout)
      # Break _get_payload if user has interrupted
      if self._user_interrupted:
        if retries == 0:
          logging.info(f"Ack-ing user_interrupted")
          return (None, 0, 0)
        retries -= 1

      if read_ready:
        bytes_read = self._cli_socket.recv_into(mview, buf_len)
        if bytes_read <= 0:
          logging.error(f"recv_into returned {bytes_read}")
          return (None, 0, 0)
        mview = mview[bytes_read:]
        buf_len -= bytes_read

    # Return time recorded when the packet was received. This information is
    # used during jitter calculation.
    ts = time.time_ns()
    sec = (np.int32)(ts // 1000000000)
    usec = (np.uint32)(ts % 1000000000)

    return (buf, sec, usec)

  def _tear_conn(self):
    """ Perform a clean TCP connection tear off by sending a FIN and waiting
        for a FIN ACK
    """
    buf_len = ActuationServiceVars.CONTROL_COMMAND_PAYLOAD_LEN
    buf = bytearray(buf_len)
    mview = memoryview(buf)
    fin_len = len(ActuationServiceVars.PACKET_ANALYZER_FIN)
    mview[:fin_len] = ActuationServiceVars.PACKET_ANALYZER_FIN
    try:
      self._cli_socket.sendall(buf)
    except Exception as s_err:
      logging.error(f"FIN packet send error: {s_err}")
      return

    # Once a FIN packet is sent, packet analyzer will look for a FIN_ACK in a
    # max of 5 packets from the Actuation Service
    # for i in range(5):
    while True:
      payload, _, _ = self._get_payload(5)
      if payload is None:
        # Any recv error is considered a hard error
        logging.error("tear conn failed to receive FIN_ACK")
        break

      if ActuationServiceVars.PACKET_ANALYZER_FIN_ACK in payload:
        logging.info('Received fin ack from Actuation Service')
        # Give the Actuation Service time to close the socket
        time.sleep(1)
        break
      else:
        logging.info(f"Ignoring {payload}, waiting for fin ack")

  def _command_from_payload(self, payload):
    """ Extract the control command from payload

    Extract control command (np.int32 (sec), np.uint32 (nanosec),
      np.float32 (acceleration), np.float32 (velocity),
      np.float32 (front wheel angle), np.float32 (rear wheel angle)

    :param payload: payload to be extracted from
    :return: an instance of control_commands.ControlCommand
    """
    sec, nanosec, accel, vel, f_angle, r_angle = \
      struct.unpack("<iIffff", payload)

    self._count += 1
    logging.debug(f"({self._count}) Rx: accel={accel}, vel={vel}, " +
                  f"f_angle={f_angle}, r_angle={r_angle}")

    return control_commands.RxControlCommand(0, accel, vel, f_angle, r_angle,
                                             sec, nanosec)

  def _rx_packet_in_order(self, command, prev_cmd_sec, prev_cmd_nsec):
    """ Use prev_cmd_sec and prev_cmd_nsec to check if packets arrive in order

    RxControlCommand consists of a prev_cmd_sec and prev_cmd_nsec. Add a check
      to make sure that the packets are received in order
    :param command: RxControlCommand instance to check
    :param prev_cmd_sec: time of the previous packet
    :param prev_cmd_nsec: time of the previous packet
    :return: true if the packets arrived in order
    """
    ret = prev_cmd_sec > command.sec or \
      (prev_cmd_sec == command.sec and prev_cmd_nsec >= command.nanosec)
    return not ret

  def _store_delta(self, prev_pkt_sec, prev_pkt_nsec, pkt_sec, pkt_nsec):
    """ Compute delta between current and previous packet receive time

    :param prev_pkt_sec: time (seconds) the last tcp packet was received
    :param prev_pkt_nsec: time (microsecs) the last tcp packet was received
    :param pkt_sec: time (seconds) the current tcp packet was received
    :param pkt_nsec: time (microsecs) the current tcp packet was received
    """
    pkt_usec_full = (np.uint64)(pkt_sec * 1000000000) + pkt_nsec
    prev_pkt_nsec_full = (np.uint64)(prev_pkt_sec * 1000000000) + prev_pkt_nsec
    self._deltas.append((pkt_usec_full - prev_pkt_nsec_full) / 1000000000)

  def _log_jitter(self):
    """ Log observed frequency, jitter average and jitter standard deviation
    """
    if len(self._deltas):
      deltas = np.array(self._deltas, np.float32)
      deltas_mean = deltas.mean()
      jitters = abs(deltas - deltas_mean)
      logging.info("Observed Frequency = {:10.8f}, Avg Jitter = {:10.8f},"
                   " Std Deviation:{:10.8f}"
                   "".format(1/deltas_mean, jitters.mean(), jitters.std()))
    else:
      logging.error("Jitter calculation failed, empty delta set")

  def _connect_to(self):
    """ Does a tcp connect (blocking) towards the Actuation Service running on
          the Safety Island
    """
    logging.info("Starting analyze, use Ctrl-C to stop the process.")
    logging.info(f"Attempting a connect to ({self._host} : {self._port})")
    while not self._user_interrupted:
      try:
        self._cli_socket.connect((self._host, self._port))
        self._cli_socket_connected = True
        logging.info(f"Successfully connected to ({self._host} : {self._port})")
        break
      except socket.error as s_err:
        if s_err.errno not in [errno.ECONNREFUSED, errno.ECONNABORTED]:
          logging.error(f"Connect failed with error: {str(s_err)}")
          raise s_err
        time.sleep(.2)

  def stop_run(self):
    self._user_interrupted = True
    logging.info(f"Setting user_interrupted to True from sig handler")

  def run_analyze_on_chain(self):
    """ Run Packet analyzer on socket that looks for commands from
          the  Actuation Service

    Performs the following:
    - Creates an iterator based on a pre-calculated reference control commands
       stored as csv file
    - Does a recv on the provided socket for control commands sent by
       the Actuation service
    - Extracts control command payload from the received payload
    - Discards packets till it finds the start of the packet chain (The first
        entry in the reference control commands is the sync packet for this
        purpose)
    - Matches each control packet till it reaches the end of ref control
        commands
    - Rets / Prints the following at the end of the function
      AnalyzerResult.SUCCESS: if incoming control commands match the
                              pre-calculated reference control commands
      AnalyzerResult.PKT_OUT_OF_ORDER: if incoming control commands are out
                                       of order (uses sec and nanosec fields)
      AnalyzerResult.COMMAND_MISMATCH: if incoming control commands mis-match
                                       the pre-calculated reference control
                                       commands
      Stats collected from the packet chain consisting of the following:
       - Observed Frequency
       - Avg Jitter
       - Std Deviation
    """
    ret = AnalyzerResult.SUCCESS
    self._deltas = []
    self._count = 0
    synced = False
    prev_cmd_sec = 0
    prev_cmd_nsec = 0
    prev_pkt_sec = 0
    prev_pkt_nsec = 0
    ref_control_iter = iter(self._ref_control_commands)

    while True:
      pkt_sec = 0
      pkt_nsec = 0
      # Checking for start of the chain my comparing sync control command
      if not synced:
        payload, pkt_sec, pkt_nsec = self._get_payload()
        if payload is None:
          if self._user_interrupted:
            ret = AnalyzerResult.USER_INTERRUPTED
          else:
            # Any recv error is considered a hard error
            ret = AnalyzerResult.FAILURE
          break

        command = self._command_from_payload(payload)
        logging.debug(f"({self._count}) Sync command: {self._sync_cmd}")
        if command == self._sync_cmd:
          logging.info(f"({self._count}) Analyzer synced with packet chain")

          # Once sync-ed, packet analyzer will detect any out of order packets,
          # hence storing current packet sec, nanosec
          prev_cmd_sec = command.sec
          prev_cmd_nsec = command.nanosec
          # Once sync-ed, packet analyzer will start computing jitter
          prev_pkt_sec = pkt_sec
          prev_pkt_nsec = pkt_nsec
          synced = True
        else:
          logging.info(f"({self._count}) (Ignored: Not Sync-ed)")
        continue

      # Stop the matching process either
      #  - when there is a mismatch
      #  OR
      #  - where the processing reaches the end of the control commands
      try:
        ref_command = next(ref_control_iter)
      except StopIteration:
        logging.info("All expected control packets received")
        ret = AnalyzerResult.SUCCESS
        break

      # Once the analyzer is in "synced" state, get_payload is called only after
      # checking for end of ref control commands (StopIteration).
      payload, pkt_sec, pkt_nsec = self._get_payload()
      if payload is None:
        # Any recv error is considered a hard error
        ret = AnalyzerResult.FAILURE
        break

      command = self._command_from_payload(payload)
      if not self._rx_packet_in_order(command, prev_cmd_sec, prev_cmd_nsec):
        logging.error(f"({self._count}) Out of Order Packet, Rx: {command}")
        ret = AnalyzerResult.PKT_OUT_OF_ORDER
        break

      logging.debug(f"({self._count}) Command Read: {ref_command}")
      if ref_command != command:
        logging.error(f"({self._count}) Mismatch, Rx: {command}")
        logging.error(f"({self._count}) Mismatch, Expected: {ref_command}")
        ret = AnalyzerResult.COMMAND_MISMATCH
        break
      else:
        # Packet Analyzer will detect any out of order packets, hence storing
        # current packet sec, nanosec
        prev_cmd_sec = command.sec
        prev_cmd_nsec = command.nanosec

        # Compute / store jitter for expected packets and store current
        # values for next iteration
        self._store_delta(prev_pkt_sec, prev_pkt_nsec, pkt_sec, pkt_nsec)
        prev_pkt_sec = pkt_sec
        prev_pkt_nsec = pkt_nsec

    self._log_jitter()
    logging.info(f"End of cycle: {ret}\n")
    return ret

  def run_analyze(self):
    """ Run Packet analyzer on loop

    Performs the following:
    - Connects to the Actuation Service
    - Validates each chain of packets logging the result
    - Does a clean tear off of the TCP socket
    """
    self._connect_to()
    ret = []
    while True:
      chain_ret = self.run_analyze_on_chain()
      ret.append(chain_ret)

      # Break the loop on user interruption or a recv failure
      if chain_ret == AnalyzerResult.FAILURE:
        logging.error("run_analyze_on_chain hard error")
        break
      elif chain_ret == AnalyzerResult.USER_INTERRUPTED:
        logging.info("run_analyze_on_chain interrupted by user")
        break

      # Break if analyzer started without -L
      if not self._run_loop:
        break

    # Attempt a tear only if client has ever connected to the server
    if self._cli_socket_connected:
      self._tear_conn()

    print("\n\nChain ID   Result")
    for chain_id, status in enumerate(ret):
        print("{:<10} {:<10}".format(chain_id, status))

    return ret
