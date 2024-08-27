# Copyright (c) 2023, Arm Limited.
#
# SPDX-License-Identifier: MIT

import sys
import time
import errno
import random
import struct
import socket
import logging
import argparse
import unittest
import numpy as np
import concurrent.futures
from analyzer_helper.analyzer_client import AnalyzerResult
from analyzer_helper.analyzer_client import AnalyzerClient
from analyzer_helper.analyzer_client import ActuationServiceVars
from analyzer_helper.control_commands import RefControlCommands


"""
Test program to send control commands as TCP packets based on
./ControlCommand.csv. This program can be used in the following ways:
 - manually (python packet_analyzer/test_packet_analyzer.py)
 OR
 - via python unittest (python -m unittest test_packet_analyzer.py)
"""


def get_sec_nanosec():
  """ Function to get seconds / nanosecond from ts

  :param ts: timestamp in np.uint64
  :return: sec, nanosec tuple
  """
  ts = time.time_ns()
  sec = (np.int32)(ts // 1000000000)
  nanosec = (np.uint32)(ts % 1000000000)
  return sec, nanosec


def init_socket(host, port):
  """ Function to create a TCP socket

  :param host: host
  :param port: TCP port number
  """
  srv_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
  srv_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  srv_socket.bind((host, port))
  srv_socket.listen()
  return srv_socket


def tear_socket(cli_socket):
  """ Function to perform a clean tearoff with the TCP client

  :param cli_socket: client connection
  """
  # Wait for a FIN from the client in order for a clean tear off
  buf_len = ActuationServiceVars.CONTROL_COMMAND_PAYLOAD_LEN
  buf = bytearray(buf_len)
  mview = memoryview(buf)
  while True:
    # Get a full packet before checking for FIN
    buf[:] = b'\x00' * len(buf)
    mview = memoryview(buf)
    while (buf_len):
      bytes_read = cli_socket.recv_into(mview, buf_len)
      mview = mview[bytes_read:]
      buf_len -= bytes_read

    # Check if received payload is FIN, send a ACK back if that is the case.
    if ActuationServiceVars.PACKET_ANALYZER_FIN in buf:
      logging.info('Received FIN from client')
      break
    else:
      logging.error(f"Rx {buf}, but expecting a FIN")
      exit(1)

  # Send fin ack back to client
  buf[:] = b'\x00' * len(buf)
  mview = memoryview(buf)
  fin_ack_len = len(ActuationServiceVars.PACKET_ANALYZER_FIN_ACK)
  mview[:fin_ack_len] = ActuationServiceVars.PACKET_ANALYZER_FIN_ACK
  send_command(cli_socket, buf)
  cli_socket.close()


def send_command(cli_socket, payload):
  """ Send payload on a TCP socket

  :param cli_socket: Client fd to use to send packets
  :param payload: payload in byte object format
  """
  cli_socket.sendall(payload)


def test_generate_control_commands(host, port, control_command_path='./data',
                                   case=AnalyzerResult.SUCCESS, repeat=1):
  """
  Sends TCP packets based on the ref control commands in order to the
    Packet Analyzer module
  :param host: host
  :param port: TCP port
  :param control_command_path: Path containing ControlCommand.csv
  :param case: Change the behaviour of the function as follows:
     AnalyzerResult.SUCCESS: no change in behaviour
      AnalyzerResult.PKT_OUT_OF_ORDER: inject a out of order packet
      AnalyzerResult.COMMAND_MISMATCH: inject a command mismatch packet
  :param repeat: Number of times to send the packet chain
  """
  srv_socket = init_socket(host, port)

  # Wait for connect from the client
  logging.info("Performing a blocking accept for client to connect")
  cli_socket, cli_addr = srv_socket.accept()
  logging.info(f"Rcvd connection from client: {cli_socket}")

  # Server socket can be closed right after client socket spawning
  srv_socket.close()

  # Get a handle towards reference control commands
  ref_control_commands = RefControlCommands(control_command_path)

  while (repeat):
    repeat -= 1
    # Send the sync control packet first
    command = ref_control_commands.get_sync_command()
    sec, nanosec = get_sec_nanosec()
    payload = struct.pack("<iIffff", sec, nanosec, command.accel, command.vel,
                          command.f_angle, command.r_angle)
    send_command(cli_socket, payload)
    logging.info("Sending sync control command on port " + str(port))

    # Send rest of control packets stored in the received csv file
    count = 1
    for command in iter(ref_control_commands):
      time.sleep(random.uniform(.024001, .024999))
      sec, nanosec = get_sec_nanosec()

      # Inject a COMMAND_MISMATCH by setting control command to 0
      if count == 100 and case == AnalyzerResult.COMMAND_MISMATCH:
        logging.info("Injecting Control Command Mismatch")
        command.accel = 0
        command.vel = 0
        command.f_angle = 0
        command.r_angle = 0

      # Inject a out of order error by setting sec (seconds after epoch) to 100
      if count == 100 and case == AnalyzerResult.PKT_OUT_OF_ORDER:
          logging.info("Injecting Out of Order Control Command")
          sec = 100

      payload = struct.pack("<iIffff", sec, nanosec, command.accel, command.vel,
                            command.f_angle, command.r_angle)
      try:
        send_command(cli_socket, payload)
      except Exception as err:
        logging.error(f"Unexpected exception: {err}")
        raise err

      count += 1

      # In case of COMMAND_MISMATCH or PKT_OUT_OF_ORDER, stop send only 2
      # after the error injection
      if count > 102 and case in [AnalyzerResult.COMMAND_MISMATCH,
                                  AnalyzerResult.PKT_OUT_OF_ORDER]:
        break
    logging.info("Successfully sent {} control commands on port {}"
                 "".format(count, port))

  # Do a clean TCP socket tear off
  tear_socket(cli_socket)


def run_client_server(host, port, control_command_path, case):
  """
  Run Packet Analyzer server in the background and send TCP packets based
    on the ref control commands to the test the Packet Analyzer module
  :param host: host
  :param port: TCP port
  :param case: Change the behaviour of the function as follows:
     AnalyzerResult.SUCCESS: no change in behaviour
      AnalyzerResult.PKT_OUT_OF_ORDER: inject a out of order packet
      AnalyzerResult.COMMAND_MISMATCH: inject a command mismatch packet
  :return: return code as returned by AnalyzerClient.run_analyze
  """
  ret = None
  with concurrent.futures.ThreadPoolExecutor() as executor:
    srv = AnalyzerClient(host, port, control_command_path)
    future = executor.submit(srv.run_analyze)
    test_generate_control_commands(host, port, control_command_path, case)

    logging.info("Waiting for the Packet Analyzer thread to join")
    # Retrieve the result of the packet chain analyze
    ret = future.result()

  return ret[0] if len(ret) else None


class Test(unittest.TestCase):
  """ Test class that captures various test cases
  """
  fmt_s = '%(levelname)-5s: %(filename)s/%(funcName)s: %(message)s'
  logging.basicConfig(level=logging.DEBUG, format=fmt_s)

  def test_deps(self):
    logging.info("Test if python version > 3.8")
    self.assertGreater(sys.version_info, (3, 8))

  def test_run_normal(self):
    logging.info("Run packet analyzer server and send all control commands"
                 " read from the csv file")
    self.assertEqual(
      run_client_server("localhost",
                        ActuationServiceVars.ACTUATION_SERVICE_PORT,
                        './data/test_data',
                        AnalyzerResult.SUCCESS),
      AnalyzerResult.SUCCESS)

  def test_run_out_of_order(self):
    logging.info("Run packet analyzer server and inject an out of order"
                 " control command")
    self.assertEqual(
      run_client_server("localhost",
                        ActuationServiceVars.ACTUATION_SERVICE_PORT + 1,
                        './data',
                        AnalyzerResult.PKT_OUT_OF_ORDER),
      AnalyzerResult.PKT_OUT_OF_ORDER)

  def test_run_command_mismatch(self):
    logging.info("Run packet analyzer server and change the order of"
                 " control commands")
    self.assertEqual(
      run_client_server("localhost",
                        ActuationServiceVars.ACTUATION_SERVICE_PORT + 2,
                        './data',
                        AnalyzerResult.COMMAND_MISMATCH),
      AnalyzerResult.COMMAND_MISMATCH)


def main():
  if sys.version_info < (3, 8):
    raise ValueError("This script requires Python 3.8 or later")

  port = ActuationServiceVars.ACTUATION_SERVICE_PORT
  parser = \
    argparse.ArgumentParser(description="Send Control Commands on TCP socket")
  parser.add_argument("-p", "--port", nargs="?", const=port, type=int,
                      default=port,
                      help=f"TCP port to use (Default: {port})")
  parser.add_argument("-L", "--loglevel", nargs="?", const="info",
                      default="info",
                      choices=['error', 'info'],
                      help="Set loglevel (Default: info)")
  parser.add_argument('-a', '--actuation_host',
                      default="localhost",
                      help='Actuation Service hostname / IP Address')
  parser.add_argument("-c", "--control_command_path", nargs="?",
                      const="./data/", default="./data",
                      help=f"Path to the directory containing \
                             ControlCommand.csv (Default: './data'")
  parser.add_argument("-r", "--repeat", nargs="?", const=1, type=int,
                      default=1,
                      help=f"Number of times to send packet chain(Default: 1)")
  args = parser.parse_args()
  fmt_s = '%(levelname)-5s: %(filename)s/%(funcName)s: %(message)s'
  logging.basicConfig(level=args.loglevel.upper(), format=fmt_s)

  # Create TCP server and send test control commands on TCP port
  test_generate_control_commands(args.actuation_host, args.port,
                                 args.control_command_path,
                                 AnalyzerResult.SUCCESS, args.repeat)


if __name__ == "__main__":
  main()
