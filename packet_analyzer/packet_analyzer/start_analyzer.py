# Copyright (c) 2023, Arm Limited.
#
# SPDX-License-Identifier: MIT

import sys
import time
import signal
import logging
import argparse
from analyzer_helper.analyzer_client import AnalyzerResult
from analyzer_helper.analyzer_client import AnalyzerClient
from analyzer_helper.analyzer_client import ActuationServiceVars

"""
Packet Analyzer is a client program that does the following:
  - Performs a TCP connect against the TCP server running in the Actuation
    Service running in the Safety
  - Performs blocking read on the socket
  - Validates the received Control Commands with pre-calculated Control
    Commands stored in csv file
"""


def main():
  if sys.version_info < (3, 8):
    raise ValueError("This script requires Python 3.8 or later")

  port = ActuationServiceVars.ACTUATION_SERVICE_PORT
  parser = \
    argparse.ArgumentParser(description="Start Packet Analyzer module,"
                                        " use Ctrl-C to stop the process.")
  parser.add_argument("-p", "--port", nargs="?", const=port, type=int,
                      default=port,
                      help=f"TCP port to use (Default: {port})")
  parser.add_argument("-L", "--loglevel", nargs="?", const="info",
                      default="info",
                      choices=['error', 'info', 'debug'],
                      help="Set loglevel (Default: info)")
  parser.add_argument("-c", "--control_command_path", nargs="?",
                      const="./data/", default="./data",
                      help=f"Path to the directory containing \
                             ControlCommand.csv (Default: './data')")
  parser.add_argument("-l", "--loop", action='store_true',
                      help=f"Run Packet Analyzer in a loop (Default: False)")
  required_args = parser.add_argument_group('required arguments')
  required_args.add_argument('-a', '--actuation_host',
                             help='Actuation Service hostname / IP Address',
                             required=True)
  args = parser.parse_args()
  fmt_s = '%(levelname)-5s: %(filename)s/%(funcName)s: %(message)s'
  logging.basicConfig(level=args.loglevel.upper(), format=fmt_s)

  cli = AnalyzerClient(args.actuation_host, args.port,
                       args.control_command_path, args.loop)

  # Add a signal handler for signal.SIGINT that does an explicit cleanup of
  # AnalyzerClient.
  def handler(signum, frame):
    cli.stop_run()

  # Catching user interrupt and doing graceful exit.
  signal.signal(signal.SIGINT, handler)
  cli.run_analyze()


if __name__ == "__main__":
  main()
