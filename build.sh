#! /usr/bin/env bash

# Copyright (c) 2022-2024, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

set -e
set -u

CONTROL_COMMANDS_CHANNEL="dds"
HOST_TOOLS_ONLY=0
AUTOWARE_ONLY=0
ZEPHYR_ONLY=0
PARTIAL_BUILD=0

ZEPHYR_TARGET_LIST=("s32z270dc2_rtu0_r52" "fvp_baser_aemv8r_smp")
ZEPHYR_TARGET=${ZEPHYR_TARGET_LIST[0]}

ROOT_DIR=$(dirname "$(realpath "$0")")

function usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "    -d    Only build the CycloneDDS host tools."
  echo "    -z    Only build the Zephyr app."
  echo "    -b    Use bsd socket for Control Command forward (default: dds)."
  echo "    -a    Only build the Autoware packages."
  echo "    -t    Zephyr target: ${ZEPHYR_TARGET_LIST[*]}"
  echo "            default: ${ZEPHYR_TARGET_LIST[0]}."
  echo "    -c    Clean all builds and exit."
  echo "    -h    Display the usage and exit."
}

function clean() {
  rm -rf "${ROOT_DIR}"/build "${ROOT_DIR}"/install "${ROOT_DIR}"/log
}

while getopts "dzbat:ch" opt; do
  case ${opt} in
    d )
      HOST_TOOLS_ONLY=1
      PARTIAL_BUILD=1
      ;;
    z )
      ZEPHYR_ONLY=1
      PARTIAL_BUILD=1
      ;;
    b )
      CONTROL_COMMANDS_CHANNEL="bsd_socket"
      ;;
    a )
      AUTOWARE_ONLY=1
      PARTIAL_BUILD=1
      ;;
    t )
      ZEPHYR_TARGET=""
      for t in "${ZEPHYR_TARGET_LIST[@]}"; do
        if [ "${t}" = "${OPTARG}" ]; then
          ZEPHYR_TARGET=${t}
          break
        fi
      done
      if [ -z "${ZEPHYR_TARGET}" ]; then
        echo -e "Invalid Zephyr target: ${OPTARG}\n" 1>&2
        exit 1
      fi
      ;;
    c )
      clean
      exit 0
      ;;
    h )
      usage
      exit 0
      ;;
    \? )
      echo -e "Invalid option: ${OPTARG}\n" 1>&2
      usage
      exit 1
      ;;
  esac
done
shift $((OPTIND -1))

cd "${ROOT_DIR}"
mkdir -p build

if [ "${PARTIAL_BUILD}" = "0" ] || [ "${HOST_TOOLS_ONLY}" = "1" ]; then
  # Build CycloneDDS host tools
  mkdir -p build/cyclonedds_host
  pushd build/cyclonedds_host
  cmake -DCMAKE_INSTALL_PREFIX="$(pwd)"/out -DENABLE_SECURITY=OFF -DENABLE_SSL=OFF -DBUILD_IDLC=ON -DBUILD_SHARED_LIBS=ON -DENABLE_SHM=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF "${ROOT_DIR}"/cyclonedds
  cmake --build . --target install -- -j"$(nproc)"
  popd
fi

if [ "${PARTIAL_BUILD}" = "0" ] || [ "${ZEPHYR_ONLY}" = "1" ]; then
  # Build Zephyr elf
  function build_zephyr() {
    # Pure pursuit forwards Control Commands to host via dds_write. Use flag
    # DCONTROL_CMDS_FWD=bsd_socket to override this behaviour to use bsd sockets
    typeset PATH="${ROOT_DIR}"/build/cyclonedds_host/out/bin:$PATH
    typeset LD_LIBRARY_PATH="${ROOT_DIR}"/build/cyclonedds_host/out/lib
    typeset CMAKE_PREFIX_PATH=""
    typeset AMENT_PREFIX_PATH=""
    west build -p auto -d build/zephyr_app -b "${ZEPHYR_TARGET}" zephyr_app/ -- -DCYCLONEDDS_SRC="${ROOT_DIR}"/cyclonedds -DCONTROL_CMDS_FWD=${CONTROL_COMMANDS_CHANNEL}
  }
  build_zephyr
fi

# if [ "${PARTIAL_BUILD}" = "0" ] || [ "${AUTOWARE_ONLY}" = "1" ]; then
#   # Build packages for the Autoware demo
#   colcon build --packages-select actuation_msgs --build-base build/autoware --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON --base-paths actuation_packages
# fi

# if [ "${PARTIAL_BUILD}" = "0" ] || [ "${AUTOWARE_ONLY}" = "1" ]; then
#   # Build packages for the Autoware demo
#   colcon build --packages-select actuation_demos actuation_message_converter actuation_msgs --build-base build/autoware --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON --base-paths actuation_packages
# fi