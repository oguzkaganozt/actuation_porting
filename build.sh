#! /usr/bin/env bash

# Copyright (c) 2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

set -e
set -u

AUTOWARE_ONLY=0
ZEPHYR_ONLY=0
PARTIAL_BUILD=0
BUILD_TEST_FLAG=0

ZEPHYR_TARGET_LIST=("s32z270dc2_rtu0_r52" "native_sim" "fvp_baser_aemv8r_smp")
ZEPHYR_TARGET=${ZEPHYR_TARGET_LIST[0]}

ROOT_DIR=$(dirname "$(realpath "$0")")

function usage() {
  echo "Usage: $0 [OPTIONS]"
  echo "    -d    Only build the CycloneDDS host tools."
  echo "    -z    Only build the Zephyr app."
  echo "    -a    Only build the Autoware packages."
  echo "    -t    Zephyr target: ${ZEPHYR_TARGET_LIST[*]}"
  echo "            default: ${ZEPHYR_TARGET_LIST[0]}."
  echo "    --unit-test    Build the unit test programs."
  echo "    --dds-publisher    Build the DDS publisher."
  echo "    --dds-subscriber    Build the DDS subscriber."
  echo "    -c    Clean all builds and exit."
  echo "    -h    Display the usage and exit."
}

function clean() {
  rm -rf "${ROOT_DIR}"/build "${ROOT_DIR}"/install "${ROOT_DIR}"/log
}

# Manual parsing for long options like --test
new_args=()
for arg in "$@"; do
  case $arg in
    --unit-test)
      BUILD_TEST_FLAG=1
      shift # Remove --test from processing
      ;;
    --dds-publisher)
      BUILD_TEST_FLAG=2
      shift # Remove --dds-publisher from processing
      ;;
    --dds-subscriber)
      BUILD_TEST_FLAG=3
      shift # Remove --dds-subscriber from processing
      ;;
    *)
      new_args+=("$arg") # Keep other arguments
      ;;
  esac
done
# Reset the positional parameters to the remaining arguments
set -- "${new_args[@]}"

while getopts "zat:ch" opt; do
  case ${opt} in
    z )
      PARTIAL_BUILD=1
      ZEPHYR_ONLY=1
      ;;
    a )
      PARTIAL_BUILD=1
      AUTOWARE_ONLY=1
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
        echo "Valid targets: ${ZEPHYR_TARGET_LIST[*]}" 1>&2
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

function build_cyclonedds_host() {
  # Build CycloneDDS host tools
  mkdir -p build/cyclonedds_host
  pushd build/cyclonedds_host
  cmake -DCMAKE_INSTALL_PREFIX="$(pwd)"/out -DENABLE_SECURITY=OFF -DENABLE_SSL=OFF -DBUILD_IDLC=ON -DBUILD_SHARED_LIBS=ON -DENABLE_SHM=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF "${ROOT_DIR}"/cyclonedds
  cmake --build . --target install -- -j"$(nproc)"
  popd
}

function build_zephyr() {
  typeset PATH="${ROOT_DIR}"/build/cyclonedds_host/out/bin:$PATH
  typeset LD_LIBRARY_PATH="${ROOT_DIR}"/build/cyclonedds_host/out/lib
  typeset CMAKE_PREFIX_PATH=""
  typeset AMENT_PREFIX_PATH=""

  if [ "${ZEPHYR_TARGET}" = "native_sim" ]; then
    echo "Building CYCLONEDDS for native simulator"
    mkdir -p build/cyclonedds_native_sim
    pushd build/cyclonedds_native_sim
    cmake_flags="-DCMAKE_C_FLAGS= \
      -fno-pic \
      -m32"
    cmake -DCMAKE_INSTALL_PREFIX="$(pwd)"/out -DENABLE_SECURITY=OFF -DENABLE_SSL=OFF -DBUILD_IDLC=OFF -DBUILD_SHARED_LIBS=OFF -DENABLE_SHM=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF "${cmake_flags}" "${ROOT_DIR}"/cyclonedds
    cmake --build . --target install -- -j"$(nproc)"
    popd
    typeset LD_LIBRARY_PATH="${ROOT_DIR}"/build/cyclonedds_native_sim/out/lib
  fi

  # Build Zephyr elf
  west build -p auto -d build/actuation_module -b "${ZEPHYR_TARGET}" actuation_module/ -- \
    -DZEPHYR_TARGET="${ZEPHYR_TARGET}" \
    -DCYCLONEDDS_SRC="${ROOT_DIR}"/cyclonedds \
    -DEXTRA_CFLAGS="-Wno-error" \
    -DEXTRA_CXXFLAGS="-Wno-error" \
    -DBUILD_TEST=${BUILD_TEST_FLAG}
}

build_cyclonedds_host
build_zephyr

# if [ "${PARTIAL_BUILD}" = "0" ] || [ "${AUTOWARE_ONLY}" = "1" ]; then
#   # Build packages for the Autoware demo
#   colcon build --packages-select actuation_msgs --build-base build/autoware --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON --base-paths autoware_packages
# fi

# if [ "${PARTIAL_BUILD}" = "0" ] || [ "${AUTOWARE_ONLY}" = "1" ]; then
#   # Build packages for the Autoware demo
#   colcon build --packages-select actuation_demos actuation_message_converter actuation_msgs --build-base build/autoware --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON --base-paths autoware_packages
# fi