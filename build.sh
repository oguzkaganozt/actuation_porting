#! /usr/bin/env bash

# Copyright (c) 2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0
#
# Build script for the Zephyr Actuation Module
#
# This script builds the Zephyr Actuation Module for the specified target board.
#
# Usage: ./build.sh [OPTIONS]

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

# Root directory
ROOT_DIR=$(dirname "$(realpath "$0")")
set -e
set -u

# Build options
BUILD_TEST_FLAG=0
ZEPHYR_TARGET_LIST=("fvp_baser_aemv8r_smp" "s32z270dc2_rtu0_r52@D")
ZEPHYR_TARGET=${ZEPHYR_TARGET_LIST[0]} # Default target is FVP

function usage() {
  echo -e "${GREEN}Usage: $0 [OPTIONS]${NC}"
  echo -e "------------------------------------------------"
  echo -e "${GREEN}    -t    ${NC}Zephyr target board: ${ZEPHYR_TARGET_LIST[*]}"
  echo -e "${GREEN}            default: ${ZEPHYR_TARGET_LIST[0]} (FVP).${NC}"
  echo -e "${GREEN}    -c    ${NC}Clean all builds and exit."
  echo -e "${GREEN}    -h    ${NC}Display the usage and exit."
  echo -e "${GREEN}    Optional arguments to build Zephyr test programs:${NC}"
  echo -e "${GREEN}    --unit-test    ${NC}Build Zephyr unit test program."
  echo -e "${GREEN}    --dds-publisher    ${NC}Build Zephyr DDS publisher."
  echo -e "${GREEN}    --dds-subscriber    ${NC}Build Zephyr DDS subscriber."
}

function parse_args() {
  new_args=()
  for arg in "$@"; do
    case $arg in
      --unit-test)
        BUILD_TEST_FLAG=1
        shift
        ;;
      --dds-publisher)
        BUILD_TEST_FLAG=2
        shift
        ;;
      --dds-subscriber)
        BUILD_TEST_FLAG=3
        shift
        ;;
      *)
        new_args+=("$arg")
        ;;
    esac
  done
  set -- "${new_args[@]}" # Reset the positional parameters to the remaining arguments

  while getopts "t:ch" opt; do
    case ${opt} in
      t )
        ZEPHYR_TARGET=""
        for t in "${ZEPHYR_TARGET_LIST[@]}"; do
          if [ "${t}" = "${OPTARG}" ]; then
            ZEPHYR_TARGET=${t}
            break
          fi
        done
        if [ -z "${ZEPHYR_TARGET}" ]; then
          echo -e "${RED}Invalid Zephyr target: ${OPTARG}${NC}\n" 1>&2
          echo -e "${YELLOW}Valid targets: ${ZEPHYR_TARGET_LIST[*]}${NC}" 1>&2
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
        echo -e "${RED}Invalid option: ${OPTARG}${NC}\n" 1>&2
        usage
        exit 1
        ;;
    esac
  done
  shift $((OPTIND -1))
}

function clean() {
  rm -rf "${ROOT_DIR}"/build "${ROOT_DIR}"/install
}

function build_cyclonedds_host() {
  echo -e "${GREEN}Building CycloneDDS host tools...${NC}"
  mkdir -p build/cyclonedds_host
  pushd build/cyclonedds_host
  cmake -DCMAKE_INSTALL_PREFIX="$(pwd)"/out -DENABLE_SECURITY=OFF -DENABLE_SSL=OFF -DBUILD_IDLC=ON -DBUILD_SHARED_LIBS=ON -DENABLE_SHM=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_DDSPERF=OFF "${ROOT_DIR}"/cyclonedds
  cmake --build . --target install -- -j"$(nproc)"
  popd
}

function build_actuation_module() {
  echo -e "${GREEN}Building Zephyr Actuation Module...${NC}"
  typeset PATH="${ROOT_DIR}"/build/cyclonedds_host/out/bin:$PATH
  typeset LD_LIBRARY_PATH="${ROOT_DIR}"/build/cyclonedds_host/out/lib
  typeset CMAKE_PREFIX_PATH=""
  typeset AMENT_PREFIX_PATH=""

  # Build command with common arguments
  local build_args=(
    -DZEPHYR_TARGET="${ZEPHYR_TARGET}"
    -DCYCLONEDDS_SRC="${ROOT_DIR}"/cyclonedds
    -DEXTRA_CFLAGS="-Wno-error"
    -DEXTRA_CXXFLAGS="-Wno-error"
    -DBUILD_TEST=${BUILD_TEST_FLAG}
  )

  # Add device tree overlay only for ARM board variant
  if [ "${ZEPHYR_TARGET}" = "s32z270dc2_rtu0_r52@D" ]; then
    build_args+=(-DEXTRA_DTC_OVERLAY_FILE="${ROOT_DIR}"/actuation_module/boards/s32z270dc2_rtu0_r52@D.overlay)
  fi

  west build -p auto -d build/actuation_module -b "${ZEPHYR_TARGET}" actuation_module/ -- "${build_args[@]}"
}

## MAIN ##
parse_args "$@"

# Create build directory
cd "${ROOT_DIR}"
mkdir -p build

# Build CycloneDDS host tools
build_cyclonedds_host

# Build Zephyr Actuation Module
build_actuation_module
