# Copyright (c) 2024-2025, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

FROM ghcr.io/autowarefoundation/autoware:universe-devel-20250207-amd64 AS autoware
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && \
    apt install --no-install-recommends -y unzip python3-pip wget && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Ensure the default ROS middleware is used
RUN echo "unset RMW_IMPLEMENTATION" >> ~/.bashrc

# The gdown version installed in the base image has a breaking bug. Update to
# the next version that has the fix.
# See https://github.com/wkentaro/gdown/issues/291
RUN pip3 install gdown==v4.7.3

FROM autoware AS zephyr
SHELL ["/bin/bash", "-c"]
ARG ZSDK_VERSION=0.16.3
ARG BUILD_ARCH=x86_64
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZSDK_VERSION}

# Install dependencies
RUN apt-get update && \
    apt upgrade -y && \
    apt install --no-install-recommends -y \ 
    ninja-build ccache dfu-util device-tree-compiler wget \
    python3-dev python3-pip python3-setuptools python3-tk screen \ 
    python3-wheel xz-utils file make gcc libsdl2-dev libmagic1 pixz \
    && apt-get install -y gcc-multilib g++-multilib \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install west
RUN pip3 install -U west pyelftools

# Install Zephyr SDK
RUN wget -q --show-progress --progress=bar:force:noscroll \
    https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz && \
    tar --use-compress-program="pixz" -xf zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz -C /opt \
        zephyr-sdk-${ZSDK_VERSION}/arm-zephyr-eabi/ \
        zephyr-sdk-${ZSDK_VERSION}/aarch64-zephyr-elf/ \
        zephyr-sdk-${ZSDK_VERSION}/setup.sh \
        zephyr-sdk-${ZSDK_VERSION}/cmake \
        zephyr-sdk-${ZSDK_VERSION}/sdk_toolchains \
        zephyr-sdk-${ZSDK_VERSION}/sdk_version && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -c -t arm-zephyr-eabi && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -c -t aarch64-zephyr-elf && \
    rm "zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}.tar.xz"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]