# Copyright (c) 2022-2024, Arm Limited.
# SPDX-License-Identifier: Apache-2.0

ARG BASE_IMAGE=ghcr.io/autowarefoundation/autoware:universe-devel-20250204-amd64

FROM $BASE_IMAGE AS autoware

ARG USER=root
ARG UID=0

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

# Add user
RUN if [[ "$USER" != "root" ]]; then \
        useradd --no-log-init -u "$UID" -m -d "/home/$USER" -s "/bin/bash" "$USER" && \
        echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/actuation \
        ; \
    fi

USER $UID

FROM autoware AS zephyr

USER root

ARG ZSDK_VERSION=0.16.3
ARG BUILD_ARCH=x86_64

# Install dependencies
RUN apt-get update && \
    apt install --no-install-recommends -y ninja-build && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install west
RUN pip3 install -U west

ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZSDK_VERSION}

# Install Zephyr SDK
RUN wget -q --show-progress --progress=bar:force:noscroll \
    https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}_minimal.tar.xz && \
    tar xf zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}_minimal.tar.xz -C /opt && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -t aarch64-zephyr-elf && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -t arm-zephyr-eabi && \
    ${ZEPHYR_SDK_INSTALL_DIR}/setup.sh -c && \
    rm "zephyr-sdk-${ZSDK_VERSION}_linux-${BUILD_ARCH}_minimal.tar.xz"

USER $UID