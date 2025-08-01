# cspell:ignore openbox, VNC, tigervnc, novnc, websockify, newkey, xstartup, keyout
FROM ghcr.io/autowarefoundation/autoware:universe-20250207 AS visualizer
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG ROS_DISTRO
WORKDIR /autoware

# Fix expired ROS GPG key issue comprehensively
# See: https://github.com/osrf/docker_images/issues/535
RUN apt-get update --allow-unauthenticated || true && \
    apt-get install -y --allow-unauthenticated curl gnupg2 && \
    apt-key del F42ED6FBAB17C654 || true && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key && \
    cat /tmp/ros.key | gpg --dearmor --batch --yes -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    rm /tmp/ros.key

# Install openbox, VNC, and simulator dependencies
# hadolint ignore=SC2002
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
  curl \
  unzip \
  openbox \
  tigervnc-standalone-server \
  tigervnc-common \
  novnc \
  websockify \
  python3-numpy \
  python3-xdg \
  && pip install --no-cache-dir yamale xmlschema 

# Create SSL certificate for NoVNC
RUN openssl req -x509 -nodes -newkey rsa:2048 \
  -keyout /etc/ssl/private/novnc.key \
  -out /etc/ssl/certs/novnc.crt \
  -days 365 \
  -subj "/O=Autoware-OpenADKit/CN=localhost"

# hadolint ignore=SC1091
RUN source /opt/autoware/setup.bash \
  && RVIZ_PATH="$(ros2 pkg prefix --share autoware_launch)/rviz" \
  && cp -r "$RVIZ_PATH" /autoware/rviz

# Need to expose VNC and NoVNC ports when running the container
EXPOSE 5900 6080

# Add source commands to bash startup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
  echo "source /opt/autoware/setup.bash" >> /root/.bashrc

# Copy startup scripts
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY visualizer/xstartup /root/.vnc/xstartup
COPY visualizer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chmod +x /root/.vnc/xstartup
ENTRYPOINT ["/entrypoint.sh"]
