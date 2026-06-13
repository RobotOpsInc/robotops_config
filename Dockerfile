# Build environment for librobotops-config.so (the protobuf-backed C++ lib)
# and its ROS 2 Debian package.
#
# Parameterized by ROS distro so a single Dockerfile builds for either
# Jazzy (Ubuntu 24.04 Noble) or Humble (Ubuntu 22.04 Jammy, the arm64/Jetson
# target). Pass --build-arg ROS_DISTRO=humble for the Humble build.
#
#   ROS_DISTRO  ROS 2 distribution (jazzy | humble). Selects the
#               `ros:${ROS_DISTRO}` base image and /opt/ros/${ROS_DISTRO}.
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

# Re-declare after FROM so the value is in scope in the build stage.
ARG ROS_DISTRO

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Install all dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-pip \
    git \
    curl \
    dpkg-dev \
    fakeroot \
    debhelper \
    protobuf-compiler \
    libprotobuf-dev \
    python3-protobuf \
    && rm -rf /var/lib/apt/lists/*

# Install bloom via pip (not available in the Ubuntu apt repos).
# Ubuntu 24.04 (Noble/Jazzy) ships a PEP 668 "externally managed" pip that
# requires --break-system-packages; Ubuntu 22.04 (Jammy/Humble) does not have
# that marker and its older pip does not understand the flag. Try the modern
# invocation first and fall back so a single Dockerfile works on both.
RUN pip3 install --break-system-packages bloom || pip3 install bloom

# Update rosdep database
RUN rosdep update

# Install buf (for protobuf code generation with correct protoc version)
RUN ARCH=$(dpkg --print-architecture) && \
    if [ "$ARCH" = "amd64" ]; then BUF_ARCH="x86_64"; else BUF_ARCH="aarch64"; fi && \
    curl -sSL "https://github.com/bufbuild/buf/releases/download/v1.28.1/buf-Linux-${BUF_ARCH}" -o /usr/local/bin/buf && \
    chmod +x /usr/local/bin/buf

# Create workspace structure
RUN mkdir -p /ws/src

# Copy source code
COPY . /ws/src/robotops-config

WORKDIR /ws/src/robotops-config

# Default command
CMD ["/bin/bash"]
