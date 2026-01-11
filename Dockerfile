FROM ubuntu:24.04

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

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
    && rm -rf /var/lib/apt/lists/*

# Install bloom via pip (not available in Ubuntu 24.04 repos)
RUN pip3 install --break-system-packages bloom

# Create workspace structure
RUN mkdir -p /ws/src

# Copy source code
COPY . /ws/src/robotops-config

WORKDIR /ws/src/robotops-config

# Default command
CMD ["/bin/bash"]
