FROM ubuntu:24.04

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-pip \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Debian build tools
RUN apt-get update && apt-get install -y \
    python3-bloom \
    dpkg-dev \
    fakeroot \
    debhelper \
    && rm -rf /var/lib/apt/lists/*

# Install protobuf
RUN apt-get update && apt-get install -y \
    protobuf-compiler \
    libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace structure
RUN mkdir -p /ws/src

# Copy source code
COPY . /ws/src/robotops-config

WORKDIR /ws/src/robotops-config

# Default command
CMD ["/bin/bash"]
