FROM ros:jazzy

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
    python3-protobuf \
    && rm -rf /var/lib/apt/lists/*

# Install bloom via pip (not available in Ubuntu 24.04 repos)
RUN pip3 install --break-system-packages bloom

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
