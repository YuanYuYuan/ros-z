FROM ros:jazzy-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    libclang-dev \
    libyaml-dev \
    ros-jazzy-rcl \
    ros-jazzy-test-msgs \
    ros-jazzy-mimick-vendor \
    ros-jazzy-osrf-testing-tools-cpp \
    ros-jazzy-rcpputils \
    ros-jazzy-fastcdr \
    ros-jazzy-rosidl-typesupport-fastrtps-c \
    ros-jazzy-rosidl-typesupport-fastrtps-cpp \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /workspace
