#!/bin/bash
set -e

echo "=== rmw_zenoh_rs Executor Tests ==="
echo "Date: $(date)"
echo "ROS Distro: jazzy"
echo ""

# Setup
RESULTS_DIR=/workspace/test-results
mkdir -p $RESULTS_DIR

# Source ROS
source /opt/ros/jazzy/setup.bash

# Install dependencies
echo "=== Installing system dependencies ==="
apt-get update
apt-get install -y \
    curl \
    build-essential \
    libclang-dev \
    ros-jazzy-rclcpp \
    ros-jazzy-test-msgs \
    ros-jazzy-mimick-vendor \
    ros-jazzy-osrf-testing-tools-cpp \
    ros-jazzy-ament-cmake-google-benchmark \
    ros-jazzy-performance-test-fixture \
    ros-jazzy-rcpputils \
    ros-jazzy-fastcdr \
    ros-jazzy-rosidl-typesupport-fastrtps-c \
    ros-jazzy-rosidl-typesupport-fastrtps-cpp \
    git

# Install Rust
echo "=== Installing Rust ==="
if ! command -v cargo &> /dev/null; then
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
fi
export PATH="$HOME/.cargo/bin:$PATH"
rustc --version
cargo --version

# Create build workspace
echo "=== Creating workspace ==="
rm -rf /workspace/ws
mkdir -p /workspace/ws/src
cd /workspace/ws/src

# Copy ros-z sources (excluding _tmp and other non-essential directories)
echo "=== Copying ros-z sources ==="
cp -r /workspace/ros-z /workspace/ws/src/ros-z

# Add COLCON_IGNORE to directories that shouldn't be scanned
for dir in _tmp book .github scripts ros-z-tests ros-z-console ros-z-py assets nix .config .claude; do
    if [ -d "/workspace/ws/src/ros-z/$dir" ]; then
        touch "/workspace/ws/src/ros-z/$dir/COLCON_IGNORE"
    fi
done

# Copy local rclcpp fork (with patches)
echo "=== Copying local rclcpp fork ==="
cp -r /workspace/rclcpp /workspace/ws/src/rclcpp

# Copy local rcl fork (with patches)
echo "=== Copying local rcl fork ==="
cp -r /workspace/rcl /workspace/ws/src/rcl

# Build rmw_zenoh_rs
echo "=== Building rmw_zenoh_rs ==="
cd /workspace/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rmw_zenoh_rs --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the built workspace
source /workspace/ws/install/setup.bash

# Build rcl and rclcpp with tests (colcon resolves dependencies)
echo "=== Building rcl and rclcpp with tests ==="
colcon build --packages-up-to rclcpp --cmake-args -DBUILD_TESTING=ON

# Source again after building
source /workspace/ws/install/setup.bash

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_zenoh_rs

echo ""
echo "=== Verifying RMW implementation ==="
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# Check if rmw_zenoh_rs library exists
if [ -f "/workspace/ws/install/rmw_zenoh_rs/lib/librmw_zenoh_rs.so" ]; then
    echo "rmw_zenoh_rs library found!"
else
    echo "ERROR: rmw_zenoh_rs library not found!"
    ls -la /workspace/ws/install/rmw_zenoh_rs/lib/ 2>/dev/null || echo "Directory not found"
    exit 1
fi

echo ""
echo "=== Running Executor Tests with rmw_zenoh_rs ==="
echo ""

# Navigate to build directory for tests
cd /workspace/ws/build/rclcpp

# Run executor tests and capture results
TESTS=(
    "test_executors"
    "test_executors_intraprocess"
    "test_executors_busy_waiting"
    "test_executors_warmup"
    "test_executors_timer_cancel_behavior"
    "test_executors_callback_group_behavior"
)

for test in "${TESTS[@]}"; do
    echo ""
    echo "=== Running $test ==="

    # Run test with timeout and capture output
    if timeout 180 ctest -R "^${test}$" -V 2>&1 | tee "$RESULTS_DIR/${test}.log"; then
        echo "RESULT: $test PASSED"
        echo "PASSED" >> "$RESULTS_DIR/${test}.result"
    else
        EXIT_CODE=$?
        echo "RESULT: $test FAILED (exit code: $EXIT_CODE)"
        echo "FAILED (exit code: $EXIT_CODE)" >> "$RESULTS_DIR/${test}.result"
    fi
done

# Summary
echo ""
echo "=== Test Summary ==="
echo ""
for test in "${TESTS[@]}"; do
    if [ -f "$RESULTS_DIR/${test}.result" ]; then
        echo "$test: $(cat $RESULTS_DIR/${test}.result)"
    else
        echo "$test: NOT RUN"
    fi
done

echo ""
echo "=== Detailed logs saved to $RESULTS_DIR ==="
ls -la $RESULTS_DIR/
