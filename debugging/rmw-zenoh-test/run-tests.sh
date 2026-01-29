#!/bin/bash
set -e

echo "=== rmw_zenoh_cpp Executor Tests ==="
echo "Date: $(date)"
echo "ROS Distro: jazzy"
echo ""

# Setup
RESULTS_DIR=/workspace/test-results
mkdir -p $RESULTS_DIR

# Source ROS
source /opt/ros/jazzy/setup.bash

# Install dependencies
echo "=== Installing dependencies ==="
apt-get update
apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rclcpp \
    ros-jazzy-test-msgs \
    ros-jazzy-mimick-vendor \
    ros-jazzy-osrf-testing-tools-cpp \
    ros-jazzy-ament-cmake-google-benchmark \
    ros-jazzy-performance-test-fixture \
    git

# Create workspace
echo "=== Creating workspace ==="
mkdir -p /workspace/ws/src
cd /workspace/ws

# Clone official rclcpp for tests (unpatched)
echo "=== Cloning official rclcpp ==="
cd /workspace/ws/src
if [ ! -d "rclcpp" ]; then
    git clone --depth 1 --branch jazzy https://github.com/ros2/rclcpp.git
fi

# Build rclcpp tests
echo "=== Building rclcpp ==="
cd /workspace/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rclcpp --cmake-args -DBUILD_TESTING=ON

# Source the built workspace
source /workspace/ws/install/setup.bash

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

echo ""
echo "=== Starting rmw_zenohd daemon ==="

# Kill any existing zenohd processes
pkill -9 zenohd 2>/dev/null || true
pkill -9 rmw_zenohd 2>/dev/null || true
sleep 1

# Start the Zenoh daemon in background
ros2 run rmw_zenoh_cpp rmw_zenohd &
ZENOHD_PID=$!
echo "Started rmw_zenohd with PID: $ZENOHD_PID"

# Wait for daemon to initialize
sleep 3

# Verify daemon is running
if ! ps -p $ZENOHD_PID > /dev/null 2>&1; then
    echo "ERROR: rmw_zenohd failed to start!"
    exit 1
fi
echo "rmw_zenohd is running"

# Cleanup function
cleanup() {
    echo ""
    echo "=== Cleaning up ==="
    if [ -n "$ZENOHD_PID" ]; then
        kill $ZENOHD_PID 2>/dev/null || true
    fi
    pkill -9 zenohd 2>/dev/null || true
    pkill -9 rmw_zenohd 2>/dev/null || true
}
trap cleanup EXIT

echo ""
echo "=== Running Executor Tests with rmw_zenoh_cpp ==="
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
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
    if timeout 120 ctest -R "^${test}$" -V 2>&1 | tee "$RESULTS_DIR/${test}.log"; then
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
