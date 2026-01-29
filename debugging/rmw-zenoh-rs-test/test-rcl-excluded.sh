#!/bin/bash
set -e

echo "=== Testing test_count_matched and test_events ==="
echo "Date: $(date)"
echo ""

source /opt/ros/jazzy/setup.bash

# Install rmw_zenoh_cpp if not present
apt-get update > /dev/null 2>&1
apt-get install -y ros-jazzy-rmw-zenoh-cpp > /dev/null 2>&1

# Create workspace
echo "=== Setting up workspace ==="
rm -rf /workspace/ws
mkdir -p /workspace/ws/src
cd /workspace/ws/src

# Copy ros-z sources
cp -r /workspace/ros-z /workspace/ws/src/ros-z

# Add COLCON_IGNORE to non-essential directories
for dir in _tmp book .github scripts ros-z-tests ros-z-console ros-z-py assets nix .config .claude; do
    if [ -d "/workspace/ws/src/ros-z/$dir" ]; then
        touch "/workspace/ws/src/ros-z/$dir/COLCON_IGNORE"
    fi
done

# Copy rcl fork for testing
cp -r /workspace/rcl /workspace/ws/src/rcl

# Build rmw_zenoh_rs
echo "=== Building rmw_zenoh_rs ==="
cd /workspace/ws
colcon build --packages-select rmw_zenoh_rs --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5

# Source workspace
source /workspace/ws/install/setup.bash

# Build rcl tests only
echo "=== Building rcl tests ==="
colcon build --packages-select rcl --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5

source /workspace/ws/install/setup.bash

cd /workspace/ws/build/rcl/test

echo ""
echo "=============================================="
echo "=== Testing with rmw_zenoh_cpp ==="
echo "=============================================="

echo ""
echo "### test_count_matched (rmw_zenoh_cpp) ###"
RMW_IMPLEMENTATION=rmw_zenoh_cpp timeout 60 ./test_count_matched 2>&1 || echo "Exit code: $?"

echo ""
echo "### test_events (rmw_zenoh_cpp) ###"
RMW_IMPLEMENTATION=rmw_zenoh_cpp timeout 60 ./test_events 2>&1 || echo "Exit code: $?"

echo ""
echo "=============================================="
echo "=== Testing with rmw_zenoh_rs ==="
echo "=============================================="

echo ""
echo "### test_count_matched (rmw_zenoh_rs) ###"
RMW_IMPLEMENTATION=rmw_zenoh_rs timeout 60 ./test_count_matched 2>&1 || echo "Exit code: $?"

echo ""
echo "### test_events (rmw_zenoh_rs) ###"
RMW_IMPLEMENTATION=rmw_zenoh_rs timeout 60 ./test_events 2>&1 || echo "Exit code: $?"

echo ""
echo "=== Done ==="
