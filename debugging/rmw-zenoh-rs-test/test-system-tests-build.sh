#!/bin/bash

echo "=== Testing system_tests build ==="
echo "Date: $(date)"
echo ""

source /opt/ros/jazzy/setup.bash
unset RMW_IMPLEMENTATION

# Install dependencies
apt-get update > /dev/null 2>&1
apt-get install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-test-msgs > /dev/null 2>&1

# Setup workspace
rm -rf /workspace/ws
mkdir -p /workspace/ws/src
cd /workspace/ws/src

# Copy ros-z sources
cp -r /workspace/ros-z /workspace/ws/src/ros-z

# Add COLCON_IGNORE to non-essential ros-z directories
for dir in _tmp book .github scripts ros-z-tests ros-z-console ros-z-py assets nix .config .claude; do
    if [ -d "/workspace/ws/src/ros-z/$dir" ]; then
        touch "/workspace/ws/src/ros-z/$dir/COLCON_IGNORE"
    fi
done

# Copy system_tests
cp -r /workspace/system_tests /workspace/ws/src/system_tests

# Build rmw_zenoh_rs
echo "=== Building rmw_zenoh_rs ==="
cd /workspace/ws
colcon build --packages-select rmw_zenoh_rs --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10

source /workspace/ws/install/setup.bash

# Check exported symbols
echo ""
echo "=== Checking rmw_serialize/rmw_deserialize symbols ==="
nm -D /workspace/ws/install/rmw_zenoh_rs/lib/librmw_zenoh_rs.so | grep -E "rmw_serialize|rmw_deserialize" || echo "Symbols not found!"

# Build test_communication
echo ""
echo "=== Building test_communication ==="
colcon build --packages-select test_communication --cmake-args -DBUILD_TESTING=ON 2>&1

echo ""
echo "=== Done ==="
