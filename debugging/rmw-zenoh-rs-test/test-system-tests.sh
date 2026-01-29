#!/bin/bash

echo "=== ROS 2 System Tests (Demo Nodes) ==="
echo "Date: $(date)"
echo ""

source /opt/ros/jazzy/setup.bash

# Clear any existing RMW_IMPLEMENTATION
unset RMW_IMPLEMENTATION

# Install dependencies
echo "=== Installing dependencies ==="
apt-get update > /dev/null 2>&1
apt-get install -y ros-jazzy-rmw-zenoh-cpp ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py > /dev/null 2>&1

# Setup workspace
echo "=== Setting up workspace ==="
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

# Build rmw_zenoh_rs
echo "=== Building rmw_zenoh_rs ==="
cd /workspace/ws
colcon build --packages-select rmw_zenoh_rs --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5

echo "=== Sourcing workspace ==="
source /workspace/ws/install/setup.bash || { echo "Failed to source workspace"; exit 1; }
echo "Workspace sourced successfully"

# Start Zenoh router for discovery
echo "=== Starting Zenoh router ==="
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
ROUTER_PID=$!
sleep 5
echo "Router started with PID: $ROUTER_PID"

# Check if router is running
if ps -p $ROUTER_PID > /dev/null 2>&1; then
    echo "Router is running"
    echo "Router log:"
    cat /tmp/zenoh_router.log | head -5
else
    echo "Router failed to start or exited"
    echo "Router log:"
    cat /tmp/zenoh_router.log
fi

cleanup() {
    echo "Cleaning up..."
    kill $ROUTER_PID 2>/dev/null || true
    pkill -f rmw_zenohd 2>/dev/null || true
}
trap cleanup EXIT

run_pubsub_test() {
    local rmw=$1
    local timeout_sec=20

    echo ""
    echo "### Pub/Sub Test ($rmw) ###"
    export RMW_IMPLEMENTATION=$rmw

    # Start publisher in background
    timeout $timeout_sec ros2 run demo_nodes_cpp talker 2>&1 &
    PUB_PID=$!
    sleep 5  # Wait for discovery

    # Start subscriber, capture output
    OUTPUT=$(timeout 10 ros2 run demo_nodes_cpp listener 2>&1 || true)

    # Kill publisher
    kill $PUB_PID 2>/dev/null || true
    wait $PUB_PID 2>/dev/null || true

    # Check if we received messages
    if echo "$OUTPUT" | grep -q "I heard"; then
        echo "PASSED: Received messages"
        echo "$OUTPUT" | grep "I heard" | head -3
    else
        echo "FAILED: No messages received"
        echo "Output:"
        echo "$OUTPUT" | head -15
    fi
}

run_service_test() {
    local rmw=$1
    local timeout_sec=20

    echo ""
    echo "### Service Test ($rmw) ###"
    export RMW_IMPLEMENTATION=$rmw

    # Start server in background
    timeout $timeout_sec ros2 run demo_nodes_cpp add_two_ints_server 2>&1 &
    SRV_PID=$!
    sleep 5  # Wait for discovery

    # Call service
    OUTPUT=$(timeout 10 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}" 2>&1 || true)

    # Kill server
    kill $SRV_PID 2>/dev/null || true
    wait $SRV_PID 2>/dev/null || true

    # Check result (handle both "sum: 3" and "sum=3" formats)
    if echo "$OUTPUT" | grep -qE "sum[=:].?3"; then
        echo "PASSED: Service returned correct result"
        echo "$OUTPUT" | grep -i "sum"
    else
        echo "FAILED: Service call failed"
        echo "Output:"
        echo "$OUTPUT" | head -15
    fi
}

echo ""
echo "=============================================="
echo "=== Testing with rmw_zenoh_cpp ==="
echo "=============================================="

run_pubsub_test rmw_zenoh_cpp
run_service_test rmw_zenoh_cpp

echo ""
echo "=============================================="
echo "=== Testing with rmw_zenoh_rs ==="
echo "=============================================="

run_pubsub_test rmw_zenoh_rs
run_service_test rmw_zenoh_rs

echo ""
echo "=== Zenoh Router Log ==="
cat /tmp/zenoh_router.log 2>/dev/null | tail -20 || echo "No router log"

echo ""
echo "=== Test Summary ==="
echo "Date: $(date)"
echo "=== Done ==="
