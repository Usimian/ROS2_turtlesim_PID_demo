#!/bin/bash

echo "🐢 Quick Distance Service Test"
echo "=============================="

# Check if launch file is running
if ! ros2 service list | grep -q "/move_distance"; then
    echo "❌ Distance service not found!"
    echo "Please start: ros2 launch turtle_demo_controller turtle_with_distance_control.launch.py"
    exit 1
fi

echo "✅ Distance service found!"
echo ""

# Source the workspace
source install/setup.bash

echo "🧪 Running quick tests..."

echo "Test 1: Small forward movement (0.3m)"
ros2 run turtle_demo_controller distance_client --dist 0.3 0.4 0.08
sleep 2

echo ""
echo "Test 2: Backward movement (-0.2m)"
ros2 run turtle_demo_controller distance_client --dist -0.2 0.4 0.08
sleep 2

echo ""
echo "Test 3: Position movement (8, 4)"
ros2 run turtle_demo_controller distance_client --pos 8.0 4.0
sleep 2

echo ""
echo "Test 4: Larger distance (1.0m)"
ros2 run turtle_demo_controller distance_client --dist 1.0 0.5 0.08

echo ""
echo "🎉 Quick test sequence completed!"
