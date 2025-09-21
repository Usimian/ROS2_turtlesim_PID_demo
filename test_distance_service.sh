#!/bin/bash

# Test script for the distance service implementation
# This script demonstrates various ways to test the distance service

echo "🐢 Testing Distance Service Implementation"
echo "========================================="

# Source the workspace
source install/setup.bash

echo ""
echo "1. Testing service availability..."
if ros2 service list | grep -q "/move_distance"; then
    echo "✅ Distance service is available"
else
    echo "❌ Distance service not found. Make sure to run:"
    echo "   ros2 launch turtle_demo_controller turtle_with_distance_control.launch.py"
    exit 1
fi

echo ""
echo "2. Testing small forward movement (0.5m)..."
ros2 service call /move_distance cpp_node/srv/MoveDistance "{distance: 0.5, max_speed: 0.3, tolerance: 0.02}" &
wait

echo ""
echo "3. Testing backward movement (-0.3m)..."
ros2 service call /move_distance cpp_node/srv/MoveDistance "{distance: -0.3, max_speed: 0.3, tolerance: 0.02}" &
wait

echo ""
echo "4. Testing larger movement (1.0m)..."
ros2 service call /move_distance cpp_node/srv/MoveDistance "{distance: 1.0, max_speed: 0.5, tolerance: 0.05}" &
wait

echo ""
echo "5. Testing position movement via client..."
ros2 run turtle_demo_controller distance_client 8.0 3.0 &
wait

echo ""
echo "6. Testing another position movement..."
ros2 run turtle_demo_controller distance_client 2.0 8.0 &
wait

echo ""
echo "🎉 Distance service and position testing completed!"
echo ""
echo "For interactive testing (both distance and position modes), run:"
echo "   ros2 run turtle_demo_controller distance_client"
echo ""
echo "For command-line testing:"
echo "   Distance mode: ros2 run turtle_demo_controller distance_client <distance> [max_speed] [tolerance]"
echo "   Position mode: ros2 run turtle_demo_controller distance_client <X> <Y>"
