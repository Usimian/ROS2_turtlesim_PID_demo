#!/bin/bash

# 🧪 Test Script for Working ROS2 Turtlesim PID Controller System
# This script tests the key functionality that was previously failing

echo "🐢 Testing ROS2 Turtlesim PID Controller - Working Implementation"
echo "=================================================================="

# Check if the system is running
echo "🔍 Checking if turtlesim system is running..."
if ! ros2 node list | grep -q "turtle_pid_controller"; then
    echo "❌ turtle_pid_controller not found. Please run:"
    echo "   ros2 launch turtle_demo_controller turtle_with_distance_control.launch.py"
    exit 1
fi

# Note: distance_controller is now a client, not a service node
echo "ℹ️  Distance functionality provided by unified distance_controller client"

echo "✅ System is running!"
echo ""

# Test 1: Basic distance movement
echo "🧪 Test 1: Basic distance movement (1.0m) - NEW SIMPLIFIED APPROACH"
echo "--------------------------------------------------------------------"
ros2 run turtle_demo_controller distance_controller 1.0
if [ $? -eq 0 ]; then
    echo "✅ Test 1 PASSED"
else
    echo "❌ Test 1 FAILED"
    exit 1
fi
echo ""

# Test 2: Second consecutive call (the critical test that was failing)
echo "🧪 Test 2: Second consecutive call (2.0m) - CRITICAL TEST"
echo "--------------------------------------------------------"
ros2 run turtle_demo_controller distance_controller 2.0
if [ $? -eq 0 ]; then
    echo "✅ Test 2 PASSED - No more hanging on second call!"
else
    echo "❌ Test 2 FAILED - Still hanging on second call"
    exit 1
fi
echo ""

# Test 3: Third consecutive call
echo "🧪 Test 3: Third consecutive call (-1.5m)"
echo "----------------------------------------"
ros2 run turtle_demo_controller distance_controller -1.5
if [ $? -eq 0 ]; then
    echo "✅ Test 3 PASSED"
else
    echo "❌ Test 3 FAILED"
    exit 1
fi
echo ""

# Test 4: Position movement
echo "🧪 Test 4: Position movement (8.0, 8.0)"
echo "---------------------------------------"
ros2 run turtle_demo_controller distance_controller 8.0 8.0
if [ $? -eq 0 ]; then
    echo "✅ Test 4 PASSED"
else
    echo "❌ Test 4 FAILED"
    exit 1
fi
echo ""

# Test 5: Mixed mode (distance after position)
echo "🧪 Test 5: Mixed mode - distance after position (1.0m)"
echo "------------------------------------------------------"
ros2 run turtle_demo_controller distance_controller 1.0
if [ $? -eq 0 ]; then
    echo "✅ Test 5 PASSED"
else
    echo "❌ Test 5 FAILED"
    exit 1
fi
echo ""

echo "🎉 ALL TESTS PASSED!"
echo "===================="
echo "✅ No hanging issues on consecutive calls"
echo "✅ Distance movement working"
echo "✅ Position movement working" 
echo "✅ Mixed mode operations working"
echo ""
echo "🏆 The ROS2 Turtlesim PID Controller system is fully functional!"
