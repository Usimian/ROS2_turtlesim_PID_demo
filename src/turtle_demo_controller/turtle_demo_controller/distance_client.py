#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from cpp_node.srv import MoveDistance
from cpp_node.action import GoToPose
from rclpy.action import ActionClient
from turtlesim.msg import Pose
import sys


class DistanceClient(Node):
    def __init__(self):
        super().__init__('distance_client')
        
        # Service client for distance movement
        self.distance_client = self.create_client(MoveDistance, '/move_distance')
        
        # Action client for position movement  
        self.action_client = ActionClient(self, GoToPose, 'GoToPose')
        
        # Current pose storage
        self.current_pose = None
        self.pose_received = False
        
        # Pose subscriber to get current turtle position
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        
        # Wait for services to be available
        while not self.distance_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Distance service not available, waiting...')
            
        # Wait for pose data with timeout
        self.get_logger().info('Waiting for turtle pose data...')
        start_time = time.time()
        timeout = 10.0  # 10 second timeout
        
        while not self.pose_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.pose_received:
            self.get_logger().error(f'Failed to receive pose data after {timeout}s timeout!')
            self.get_logger().error('Make sure turtlesim is running and publishing on /turtle1/pose')
        else:
            self.get_logger().info(f'Pose data received: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})')
    
    def pose_callback(self, msg: Pose):
        """Store current turtle pose"""
        self.current_pose = msg
        self.pose_received = True
    
    def send_distance_request(self, distance, max_speed=0.5, tolerance=0.12):
        """Send a distance movement request"""
        # Check if we have pose data
        if not self.pose_received or not self.current_pose:
            self.get_logger().error('No pose data available! Cannot send distance request.')
            return None
            
        request = MoveDistance.Request()
        request.distance = distance
        request.max_speed = max_speed
        request.tolerance = tolerance
        
        self.get_logger().info(f'Requesting distance movement: {distance}m at max {max_speed}m/s')
        
        future = self.distance_client.call_async(request)
        
        # Wait for service response with timeout
        start_wait = time.time()
        while not future.done() and (time.time() - start_wait) < 60.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done() and future.result() is not None:
            response = future.result()
            self.get_logger().info('Distance service call completed!')
            self.get_logger().info(f'Success: {response.success}')
            self.get_logger().info(f'Actual distance: {response.actual_distance:.3f}m')
            self.get_logger().info(f'Final error: {response.final_error:.3f}m')
            self.get_logger().info(f'Execution time: {response.execution_time:.1f}s')
            self.get_logger().info(f'Status: {response.status}')
            return response
        elif not future.done():
            self.get_logger().error('Distance service call timeout!')
            return None
        else:
            self.get_logger().error('Distance service call failed!')
            return None
    
    def send_position_request(self, x, y):
        """Send a position movement request using the action client"""
        # Check if we have pose data
        if not self.pose_received or not self.current_pose:
            self.get_logger().error('No pose data available! Cannot send position request.')
            return None
            
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('GoToPose action server not available!')
            return None
        
        # Calculate distance and direction for logging
        if self.current_pose:
            dx = x - self.current_pose.x
            dy = y - self.current_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            self.get_logger().info(f'Requesting position movement: ({x:.2f}, {y:.2f}) - distance: {distance:.2f}m')
        else:
            self.get_logger().info(f'Requesting position movement: ({x:.2f}, {y:.2f})')
        
        # Create and send goal
        goal_msg = GoToPose.Goal()
        goal_msg.desired_x_pos = x
        goal_msg.desired_y_pos = y
        
        future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance with timeout
        start_wait = time.time()
        while not future.done() and (time.time() - start_wait) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Goal submission timeout!')
            return None
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return None
            
        self.get_logger().info('Goal accepted! Waiting for completion...')
        
        # Wait for result with timeout
        result_future = goal_handle.get_result_async()
        start_wait = time.time()
        while not result_future.done() and (time.time() - start_wait) < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not result_future.done():
            self.get_logger().error('Goal execution timeout!')
            return None
            
        result = result_future.result().result
        self.get_logger().info('Position action completed!')
        self.get_logger().info(f'Success: {result.success}')
        
        # Calculate final distance for comparison
        if self.current_pose:
            final_dx = x - self.current_pose.x
            final_dy = y - self.current_pose.y
            final_distance = math.sqrt(final_dx*final_dx + final_dy*final_dy)
            self.get_logger().info(f'Final distance to target: {final_distance:.3f}m')
        
        return result


def get_user_input():
    """Get user input for movement parameters - supports both distance and position modes"""
    while True:
        try:
            print("\n" + "="*60)
            print("🎯 TURTLE MOVEMENT CONTROLLER TEST 🐢")
            print("="*60)
            print("Choose movement mode:")
            print("1. Distance mode - move forward/backward by distance")
            print("2. Position mode - move to specific X,Y coordinates")
            print("Or type 'quit' to exit")
            
            mode_input = input("Enter mode (1/2) or 'quit': ").strip()
            if mode_input.lower() in ['quit', 'exit', 'q']:
                return None
            
            if mode_input == '1':
                # Distance mode
                distance_input = input("Enter distance to move (meters, +forward/-backward): ").strip()
                if distance_input.lower() in ['quit', 'exit', 'q']:
                    return None
                
                distance = float(distance_input)
                
                speed_input = input("Enter max speed (m/s, default 0.5): ").strip()
                max_speed = float(speed_input) if speed_input else 0.5
                
                tolerance_input = input("Enter tolerance (m, default 0.05): ").strip()
                tolerance = float(tolerance_input) if tolerance_input else 0.05
                
                return ('distance', distance, max_speed, tolerance)
                
            elif mode_input == '2':
                # Position mode
                x_input = input("Enter target X coordinate (0-11): ").strip()
                if x_input.lower() in ['quit', 'exit', 'q']:
                    return None
                    
                y_input = input("Enter target Y coordinate (0-11): ").strip()
                if y_input.lower() in ['quit', 'exit', 'q']:
                    return None
                
                x = float(x_input)
                y = float(y_input)
                
                # Validate coordinates are within bounds
                if x < 0 or x > 11 or y < 0 or y > 11:
                    print("⚠️  Warning: Coordinates outside turtlesim bounds (0-11)")
                    confirm = input("Continue anyway? (y/n): ").strip().lower()
                    if confirm not in ['y', 'yes']:
                        continue
                
                return ('position', x, y)
                
            else:
                print("❌ Invalid mode! Please enter 1 or 2.")
            
        except ValueError:
            print("❌ Invalid input! Please enter numeric values.")
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            return None


def main(args=None):
    rclpy.init(args=args)
    
    # Check if arguments were provided
    if len(sys.argv) >= 2:
        # Command line mode
        try:
            # Check for explicit mode flags
            if sys.argv[1] == '--pos' and len(sys.argv) >= 4:
                # Explicit position mode
                x = float(sys.argv[2])
                y = float(sys.argv[3])
                print(f"Explicit position mode: ({x:.2f}, {y:.2f})")
                
                client = DistanceClient()
                result = client.send_position_request(x, y)
                
                if result and result.success:
                    print(f"✅ Position movement successful! Reached ({x:.2f}, {y:.2f})")
                else:
                    print(f"❌ Position movement failed or incomplete")
                    
            elif sys.argv[1] == '--dist' and len(sys.argv) >= 3:
                # Explicit distance mode
                distance = float(sys.argv[2])
                max_speed = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.5
                tolerance = float(sys.argv[4]) if len(sys.argv) >= 5 else 0.12
                print(f"Explicit distance mode: {distance:.2f}m at max {max_speed:.2f}m/s (tolerance: {tolerance:.3f}m)")
                
                client = DistanceClient()
                response = client.send_distance_request(distance, max_speed, tolerance)
                
                if response and response.success:
                    print(f"✅ Distance movement successful! Moved {response.actual_distance:.3f}m in {response.execution_time:.1f}s")
                else:
                    print(f"❌ Distance movement failed or incomplete")
                    
            # Check if this is position mode (exactly 3 args and both are reasonable coordinates)
            elif len(sys.argv) == 3:
                arg1 = float(sys.argv[1])
                arg2 = float(sys.argv[2])
                
                # Improved heuristic: 
                # - If both values are reasonable coordinates (2-10) AND second value > 1.5, likely position
                # - If second value <= 1.5, likely distance with max_speed
                # - If first value is negative, definitely distance mode
                if arg1 < 0 or arg2 <= 1.5:
                    # Distance mode: distance_client DISTANCE max_speed
                    distance = arg1
                    max_speed = arg2
                    tolerance = 0.12  # default - match actual PID controller performance
                    print(f"Interpreting as distance mode: {distance:.2f}m at {max_speed:.2f}m/s")
                    
                    client = DistanceClient()
                    response = client.send_distance_request(distance, max_speed, tolerance)
                    
                    if response and response.success:
                        print(f"✅ Distance movement successful! Moved {response.actual_distance:.3f}m in {response.execution_time:.1f}s")
                    else:
                        print(f"❌ Distance movement failed or incomplete")
                        
                elif 2 <= arg1 <= 10 and 2 <= arg2 <= 10:
                    # Position mode: distance_client X Y
                    x, y = arg1, arg2
                    print(f"Interpreting as position mode: ({x:.2f}, {y:.2f})")
                    
                    client = DistanceClient()
                    result = client.send_position_request(x, y)
                    
                    if result and result.success:
                        print(f"✅ Position movement successful! Reached ({x:.2f}, {y:.2f})")
                    else:
                        print(f"❌ Position movement failed or incomplete")
                else:
                    # Ambiguous case - default to distance mode
                    distance = arg1
                    max_speed = arg2
                    tolerance = 0.12  # default - match actual PID controller performance
                    print(f"Ambiguous case - interpreting as distance mode: {distance:.2f}m at {max_speed:.2f}m/s")
                    print("Use --dist or --pos flags for explicit mode selection")
                    
                    client = DistanceClient()
                    response = client.send_distance_request(distance, max_speed, tolerance)
                    
                    if response and response.success:
                        print(f"✅ Distance movement successful! Moved {response.actual_distance:.3f}m in {response.execution_time:.1f}s")
                    else:
                        print(f"❌ Distance movement failed or incomplete")
                        
            else:
                # Distance mode: distance_client DISTANCE [max_speed] [tolerance]
                distance = float(sys.argv[1])
                max_speed = float(sys.argv[2]) if len(sys.argv) >= 3 else 0.5
                tolerance = float(sys.argv[3]) if len(sys.argv) >= 4 else 0.12
                print(f"Distance mode: {distance:.2f}m at max {max_speed:.2f}m/s (tolerance: {tolerance:.3f}m)")
                
                client = DistanceClient()
                response = client.send_distance_request(distance, max_speed, tolerance)
                
                if response and response.success:
                    print(f"✅ Distance movement successful! Moved {response.actual_distance:.3f}m in {response.execution_time:.1f}s")
                else:
                    print(f"❌ Distance movement failed or incomplete")
                
        except ValueError:
            print("Usage:")
            print("  Position mode: ros2 run turtle_demo_controller distance_client <X> <Y>")
            print("  Distance mode: ros2 run turtle_demo_controller distance_client <distance> [max_speed] [tolerance]")
            print("  Explicit modes: ros2 run turtle_demo_controller distance_client --pos <X> <Y>")
            print("                  ros2 run turtle_demo_controller distance_client --dist <distance> [max_speed] [tolerance]")
            print("")
            print("Examples:")
            print("  ros2 run turtle_demo_controller distance_client 8.0 3.0        # Move to position (8,3)")
            print("  ros2 run turtle_demo_controller distance_client 2.0            # Move forward 2m")
            print("  ros2 run turtle_demo_controller distance_client 2.0 0.8 0.02   # Move 2m at 0.8m/s with 0.02m tolerance")
            print("  ros2 run turtle_demo_controller distance_client --dist 2.0 0.8 # Explicitly distance mode")
            print("  ros2 run turtle_demo_controller distance_client --pos 8.0 3.0  # Explicitly position mode")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if 'client' in locals():
                try:
                    client.destroy_node()
                except:
                    pass
            try:
                rclpy.shutdown()
            except:
                pass
    else:
        # Interactive mode
        client = DistanceClient()
        
        try:
            print("🚀 Turtle Movement Test Client Started")
            print("Make sure turtlesim and turtle controllers are running!")
            print(f"Current turtle position: ({client.current_pose.x:.2f}, {client.current_pose.y:.2f})")
            
            while rclpy.ok():
                user_input = get_user_input()
                
                if user_input is None:
                    break
                
                if user_input[0] == 'distance':
                    # Distance mode
                    _, distance, max_speed, tolerance = user_input
                    response = client.send_distance_request(distance, max_speed, tolerance)
                    
                    if response:
                        if response.success:
                            print(f"✅ Distance movement completed successfully!")
                            print(f"   Actual: {response.actual_distance:.3f}m, Error: {response.final_error:.3f}m")
                            print(f"   Time: {response.execution_time:.1f}s")
                        else:
                            print(f"❌ Distance movement failed or incomplete: {response.status}")
                    else:
                        print("❌ Distance service call failed")
                        
                elif user_input[0] == 'position':
                    # Position mode
                    _, x, y = user_input
                    result = client.send_position_request(x, y)
                    
                    if result:
                        if result.success:
                            print(f"✅ Position movement completed successfully!")
                            print(f"   Target: ({x:.2f}, {y:.2f})")
                        else:
                            print(f"❌ Position movement failed!")
                    else:
                        print("❌ Position action call failed")
                
                # Show current position after movement
                if client.current_pose:
                    print(f"Current position: ({client.current_pose.x:.2f}, {client.current_pose.y:.2f})")
                print("\nReady for next command...")
                
        except KeyboardInterrupt:
            print("\n👋 Shutting down gracefully...")
        finally:
            if 'client' in locals():
                try:
                    client.destroy_node()
                except:
                    pass
            try:
                rclpy.shutdown()
            except:
                pass


if __name__ == '__main__':
    main()
