#!/usr/bin/env python3

"""
Simple Distance Client for ROS2 Turtlesim PID Demo

This client provides distance movement functionality by calculating the endpoint
locally and using the proven position movement action directly.

Architecture:
    Distance Request -> Calculate Endpoint -> Position Action -> PID Controller -> Turtlesim

This approach eliminates the problematic service layer and uses the working
position mode directly for reliable consecutive calls.
"""

import rclpy
import math
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from cpp_node.action import GoToPose


class SimpleDistanceClient(Node):
    def __init__(self):
        super().__init__('simple_distance_client')
        
        # Current pose storage
        self.current_pose = None
        self.pose_received = False
        
        # Action client for position movement (this works perfectly!)
        self.action_client = ActionClient(self, GoToPose, 'go_to_pose')
        
        # Pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )
        
        # Wait for pose data
        self.get_logger().info("Waiting for turtle pose data...")
        while not self.pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Pose data received: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")
    
    def pose_callback(self, msg: Pose):
        """Store current turtle pose"""
        self.current_pose = msg
        self.pose_received = True
    
    def move_distance(self, distance, max_speed=0.5):
        """Move the turtle a specific distance in its current heading direction"""
        if not self.pose_received:
            self.get_logger().error('No pose data available!')
            return False
        
        # Calculate endpoint using current position and heading
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        start_theta = self.current_pose.theta
        
        target_x = start_x + distance * math.cos(start_theta)
        target_y = start_y + distance * math.sin(start_theta)
        
        # Boundary clamping (keep within turtlesim bounds)
        target_x = max(0.5, min(10.5, target_x))
        target_y = max(0.5, min(10.5, target_y))
        
        # Calculate actual distance after boundary clamping
        actual_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        if distance < 0:
            actual_distance = -actual_distance
        
        self.get_logger().info(f"Moving {distance:.2f}m from ({start_x:.2f}, {start_y:.2f}) to ({target_x:.2f}, {target_y:.2f})")
        if abs(actual_distance - distance) > 0.1:
            self.get_logger().warn(f"Distance adjusted due to boundaries: {distance:.2f}m -> {actual_distance:.2f}m")
        
        # Use the working position movement directly
        return self.move_to_position(target_x, target_y)
    
    def move_to_position(self, x, y):
        """Move to specific X,Y coordinates using the working action client"""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('GoToPose action server not available!')
            return False
        
        # Create and send goal
        goal_msg = GoToPose.Goal()
        goal_msg.desired_x_pos = x
        goal_msg.desired_y_pos = y
        
        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f})')
        future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('Goal submission timeout!')
            return False
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
            
        self.get_logger().info('Goal accepted! Waiting for completion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        if not result_future.done():
            self.get_logger().error('Goal execution timeout!')
            return False
        
        result = result_future.result()
        if result.result.success:
            self.get_logger().info('Movement completed successfully!')
            return True
        else:
            self.get_logger().error('Movement failed!')
            return False


def main(args=None):
    """Main function - supports both distance and position modes"""
    rclpy.init(args=args)
    
    try:
        if len(sys.argv) < 2:
            print("Usage:")
            print("  Distance: ros2 run turtle_demo_controller distance_controller <distance>")
            print("  Position: ros2 run turtle_demo_controller distance_controller <x> <y>")
            print("  Examples:")
            print("    ros2 run turtle_demo_controller distance_controller 2.0    # Move 2m forward")
            print("    ros2 run turtle_demo_controller distance_controller -1.5   # Move 1.5m backward")
            print("    ros2 run turtle_demo_controller distance_controller 3.0 7.0 # Move to (3,7)")
            return
        
        client = SimpleDistanceClient()
        
        if len(sys.argv) == 2:
            # Distance mode
            distance = float(sys.argv[1])
            print(f"Distance mode: {distance:.2f}m")
            success = client.move_distance(distance)
        else:
            # Position mode  
            x, y = float(sys.argv[1]), float(sys.argv[2])
            print(f"Position mode: ({x:.2f}, {y:.2f})")
            success = client.move_to_position(x, y)
        
        if success:
            print("✅ Movement successful!")
        else:
            print("❌ Movement failed!")
            
    except ValueError:
        print("Error: Invalid arguments. Please provide numeric values.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'client' in locals():
            client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()