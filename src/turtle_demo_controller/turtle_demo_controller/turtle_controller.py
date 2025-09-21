#!/usr/bin/env python3

"""
Simple, robust ROS2 action server for turtle control following best practices.
Replaces the previous problematic implementation with a clean, working solution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from cpp_node.action import GoToPose


class Controller_Node(Node):
    def __init__(self):
        super().__init__('turtle_pid_controller')
        self.get_logger().info("Simple Turtle Controller initialized")
        
        # Create callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10, 
            callback_group=self.callback_group)
        
        # Action server
        self._action_server = ActionServer(
            self, GoToPose, 'go_to_pose',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        # State variables
        self.current_pose = None
        self.goal_handle = None
        self.executing = False

    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        self.get_logger().info(f"Received goal: ({goal_request.desired_x_pos:.2f}, {goal_request.desired_y_pos:.2f})")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation."""
        self.get_logger().info("Goal canceled")
        self.executing = False
        self.stop_turtle()
        return CancelResponse.ACCEPT

    def pose_callback(self, msg):
        """Update current pose."""
        self.current_pose = msg

    def stop_turtle(self):
        """Stop the turtle by sending zero velocities."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def execute_callback(self, goal_handle):
        """Execute the goal using simple PID control."""
        self.goal_handle = goal_handle
        self.executing = True
        
        target_x = goal_handle.request.desired_x_pos
        target_y = goal_handle.request.desired_y_pos
        
        self.get_logger().info("Executing goal...")
        
        # PID constants
        kp_linear = 1.5
        kp_angular = 6.0
        
        # Control loop
        rate = self.create_rate(10)  # 10 Hz
        
        try:
            while self.executing and rclpy.ok():
                if self.current_pose is None:
                    continue
                
                # Calculate errors
                dx = target_x - self.current_pose.x
                dy = target_y - self.current_pose.y
                distance = math.sqrt(dx**2 + dy**2)
                
                target_angle = math.atan2(dy, dx)
                angle_error = target_angle - self.current_pose.theta
                
                # Normalize angle error to [-pi, pi]
                while angle_error > math.pi:
                    angle_error -= 2 * math.pi
                while angle_error < -math.pi:
                    angle_error += 2 * math.pi
                
                # Check if goal is reached
                if distance < 0.1 and abs(angle_error) < 0.1:
                    self.get_logger().info("Goal reached!")
                    self.stop_turtle()
                    
                    # Create result
                    result = GoToPose.Result()
                    result.success = True
                    goal_handle.succeed()
                    self.executing = False
                    return result
                
                # Publish feedback
                feedback = GoToPose.Feedback()
                feedback.current_x_pos = self.current_pose.x
                feedback.current_y_pos = self.current_pose.y
                goal_handle.publish_feedback(feedback)
                
                # Calculate control commands
                twist = Twist()
                
                # Linear velocity (proportional to distance)
                twist.linear.x = kp_linear * distance
                
                # Angular velocity (proportional to angle error)
                twist.angular.z = kp_angular * angle_error
                
                # Apply velocity limits
                twist.linear.x = max(-2.0, min(2.0, twist.linear.x))
                twist.angular.z = max(-2.0, min(2.0, twist.angular.z))
                
                # Publish velocity command
                self.cmd_vel_pub.publish(twist)
                
                # Sleep to maintain loop rate
                rate.sleep()
                
        except Exception as e:
            self.get_logger().error(f"Error during goal execution: {e}")
            self.stop_turtle()
            goal_handle.abort()
            self.executing = False
            result = GoToPose.Result()
            result.success = False
            return result
        finally:
            self.stop_turtle()
            self.executing = False
            self.goal_handle = None


def main(args=None):
    """Main function to run the turtle controller node."""
    rclpy.init(args=args)
    
    # Create the node with MultiThreadedExecutor for proper concurrency
    node = Controller_Node()
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()