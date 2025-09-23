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
from cpp_node.action import GoToPose, RotateToPose


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
        
        # Action servers
        self._movement_server = ActionServer(
            self, GoToPose, '/go_to_pose',
            execute_callback=self.execute_movement_callback,
            callback_group=self.callback_group,
            goal_callback=self.movement_goal_callback,
            cancel_callback=self.movement_cancel_callback)
        
        self._rotation_server = ActionServer(
            self, RotateToPose, '/rotate_to_pose',
            execute_callback=self.execute_rotation_callback,
            callback_group=self.callback_group,
            goal_callback=self.rotation_goal_callback,
            cancel_callback=self.rotation_cancel_callback)
        
        # State variables
        self.current_pose = None
        self.goal_handle = None
        self.executing = False

    def movement_goal_callback(self, goal_request):
        """Accept or reject incoming movement goals."""
        self.get_logger().info(f"Received movement goal: ({goal_request.desired_x_pos:.2f}, {goal_request.desired_y_pos:.2f})")
        return GoalResponse.ACCEPT

    def movement_cancel_callback(self, goal_handle):
        """Handle movement goal cancellation."""
        self.get_logger().info("Movement goal canceled")
        self.executing = False
        self.stop_turtle()
        return CancelResponse.ACCEPT

    def rotation_goal_callback(self, goal_request):
        """Accept or reject incoming rotation goals."""
        if goal_request.relative:
            self.get_logger().info(f"Received relative rotation goal: {math.degrees(goal_request.desired_angle):.1f}°")
        else:
            self.get_logger().info(f"Received absolute rotation goal: {math.degrees(goal_request.desired_angle):.1f}°")
        return GoalResponse.ACCEPT

    def rotation_cancel_callback(self, goal_handle):
        """Handle rotation goal cancellation."""
        self.get_logger().info("Rotation goal canceled")
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

    def execute_movement_callback(self, goal_handle):
        """Execute movement goal using simple PID control."""
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

    def execute_rotation_callback(self, goal_handle):
        """Execute rotation goal using PID control."""
        self.goal_handle = goal_handle
        self.executing = True
        
        start_time = time.time()
        
        if self.current_pose is None:
            self.get_logger().error("No pose data available")
            result = RotateToPose.Result()
            result.success = False
            result.message = "No pose data available"
            result.execution_time = time.time() - start_time
            goal_handle.abort()
            self.executing = False
            return result
        
        # Calculate target angle
        start_angle = self.current_pose.theta
        if goal_handle.request.relative:
            target_angle = start_angle + goal_handle.request.desired_angle
        else:
            target_angle = goal_handle.request.desired_angle
        
        # Normalize target angle to [-pi, pi]
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        while target_angle < -math.pi:
            target_angle += 2 * math.pi
        
        max_angular_speed = goal_handle.request.max_angular_speed if goal_handle.request.max_angular_speed > 0 else 2.0
        
        self.get_logger().info(f"Starting rotation: {math.degrees(start_angle):.1f}° → {math.degrees(target_angle):.1f}°")
        
        # PID constants for rotation (similar to movement)
        kp = 3.0  # Proportional gain
        ki = 0.1  # Integral gain
        kd = 0.5  # Derivative gain
        
        # PID state variables
        integral_error = 0.0
        previous_error = 0.0
        
        # Control parameters
        tolerance = 0.05  # 0.05 radians ≈ 2.9 degrees
        rate = self.create_rate(50)  # 50 Hz for smooth control
        dt = 1.0 / 50.0
        max_time = 30.0  # 30 second timeout
        
        try:
            while self.executing and rclpy.ok() and (time.time() - start_time) < max_time:
                if self.current_pose is None:
                    continue
                
                # Calculate angular error (shortest path)
                current_angle = self.current_pose.theta
                angular_error = target_angle - current_angle
                
                # Normalize error to [-pi, pi] (shortest rotation path)
                while angular_error > math.pi:
                    angular_error -= 2 * math.pi
                while angular_error < -math.pi:
                    angular_error += 2 * math.pi
                
                # Check if goal is reached
                if abs(angular_error) <= tolerance:
                    self.get_logger().info(f"Rotation completed! Final error: {math.degrees(abs(angular_error)):.2f}°")
                    break
                
                # PID calculations
                # Proportional term
                p_term = kp * angular_error
                
                # Integral term (with windup protection)
                integral_error += angular_error * dt
                integral_error = max(-1.0, min(1.0, integral_error))  # Clamp integral
                i_term = ki * integral_error
                
                # Derivative term
                d_term = kd * (angular_error - previous_error) / dt
                
                # PID output
                pid_output = p_term + i_term + d_term
                
                # Apply velocity limits
                angular_velocity = max(-max_angular_speed, min(max_angular_speed, pid_output))
                
                # Publish feedback
                feedback = RotateToPose.Feedback()
                feedback.current_angle = current_angle
                feedback.remaining_angle = angular_error
                feedback.angular_velocity = angular_velocity
                goal_handle.publish_feedback(feedback)
                
                # Apply velocity command
                twist = Twist()
                twist.angular.z = angular_velocity
                self.cmd_vel_pub.publish(twist)
                
                # Update for next iteration
                previous_error = angular_error
                rate.sleep()
            
            # Stop rotation
            self.stop_turtle()
            
            # Create result
            final_angle = self.current_pose.theta if self.current_pose else target_angle
            angular_distance = final_angle - start_angle
            
            # Normalize angular distance
            while angular_distance > math.pi:
                angular_distance -= 2 * math.pi
            while angular_distance < -math.pi:
                angular_distance += 2 * math.pi
            
            execution_time = time.time() - start_time
            
            result = RotateToPose.Result()
            result.success = True
            result.final_angle = final_angle
            result.angular_distance = angular_distance
            result.execution_time = execution_time
            result.message = f"Rotation completed: {math.degrees(angular_distance):.1f}° in {execution_time:.2f}s"
            
            goal_handle.succeed()
            self.executing = False
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error during rotation execution: {e}")
            self.stop_turtle()
            goal_handle.abort()
            self.executing = False
            
            result = RotateToPose.Result()
            result.success = False
            result.message = f"Rotation failed: {str(e)}"
            result.execution_time = time.time() - start_time
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