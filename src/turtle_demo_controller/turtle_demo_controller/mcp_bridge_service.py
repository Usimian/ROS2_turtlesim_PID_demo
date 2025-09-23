#!/usr/bin/env python3

"""
MCP Bridge Service for ROS2 Turtlesim PID Demo

This service node provides calculation and validation services for ros-mcp-server.
It does NOT execute movements directly to avoid the service->action hanging issue.

Architecture:
    LLM -> ros-mcp-server -> rosbridge -> THIS SERVICE (calculation only)
                          -> rosbridge -> /go_to_pose ACTION (movement)

Services provided:
- /turtle_distance_command: Calculate endpoint for distance movement
- /turtle_position_command: Validate position coordinates  
- /turtle_status: Get current turtle position and status

Note: Actual movement is done by ros-mcp-server calling /go_to_pose action directly
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from cpp_node.srv import TurtleDistanceCommand, TurtlePositionCommand, TurtleStatus, TurtleRotateRelative, TurtleRotateAbsolute
from cpp_node.action import GoToPose, RotateToPose


class MCPBridgeService(Node):
    def __init__(self):
        super().__init__('mcp_bridge_service')
        self.get_logger().info("MCP Bridge Service initializing...")

        # Current pose storage
        self.current_pose = None
        self.pose_received = False

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Service servers
        self.distance_service = self.create_service(
            TurtleDistanceCommand,
            '/turtle_move_relative',
            self.handle_distance_command
        )

        self.position_service = self.create_service(
            TurtlePositionCommand,
            '/turtle_move_position',
            self.handle_position_command
        )

        self.status_service = self.create_service(
            TurtleStatus,
            '/turtle_status',
            self.handle_status_query
        )


        self.rotate_relative_service = self.create_service(
            TurtleRotateRelative,
            '/turtle_rotate_relative',
            self.handle_rotate_relative_command
        )

        self.rotate_absolute_service = self.create_service(
            TurtleRotateAbsolute,
            '/turtle_rotate_angle',
            self.handle_rotate_absolute_command
        )

        # Action clients
        self.movement_client = ActionClient(self, GoToPose, '/go_to_pose')
        self.rotation_client = ActionClient(self, RotateToPose, '/rotate_to_pose')
        
        # Command queue system to ensure sequential execution
        self.command_queue = []
        self.current_action = None
        self.queue_timer = self.create_timer(0.1, self.process_command_queue)

        # Wait for initial pose data
        self.get_logger().info("Waiting for turtle pose data...")
        start_time = time.time()
        timeout = 10.0
        
        while not self.pose_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.pose_received:
            self.get_logger().error(f'Failed to receive pose data after {timeout}s timeout!')
            self.get_logger().error('Make sure turtlesim is running and publishing on /turtle1/pose')
        else:
            self.get_logger().info(f'Initial pose: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}, {self.current_pose.theta:.2f})')

        self.get_logger().info("MCP Bridge Service ready!")
        self.get_logger().info("Available services:")
        self.get_logger().info("  - /turtle_move_relative (calculate + execute distance movement)")
        self.get_logger().info("  - /turtle_move_position (validate + execute position movement)")
        self.get_logger().info("  - /turtle_rotate_relative (execute relative rotation)")
        self.get_logger().info("  - /turtle_rotate_angle (execute absolute rotation to angle)")
        self.get_logger().info("  - /turtle_status (status query)")
        self.get_logger().info("Note: All movement services now execute precise PID-controlled movement!")

    def process_command_queue(self):
        """Process the command queue to ensure sequential execution."""
        if self.current_action is not None:
            # Check if current action is complete
            if 'result_future' in self.current_action and self.current_action['result_future'].done():
                # Action completed, log result
                try:
                    result = self.current_action['result_future'].result().result
                    if result.success:
                        self.get_logger().info(f"Action completed: {result.message}")
                    else:
                        self.get_logger().error(f"Action failed: {result.message}")
                except Exception as e:
                    self.get_logger().error(f"Error getting action result: {e}")
                
                self.current_action = None
            else:
                # Action still running, wait
                return
        
        # Start next action in queue
        if self.command_queue:
            command = self.command_queue.pop(0)
            self.current_action = command
            self.get_logger().info(f"Starting execution of {command['name']}")
            
            try:
                # Send the goal
                goal_future = command['client'].send_goal_async(command['goal'])
                
                def goal_response_callback(future):
                    try:
                        goal_handle = future.result()
                        if goal_handle.accepted:
                            self.get_logger().info(f"Goal accepted for {command['name']}")
                            result_future = goal_handle.get_result_async()
                            self.current_action['result_future'] = result_future
                        else:
                            self.get_logger().error(f"Goal rejected for {command['name']}")
                            self.current_action = None
                    except Exception as e:
                        self.get_logger().error(f"Error in goal response callback: {e}")
                        self.current_action = None
                
                goal_future.add_done_callback(goal_response_callback)
                
            except Exception as e:
                self.get_logger().error(f"Error sending goal for {command['name']}: {e}")
                self.current_action = None

    def queue_action(self, client, goal, action_name):
        """Add an action to the queue for sequential execution."""
        command = {
            'client': client,
            'goal': goal,
            'name': action_name
        }
        self.command_queue.append(command)
        self.get_logger().info(f"Queued {action_name} (queue size: {len(self.command_queue)})")
        return True


    def pose_callback(self, msg: Pose):
        """Store current turtle pose"""
        self.current_pose = msg
        self.pose_received = True

    def validate_coordinates(self, x, y):
        """Validate and clamp coordinates to turtlesim bounds"""
        x_clamped = max(0.5, min(10.5, x))
        y_clamped = max(0.5, min(10.5, y))
        return x_clamped, y_clamped

    def validate_distance(self, distance):
        """Validate distance parameter"""
        if abs(distance) > 20.0:
            return False, "Distance too large (max 20 meters)"
        return True, ""

    def validate_speed_tolerance(self, max_speed, tolerance):
        """Validate speed and tolerance parameters"""
        if max_speed <= 0 or max_speed > 5.0:
            return False, "Invalid max_speed (must be 0 < speed <= 5.0)"
        if tolerance <= 0 or tolerance > 1.0:
            return False, "Invalid tolerance (must be 0 < tolerance <= 1.0)"
        return True, ""

    def handle_distance_command(self, request, response):
        """Calculate endpoint and EXECUTE distance movement"""
        start_time = time.time()
        self.get_logger().info(f"Distance movement: {request.distance:.2f}m")

        # Validate parameters
        valid, error_msg = self.validate_distance(request.distance)
        if not valid:
            response.success = False
            response.message = error_msg
            response.actual_distance = 0.0
            response.execution_time = time.time() - start_time
            return response

        # Get current pose
        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available"
            response.actual_distance = 0.0
            response.execution_time = time.time() - start_time
            return response

        # Calculate endpoint using trigonometry
        start_x = self.current_pose.x
        start_y = self.current_pose.y
        start_theta = self.current_pose.theta

        target_x = start_x + request.distance * math.cos(start_theta)
        target_y = start_y + request.distance * math.sin(start_theta)

        # Apply boundary clamping
        target_x, target_y = self.validate_coordinates(target_x, target_y)

        self.get_logger().info(f'Moving to calculated endpoint: ({target_x:.2f}, {target_y:.2f})')

        # Check if action server is available (but don't wait long)
        if not self.movement_client.server_is_ready():
            # Try to wait briefly
            if not self.movement_client.wait_for_server(timeout_sec=1.0):
                response.success = False
                response.message = "PID action server not available"
                response.actual_distance = 0.0
                response.execution_time = time.time() - start_time
                return response

        # Create and send goal
        goal_msg = GoToPose.Goal()
        goal_msg.desired_x_pos = target_x
        goal_msg.desired_y_pos = target_y

        # Queue the action for sequential execution
        try:
            success = self.queue_action(self.movement_client, goal_msg, "Distance Movement")
            
            if success:
                response.success = True
                response.message = f"Distance movement queued: {request.distance:.2f}m. Will execute when ready."
                response.actual_distance = request.distance
                response.execution_time = time.time() - start_time
            else:
                response.success = False
                response.message = "Failed to queue distance movement"
                response.actual_distance = 0.0
                response.execution_time = time.time() - start_time
        except Exception as e:
            response.success = False
            response.message = f"Failed to queue goal: {str(e)}"
            response.actual_distance = 0.0
            response.execution_time = time.time() - start_time

        return response

    def handle_position_command(self, request, response):
        """Validate coordinates and EXECUTE position movement"""
        start_time = time.time()
        self.get_logger().info(f"Position movement: ({request.target_x:.2f}, {request.target_y:.2f})")

        # Validate and clamp coordinates
        target_x, target_y = self.validate_coordinates(request.target_x, request.target_y)

        if target_x != request.target_x or target_y != request.target_y:
            self.get_logger().warn(f"Coordinates clamped to bounds: ({target_x:.2f}, {target_y:.2f})")

        self.get_logger().info(f'Moving to position: ({target_x:.2f}, {target_y:.2f})')

        # Check if action server is available (but don't wait long)
        if not self.movement_client.server_is_ready():
            # Try to wait briefly
            if not self.movement_client.wait_for_server(timeout_sec=1.0):
                response.success = False
                response.message = "PID action server not available"
                response.final_x = 0.0
                response.final_y = 0.0
                response.execution_time = time.time() - start_time
                return response

        # Create and send goal
        goal_msg = GoToPose.Goal()
        goal_msg.desired_x_pos = target_x
        goal_msg.desired_y_pos = target_y

        # Queue the action for sequential execution
        try:
            success = self.queue_action(self.movement_client, goal_msg, "Position Movement")
            
            if success:
                response.success = True
                response.message = f"Position movement queued: ({target_x:.2f}, {target_y:.2f}). Will execute when ready."
                response.final_x = target_x
                response.final_y = target_y
                response.execution_time = time.time() - start_time
            else:
                response.success = False
                response.message = "Failed to queue position movement"
                response.final_x = self.current_pose.x
                response.final_y = self.current_pose.y
                response.execution_time = time.time() - start_time
        except Exception as e:
            response.success = False
            response.message = f"Failed to send goal: {str(e)}"
            response.final_x = 0.0
            response.final_y = 0.0
            response.execution_time = time.time() - start_time

        return response

    def handle_status_query(self, request, response):
        """Handle status query service request"""
        self.get_logger().info("Status query received")

        if self.current_pose is None:
            response.success = False
            response.current_x = 0.0
            response.current_y = 0.0
            response.current_theta = 0.0
            response.status_message = "No pose data available"
            return response

        response.success = True
        response.current_x = self.current_pose.x
        response.current_y = self.current_pose.y
        response.current_theta = self.current_pose.theta
        response.status_message = f"Position: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}), " \
                                 f"Heading: {self.current_pose.theta:.2f}rad"

        return response


    def handle_rotate_relative_command(self, request, response):
        """Execute relative rotation using GoToPose action (non-blocking)"""
        start_time = time.time()
        self.get_logger().info(f"Relative rotation command: {request.angle:.2f} radians ({math.degrees(request.angle):.1f} degrees)")

        # Get current pose
        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available"
            response.final_heading = 0.0
            response.actual_rotation = 0.0
            response.execution_time = time.time() - start_time
            return response

        # Calculate target heading
        start_heading = self.current_pose.theta
        target_heading = start_heading + request.angle

        # Normalize target heading to [-pi, pi]
        while target_heading > math.pi:
            target_heading -= 2 * math.pi
        while target_heading < -math.pi:
            target_heading += 2 * math.pi

        self.get_logger().info(f"Delegating rotation to RotateToPose action: {math.degrees(start_heading):.1f}° → {math.degrees(target_heading):.1f}°")

        # Check if rotation action server is available
        if not self.rotation_client.server_is_ready():
            self.get_logger().warn("RotateToPose action server not available")
            if not self.rotation_client.wait_for_server(timeout_sec=1.0):
                response.success = False
                response.message = "RotateToPose action server not available"
                response.final_heading = start_heading
                response.actual_rotation = 0.0
                response.execution_time = time.time() - start_time
                return response

        # Create and send goal to RotateToPose action
        goal_msg = RotateToPose.Goal()
        goal_msg.desired_angle = request.angle  # Relative rotation amount
        goal_msg.max_angular_speed = request.angular_speed if request.angular_speed > 0 else 2.0
        goal_msg.relative = True  # Relative rotation

        try:
            success = self.queue_action(self.rotation_client, goal_msg, "Relative Rotation")
            
            if success:
                response.success = True
                response.message = f"Relative rotation queued: {math.degrees(request.angle):.1f}°. Will execute when ready."
                response.final_heading = target_heading
                response.actual_rotation = request.angle
                response.execution_time = time.time() - start_time
            else:
                response.success = False
                response.message = "Failed to queue relative rotation"
                response.final_heading = start_heading
                response.actual_rotation = 0.0
                response.execution_time = time.time() - start_time
        except Exception as e:
            self.get_logger().error(f"Failed to queue rotation goal: {e}")
            response.success = False
            response.message = f"Failed to queue rotation: {str(e)}"
            response.final_heading = start_heading
            response.actual_rotation = 0.0
            response.execution_time = time.time() - start_time

        return response

    def handle_rotate_absolute_command(self, request, response):
        """Execute absolute rotation using RotateToPose action (blocking until completion)"""
        start_time = time.time()
        self.get_logger().info(f"Absolute rotation command: to {request.target_angle:.2f} radians ({math.degrees(request.target_angle):.1f} degrees)")

        # Get current pose
        if self.current_pose is None:
            response.success = False
            response.message = "No pose data available"
            response.final_heading = 0.0
            response.rotation_amount = 0.0
            response.execution_time = time.time() - start_time
            return response

        start_heading = self.current_pose.theta
        target_heading = request.target_angle

        # Normalize target heading to [-pi, pi]
        while target_heading > math.pi:
            target_heading -= 2 * math.pi
        while target_heading < -math.pi:
            target_heading += 2 * math.pi

        # Calculate rotation amount
        rotation_amount = target_heading - start_heading
        while rotation_amount > math.pi:
            rotation_amount -= 2 * math.pi
        while rotation_amount < -math.pi:
            rotation_amount += 2 * math.pi

        self.get_logger().info(f"Delegating rotation to RotateToPose action: {math.degrees(start_heading):.1f}° → {math.degrees(target_heading):.1f}° (rotation: {math.degrees(rotation_amount):.1f}°)")

        # Check if rotation action server is available
        if not self.rotation_client.server_is_ready():
            self.get_logger().warn("RotateToPose action server not available")
            if not self.rotation_client.wait_for_server(timeout_sec=1.0):
                response.success = False
                response.message = "RotateToPose action server not available"
                response.final_heading = start_heading
                response.rotation_amount = 0.0
                response.execution_time = time.time() - start_time
                return response

        # Create and send goal to RotateToPose action
        goal_msg = RotateToPose.Goal()
        goal_msg.desired_angle = request.target_angle  # Absolute target angle
        goal_msg.max_angular_speed = request.angular_speed if request.angular_speed > 0 else 2.0
        goal_msg.relative = False  # Absolute rotation

        try:
            success = self.queue_action(self.rotation_client, goal_msg, "Absolute Rotation")
            
            if success:
                response.success = True
                response.message = f"Absolute rotation queued: to {math.degrees(request.target_angle):.1f}°. Will execute when ready."
                response.final_heading = request.target_angle
                response.rotation_amount = rotation_amount
                response.execution_time = time.time() - start_time
            else:
                response.success = False
                response.message = "Failed to queue absolute rotation"
                response.final_heading = start_heading
                response.rotation_amount = 0.0
                response.execution_time = time.time() - start_time
        except Exception as e:
            self.get_logger().error(f"Failed to queue rotation goal: {e}")
            response.success = False
            response.message = f"Failed to queue rotation: {str(e)}"
            response.final_heading = start_heading
            response.rotation_amount = 0.0
            response.execution_time = time.time() - start_time

        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = MCPBridgeService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("MCP Bridge Service stopped cleanly")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
