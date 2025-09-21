#!/usr/bin/env python3

# Import necessary modules from ROS 2 Python API
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from cpp_node.action import GoToPose  # Import custom action message GoToPose from cpp_node


class GoToPoseClient(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'goto_pose_client'
        super().__init__('goto_pose_client')

        # Create an ActionClient for the GoToPose action server
        self._action_client = ActionClient(self, GoToPose, 'GoToPose')
        
        # Flag to track if goal is completed
        self.goal_completed = False
        self.goal_success = False

    def send_goal(self, x, y):
        # Reset goal completion flag
        self.goal_completed = False
        self.goal_success = False
        
        # Wait for the action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self.goal_completed = True
            return False

        # Create a goal message to send to the action server
        goal_msg = GoToPose.Goal()
        goal_msg.desired_x_pos = x  # Set desired x position
        goal_msg.desired_y_pos = y  # Set desired y position

        # Log the goal message being sent
        self.get_logger().info(f'🚀 Sending goal: x={x}, y={y}')

        # Send the goal asynchronously and add a callback for feedback
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        # Add a done callback to handle the result after goal is accepted
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        # Callback function to handle the result of the goal response
        self.goal_handle: ClientGoalHandle = future.result()  # Get the goal handle from the future result
        if self.goal_handle.accepted:
            # If the goal was accepted, log the success and wait for the result
            self.get_logger().info('✅ Goal accepted! Turtle is moving...')
            self.goal_handle.get_result_async().add_done_callback(self.get_result_callback)
        else:
            # If the goal was rejected, log the rejection
            self.get_logger().info('❌ Goal rejected!')
            self.goal_completed = True  # Mark as completed even if rejected

    def get_result_callback(self, future):
        # Callback function to handle the final result of the goal execution
        result = future.result().result  # Get the result of the goal
        self.goal_success = result.success
        self.get_logger().info("Result " + str(result.success))  # Log the result of the goal execution
        if result.success:
            self.get_logger().info('🎉 Goal Reached Successfully! 🎉')
        else:
            self.get_logger().info('❌ Goal Failed!')
        
        # Mark goal as completed to trigger new input
        self.goal_completed = True

    def feedback_callback(self, feedback_msg):
        # Callback function to handle feedback messages during goal execution
        x_feedback = feedback_msg.feedback.current_x_pos  # Get the current x position from feedback
        y_feedback = feedback_msg.feedback.current_y_pos  # Get the current y position from feedback
        # Log the received feedback
        self.get_logger().info(f"Feedback received - Current X Position: {x_feedback}, Current Y Position: {y_feedback}")


def get_user_input():
    """Get user input for target coordinates with validation."""
    while True:
        try:
            print("\n" + "="*50)
            print("🎯 TURTLE NAVIGATION SYSTEM 🐢")
            print("="*50)
            print("Enter target coordinates (valid range: -11 to 11)")
            print("Or type 'quit' to exit")
            
            x_input = input("Enter desired X position: ").strip()
            if x_input.lower() in ['quit', 'exit', 'q']:
                return None, None
                
            y_input = input("Enter desired Y position: ").strip()
            if y_input.lower() in ['quit', 'exit', 'q']:
                return None, None
            
            x = float(x_input)
            y = float(y_input)
            
            # Validate coordinates are within bounds
            if abs(x) > 11 or abs(y) > 11:
                print("⚠️  Warning: Coordinates outside recommended range (-11 to 11)")
                confirm = input("Continue anyway? (y/n): ").strip().lower()
                if confirm not in ['y', 'yes']:
                    continue
            
            return x, y
            
        except ValueError:
            print("❌ Invalid input! Please enter numeric values.")
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            return None, None

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the GoToPoseClient node
    client = GoToPoseClient()
    
    print("🚀 Starting Turtle Controller Client...")
    print("Waiting for action server...")
    
    try:
        # Continuous goal loop
        while rclpy.ok():
            # Get user input for new goal
            x, y = get_user_input()
            
            # Check if user wants to quit
            if x is None or y is None:
                break
            
            # Send the goal
            if client.send_goal(x, y):
                # Wait for goal completion
                while rclpy.ok() and not client.goal_completed:
                    rclpy.spin_once(client, timeout_sec=0.1)
                
                # Show completion message
                if client.goal_completed:
                    if client.goal_success:
                        print("✅ Goal completed successfully!")
                    else:
                        print("❌ Goal failed or was aborted.")
                    print("Ready for next goal...")
            else:
                print("❌ Failed to send goal. Make sure the turtle controller is running.")
                break
                
    except KeyboardInterrupt:
        print("\n👋 Shutting down gracefully...")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Call the main function when the script is executed
    main()


