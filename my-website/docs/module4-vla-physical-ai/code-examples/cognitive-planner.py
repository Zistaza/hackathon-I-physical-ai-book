"""
Cognitive Planning with LLMs Example

This example demonstrates cognitive planning using LLMs to translate
natural language commands into ROS 2 action sequences.
"""

import openai
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from typing import List, Dict, Any
import json

class CognitivePlannerNode(Node):
    """
    Cognitive planning system using LLMs to translate natural language to robot actions
    """

    def __init__(self):
        super().__init__('cognitive_planner')

        # Initialize OpenAI client (or use local model)
        # self.openai_client = openai.OpenAI(api_key="your-api-key")

        # Publisher for action commands
        self.action_pub = self.create_publisher(String, '/robot/action', 10)

        # Subscription for natural language commands
        self.command_sub = self.create_subscription(
            String,
            '/natural_language_command',
            self.command_callback,
            10
        )

        # Publisher for planning status
        self.status_pub = self.create_publisher(String, '/cognitive_planner/status', 10)

        # Store world state
        self.world_state = {
            "robot_position": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "objects": [],
            "robot_capabilities": ["navigation", "manipulation", "grasping"],
            "navigation_map": {}
        }

        # Store action sequence
        self.current_action_sequence = []
        self.current_action_index = 0

        self.get_logger().info("Cognitive Planner initialized")

    def command_callback(self, msg):
        """
        Handle incoming natural language commands
        """
        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Update status
        status_msg = String()
        status_msg.data = f"Processing command: {command}"
        self.status_pub.publish(status_msg)

        # Generate action sequence using LLM
        action_sequence = self.generate_action_sequence(command, self.world_state)

        if action_sequence:
            self.get_logger().info(f"Generated action sequence with {len(action_sequence)} actions")
            self.execute_action_sequence(action_sequence)
        else:
            self.get_logger().error("Failed to generate action sequence")
            status_msg.data = "Failed to generate action sequence"
            self.status_pub.publish(status_msg)

    def generate_action_sequence(self, command: str, world_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Use LLM to generate action sequence from natural language command
        """
        try:
            # Create a detailed prompt for the LLM
            prompt = self.create_planning_prompt(command, world_state)

            # For this example, we'll simulate the LLM response
            # In practice, you would call the LLM API here
            action_sequence = self.simulate_llm_planning(command, world_state)

            return action_sequence

        except Exception as e:
            self.get_logger().error(f"Error in LLM planning: {e}")
            return []

    def create_planning_prompt(self, command: str, world_state: Dict[str, Any]) -> str:
        """
        Create a structured prompt for the LLM cognitive planner
        """
        prompt = f"""
        You are a cognitive planning system for a robot. Your task is to convert a natural language command into a sequence of executable actions for the robot.

        The robot has these capabilities:
        - Navigation: Moving to specific locations
        - Manipulation: Moving objects
        - Grasping: Picking up objects
        - Object identification: Recognizing objects in the environment

        Current world state:
        - Robot position: {world_state['robot_position']}
        - Objects in environment: {world_state['objects']}
        - Robot capabilities: {world_state['robot_capabilities']}

        Natural language command: "{command}"

        Please provide a sequence of actions to accomplish this command. Each action should be in JSON format with:
        - action_type: The type of action (navigate, identify_object, grasp_object, place_object, etc.)
        - parameters: Specific parameters for the action
        - description: A human-readable description of what the action does

        Return only a JSON array of actions, nothing else.
        """
        return prompt

    def simulate_llm_planning(self, command: str, world_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Simulate LLM planning for demonstration purposes
        In practice, this would call an actual LLM API
        """
        # This is a simplified simulation - in practice, this would be a real LLM call
        command_lower = command.lower()

        if "bring" in command_lower or "fetch" in command_lower or "get" in command_lower:
            # Extract object information from command
            object_name = self.extract_object_name(command_lower)

            return [
                {
                    "action_type": "identify_object",
                    "parameters": {"object_name": object_name},
                    "description": f"Identify the {object_name} in the environment"
                },
                {
                    "action_type": "navigate",
                    "parameters": {"target_object": object_name},
                    "description": f"Navigate to the {object_name}"
                },
                {
                    "action_type": "grasp_object",
                    "parameters": {"object_name": object_name},
                    "description": f"Grasp the {object_name}"
                },
                {
                    "action_type": "navigate",
                    "parameters": {"target_location": "user"},
                    "description": "Return to user location"
                },
                {
                    "action_type": "place_object",
                    "parameters": {"location": "user"},
                    "description": f"Place the {object_name} near the user"
                }
            ]

        elif "go to" in command_lower or "move to" in command_lower:
            # Extract destination from command
            destination = self.extract_destination(command_lower)

            return [
                {
                    "action_type": "navigate",
                    "parameters": {"target_location": destination},
                    "description": f"Navigate to {destination}"
                }
            ]

        elif "clean" in command_lower or "tidy" in command_lower:
            return [
                {
                    "action_type": "identify_objects",
                    "parameters": {"object_types": ["trash", "misplaced"]},
                    "description": "Identify objects that need cleaning"
                },
                {
                    "action_type": "navigate",
                    "parameters": {"target_object": "first_object"},
                    "description": "Navigate to first object to clean"
                },
                {
                    "action_type": "grasp_object",
                    "parameters": {"object_name": "first_object"},
                    "description": "Grasp the object"
                },
                {
                    "action_type": "navigate",
                    "parameters": {"target_location": "trash_bin"},
                    "description": "Move to trash bin"
                },
                {
                    "action_type": "place_object",
                    "parameters": {"location": "trash_bin"},
                    "description": "Dispose of the object"
                }
            ]

        else:
            # Default to simple navigation if command is unclear
            return [
                {
                    "action_type": "navigate",
                    "parameters": {"target_location": "default"},
                    "description": "Navigate to default location"
                }
            ]

    def extract_object_name(self, command: str) -> str:
        """
        Extract object name from command (simplified approach)
        In practice, this would use more sophisticated NLP
        """
        # Simple keyword extraction
        keywords = ["cup", "bottle", "book", "phone", "keys", "ball", "box", "toy"]
        for keyword in keywords:
            if keyword in command:
                # Look for color descriptors
                colors = ["red", "blue", "green", "yellow", "black", "white"]
                for color in colors:
                    if color in command:
                        return f"{color} {keyword}"
                return keyword
        return "object"

    def extract_destination(self, command: str) -> str:
        """
        Extract destination from command (simplified approach)
        """
        locations = ["kitchen", "living room", "bedroom", "office", "bathroom", "dining room"]
        for location in locations:
            if location in command:
                return location
        return "default"

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]):
        """
        Execute the generated action sequence
        """
        self.current_action_sequence = action_sequence
        self.current_action_index = 0

        if action_sequence:
            self.execute_next_action()

    def execute_next_action(self):
        """
        Execute the next action in the sequence
        """
        if self.current_action_index >= len(self.current_action_sequence):
            # All actions completed
            self.get_logger().info("Action sequence completed")
            status_msg = String()
            status_msg.data = "Action sequence completed successfully"
            self.status_pub.publish(status_msg)
            return

        current_action = self.current_action_sequence[self.current_action_index]

        self.get_logger().info(f"Executing action {self.current_action_index + 1}/{len(self.current_action_sequence)}: {current_action['description']}")

        # Publish the action to the robot
        action_msg = String()
        action_msg.data = json.dumps(current_action)
        self.action_pub.publish(action_msg)

        # Update status
        status_msg = String()
        status_msg.data = f"Executing: {current_action['description']}"
        self.status_pub.publish(status_msg)

        # For this example, we'll simulate completion after a delay
        # In practice, you would wait for action completion feedback
        self.current_action_index += 1
        self.create_timer(2.0, self.execute_next_action)  # Simulate action completion after 2 seconds

def main(args=None):
    rclpy.init(args=args)

    # Create and run the cognitive planner
    planner = CognitivePlannerNode()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()