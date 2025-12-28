---
title: Cognitive Planning with LLMs for Natural Language to Robot Actions
sidebar_label: Cognitive Planning with LLMs
---

## Learning Objectives for Cognitive Planning
- Implement cognitive planning using LLMs to translate natural language commands into ROS 2 action sequences
- Understand LLM integration concepts for robotics applications
- Apply natural language processing techniques for command interpretation
- Design command interpretation APIs for complex tasks
- Generate action sequences from high-level natural language commands
- Understand task decomposition concepts for complex robot behaviors
- Create annotated code examples for LLM-based cognitive planning
- Implement LLM integration steps for robotics applications
- Complete exercises for cognitive planning systems

## Introduction
This chapter focuses on cognitive planning using Large Language Models (LLMs) to translate natural language commands into sequences of ROS 2 actions that achieve complex tasks. Cognitive planning represents a higher level of VLA system capability, where LLMs serve as reasoning engines that can interpret complex, multi-step commands and decompose them into executable robot actions.

Cognitive planning bridges the gap between high-level human instructions and low-level robot control. Unlike simple command mapping, cognitive planning enables robots to understand complex, abstract instructions and reason about how to achieve the requested goals using their available capabilities.

## LLM Integration Concepts
### Role of LLMs in Robotics
Large Language Models serve as cognitive engines in VLA systems by:
- **Understanding context**: Interpreting commands within the current world state
- **Reasoning**: Determining appropriate sequences of actions to achieve goals
- **Planning**: Breaking down complex tasks into manageable subtasks
- **Adapting**: Adjusting plans based on environmental constraints and feedback

### LLM Architectures for Robotics
Different LLM architectures have various applications in robotics:
- **Instruction-tuned models**: Better for following explicit commands
- **Conversational models**: Better for interactive planning
- **Specialized models**: Fine-tuned for robotics-specific tasks
- **Multimodal models**: Can incorporate visual information for better planning

### Integration Approaches
LLMs can be integrated into robotics systems through:
- **Cloud APIs**: Using services like OpenAI, Anthropic, or Google
- **Local models**: Running open-source models on robot hardware
- **Hybrid approaches**: Using local models for simple tasks, cloud for complex reasoning

## Natural Language Processing Explanation
### Command Interpretation Pipeline
The process of converting natural language commands to robot actions involves several stages:

1. **Command Parsing**: Breaking down the command into semantic components
2. **Intent Recognition**: Identifying the high-level goal
3. **Entity Extraction**: Identifying objects, locations, and parameters
4. **Context Integration**: Incorporating world state information
5. **Action Planning**: Generating sequences of executable actions

### Semantic Understanding
LLMs provide semantic understanding capabilities that go beyond keyword matching:
- **Synonym recognition**: Understanding different ways to express the same command
- **Implicit knowledge**: Using common-sense reasoning about the world
- **Temporal reasoning**: Understanding time-based commands
- **Spatial reasoning**: Understanding location and movement commands

### Context Awareness
Effective cognitive planning requires LLMs to understand:
- **Current world state**: What objects are present, robot location, etc.
- **Robot capabilities**: What actions the robot can perform
- **Environmental constraints**: What is possible in the current environment
- **Task history**: Previous actions and their outcomes

## Command Interpretation API Details
### API Design Principles
The command interpretation API should be:
- **Consistent**: Same input/output format regardless of command complexity
- **Extensible**: Easy to add new command types and capabilities
- **Robust**: Handles ambiguous or incomplete commands gracefully
- **Traceable**: Provides reasoning for action sequences

### Input Format
The API accepts natural language commands with optional context:

```json
{
  "command": "Bring me the red cup from the kitchen table",
  "world_state": {
    "robot_position": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "objects": [
      {"name": "red_cup", "type": "cup", "color": "red", "location": "kitchen_table", "pose": {"x": 1.5, "y": 2.0}},
      {"name": "kitchen_table", "type": "furniture", "pose": {"x": 1.5, "y": 2.0}}
    ],
    "robot_capabilities": ["navigation", "manipulation", "grasping"]
  },
  "context": {
    "user_location": {"x": -1.0, "y": 0.0},
    "previous_commands": ["go to kitchen", "find red cup"]
  }
}
```

### Output Format
The API returns a structured action sequence:

```json
{
  "success": true,
  "action_sequence": [
    {
      "action_type": "navigate",
      "parameters": {"target_location": "kitchen_table"},
      "description": "Navigate to the kitchen table"
    },
    {
      "action_type": "identify_object",
      "parameters": {"object_name": "red_cup"},
      "description": "Identify the red cup on the table"
    },
    {
      "action_type": "grasp_object",
      "parameters": {"object_name": "red_cup", "approach_direction": "top"},
      "description": "Grasp the red cup"
    },
    {
      "action_type": "navigate",
      "parameters": {"target_location": "user_location"},
      "description": "Return to user location"
    },
    {
      "action_type": "place_object",
      "parameters": {"location": "user_location"},
      "description": "Place the cup near the user"
    }
  ],
  "confidence": 0.95,
  "reasoning": "The command requires fetching an object. The plan involves navigating to the object, grasping it, and returning to the user."
}
```

## Action Sequence Generation Explanation
### Planning Algorithms
Action sequence generation can use various approaches:
- **Template-based**: Predefined patterns for common commands
- **Rule-based**: Logical rules for command decomposition
- **Learning-based**: Trained models for action planning
- **LLM-based**: Natural language reasoning for planning

### Sequence Optimization
Effective action sequences should consider:
- **Efficiency**: Minimizing travel distance and execution time
- **Safety**: Avoiding collisions and dangerous situations
- **Feasibility**: Ensuring actions are within robot capabilities
- **Robustness**: Handling failures and alternative approaches

### Multi-step Planning
Complex commands require:
- **Task decomposition**: Breaking high-level goals into subtasks
- **Dependency management**: Ensuring prerequisites are met
- **Resource allocation**: Managing robot time and capabilities
- **Contingency planning**: Alternative approaches if primary plan fails

## Task Decomposition Concepts
### Hierarchical Task Networks
Task decomposition follows a hierarchical approach:
- **High-level tasks**: Overall goals (e.g., "set the table")
- **Mid-level tasks**: Major components (e.g., "place plates", "place utensils")
- **Low-level tasks**: Individual actions (e.g., "pick up plate", "move to table")

### Subtask Dependencies
Tasks have various dependency relationships:
- **Sequential**: Tasks must be completed in order
- **Parallelizable**: Tasks can be executed simultaneously
- **Conditional**: Tasks depend on the success of other tasks
- **Iterative**: Tasks repeat until a condition is met

### World State Integration
Task decomposition must consider:
- **Current state**: What is already accomplished
- **Required state**: What is needed to proceed
- **Available resources**: What tools and objects are accessible
- **Constraints**: Physical and logical limitations

## Annotated Code Example for Cognitive Planning
```python
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

class LLMIntegrationManager:
    """
    Manager for LLM integration in cognitive planning
    Handles different LLM backends and configuration
    """

    def __init__(self, llm_type: str = "openai"):
        self.llm_type = llm_type
        self.client = None
        self.setup_llm_client()

    def setup_llm_client(self):
        """
        Set up the appropriate LLM client based on type
        """
        if self.llm_type == "openai":
            # import openai
            # self.client = openai.OpenAI()
            pass  # Placeholder for actual client setup
        elif self.llm_type == "local":
            # Setup for local models like Llama
            pass
        elif self.llm_type == "anthropic":
            # Setup for Anthropic models
            pass

    def generate_response(self, prompt: str) -> str:
        """
        Generate response from LLM based on prompt
        """
        if self.llm_type == "openai":
            # response = self.client.chat.completions.create(
            #     model="gpt-3.5-turbo",
            #     messages=[{"role": "user", "content": prompt}],
            #     max_tokens=500
            # )
            # return response.choices[0].message.content
            return "Simulated response"  # Placeholder
        else:
            # Handle other LLM types
            return "Simulated response"

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
```

## Implementation Steps for LLM Integration
### Step 1: LLM Setup
1. Choose your LLM provider (OpenAI, Anthropic, local model, etc.)
2. Set up API keys or model files
3. Install required dependencies:
```bash
pip install openai  # for OpenAI models
# or
pip install transformers torch  # for local models
```

### Step 2: World State Integration
1. Set up world state publisher/subscription
2. Ensure robot can provide current state information
3. Integrate with perception systems for object detection

### Step 3: Command Processing
1. Create natural language command interface
2. Implement command parsing and validation
3. Set up error handling for ambiguous commands

### Step 4: Action Sequence Generation
1. Design the LLM prompt for action planning
2. Implement response parsing and validation
3. Create fallback strategies for LLM failures

### Step 5: Action Execution
1. Set up action execution pipeline
2. Implement action completion feedback
3. Add error handling and recovery

### Step 6: Testing and Validation
1. Test with various natural language commands
2. Validate action sequence correctness
3. Evaluate planning efficiency and safety

## Exercises for Cognitive Planning
### Exercise 1: Basic Command Interpretation
1. Implement a simple LLM-based command interpreter
2. Test with basic commands like "go to kitchen" or "pick up the red ball"
3. Verify that correct action sequences are generated

### Exercise 2: Complex Task Planning
1. Create commands with multiple steps (e.g., "Go to the kitchen, find a cup, fill it with water, and bring it to me")
2. Verify that the system decomposes the task correctly
3. Test each step of the generated action sequence

### Exercise 3: Context-Aware Planning
1. Modify the system to consider world state in planning
2. Test commands that depend on current environment (e.g., "go to the room with the blue chair")
3. Verify that the system uses world state appropriately

### Exercise 4: Error Handling
1. Test the system with ambiguous or impossible commands
2. Implement graceful degradation for unclear commands
3. Add user feedback for planning failures

### Exercise 5: Performance Optimization
1. Measure planning time for different command complexities
2. Optimize prompt engineering for better performance
3. Implement caching for common command patterns

## Summary
This chapter has covered cognitive planning using LLMs to translate natural language commands into ROS 2 action sequences. Students have learned how to integrate LLMs as reasoning engines in VLA systems, design effective command interpretation APIs, and generate complex action sequences from high-level human instructions.

Cognitive planning represents a significant advancement in VLA systems, moving beyond simple command mapping to enable true natural language understanding and reasoning. The integration of LLMs allows robots to interpret complex, multi-step commands and decompose them into executable actions, significantly expanding the range of tasks that can be performed through natural language interaction.

The annotated code example provides a foundation for implementing cognitive planning systems that can be extended and adapted for specific robot platforms and applications. Understanding these concepts is crucial for building advanced VLA systems that can handle complex, real-world tasks through natural language commands.