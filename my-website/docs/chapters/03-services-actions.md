# Chapter 3: Services & Actions for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Implement ROS 2 services for synchronous communication
- Design and use ROS 2 actions for goal-oriented tasks
- Integrate services and actions with Python agents
- Apply services and actions to humanoid robot control

## 1. ROS 2 Services

### Service definition and implementation

Services in ROS 2 provide a synchronous request-response communication pattern. They are ideal for operations that need to return a result immediately, such as:

- Calculating inverse kinematics for a robot pose
- Requesting sensor calibration
- Querying robot state information
- Executing a specific manipulation task

### Client-server communication patterns

In the service pattern, a client sends a request to a service server, which processes the request and returns a response. This is different from the asynchronous topic-based communication.

### Error handling and timeout management

Service calls can fail or timeout, so it's important to implement proper error handling and timeout management in your client code.

### Service introspection and debugging

ROS 2 provides tools to inspect and debug services, which is important for humanoid robotics applications where reliability is critical.

## 2. ROS 2 Actions

### Action definition and structure

Actions are designed for long-running tasks that require feedback and the ability to cancel. They follow a goal-feedback-result pattern and are ideal for tasks like:

- Navigation to a target location
- Manipulation of objects
- Calibration procedures
- Complex multi-step robot behaviors

An action definition includes:
- **Goal**: The request sent to the action server
- **Feedback**: Information sent back during execution
- **Result**: The final outcome of the action

### Action server implementation

Action servers handle goals, execute them, provide feedback, and return results. They also handle preemption requests.

### Action client implementation

Action clients send goals to action servers, receive feedback during execution, and get the final result.

### Canceling and preemption handling

Actions can be canceled or preempted, which is important for humanoid robots that need to react to changing conditions.

## 3. Service and Action Design Patterns

### When to use services vs actions vs topics

- **Topics**: Use for continuous data streams, sensor data, robot state publishing
- **Services**: Use for request-response operations that complete quickly
- **Actions**: Use for long-running operations that need feedback or cancellation

### Best practices for API design

- Keep service requests and responses concise
- Use appropriate data types for the application
- Implement proper error handling
- Consider the timing and reliability requirements

### Performance considerations

- Services block the calling thread until a response is received
- Actions can run in the background without blocking
- Consider the impact of synchronous vs asynchronous operations on system performance

### Integration with humanoid robot control systems

Services and actions are particularly important for humanoid robotics where many operations are goal-oriented and require feedback, such as walking to a location or grasping an object.

## 4. Python Integration with rclpy

### Service and action client implementation in Python

Using rclpy, you can implement both service clients and action clients in Python, which is useful for creating control agents that interact with humanoid robots.

### Error handling in Python clients

Python clients should handle various error conditions including timeouts, service unavailability, and action preemption.

### Integration with external systems

Python's rich ecosystem allows for easy integration of ROS 2 services and actions with external systems like vision libraries, planning algorithms, and user interfaces.

## Code Examples

### Basic service server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

class BasicServiceServer(Node):
    def __init__(self):
        super().__init__('basic_service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = BasicServiceServer()
    rclpy.spin(service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic service client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class BasicServiceClient(Node):
    def __init__(self):
        super().__init__('basic_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    service_client = BasicServiceClient()
    response = service_client.send_request(2, 3)
    service_client.get_logger().info(f'Result: {response.sum}')
    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises

### Exercise 3.1: Create a service that calculates humanoid robot inverse kinematics

Create a service that takes a desired end-effector pose and calculates the joint angles needed to achieve it.

### Exercise 3.2: Implement an action that moves a humanoid robot to a target pose

Create an action that accepts a target pose, moves the robot toward it, provides feedback during the movement, and returns the result.

### Exercise 3.3: Design a service for humanoid robot calibration

Create a service that calibrates sensors or adjusts robot parameters.

### Exercise 3.4: Integrate services and actions with a Python-based control agent

Combine services and actions in a Python agent that controls a humanoid robot for a complex task.

## Summary

This chapter covered ROS 2 services and actions, which are essential for synchronous communication and goal-oriented tasks in humanoid robotics. Services provide request-response communication for quick operations, while actions handle long-running tasks with feedback and cancellation capabilities. These patterns are crucial for implementing complex humanoid robot behaviors.

## References

- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html#services)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Concepts/About-Actions.html)
- [Action Client and Server Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions.html)