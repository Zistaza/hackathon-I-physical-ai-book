---
title: Voice-to-Action Pipelines with OpenAI Whisper and ROS 2
sidebar_label: Voice-to-Action Pipelines
---

## Learning Objectives
- Implement a voice-to-action pipeline using OpenAI Whisper
- Integrate speech recognition with ROS 2 action sequences
- Design API contracts for speech recognition systems
- Create annotated code examples for voice-to-action processing
- Troubleshoot common issues in voice-to-action systems

## Introduction
This chapter covers the implementation of voice-to-action pipelines using OpenAI Whisper for speech recognition and ROS 2 for action execution. Students will learn how to process voice commands and translate them into executable robot actions.

The voice-to-action pipeline is fundamental to creating robots that respond naturally to human speech. By combining Whisper’s speech recognition capabilities with ROS 2's robust action framework, we can create systems that understand commands and execute appropriate behaviors.

## OpenAI Whisper Integration
OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system that converts spoken language into text. In VLA systems, Whisper performs the critical first step in processing voice commands.

### Whisper Models
- **Tiny**: Fastest, least accurate (75MB)  
- **Base**: Good balance of speed and accuracy (145MB)  
- **Small**: Better accuracy (485MB)  
- **Medium**: High accuracy (1.5GB)  
- **Large**: Highest accuracy (3.0GB)  

### Integration with VLA Systems
- Converts spoken commands to text  
- Provides confidence scores for recognition quality  
- Handles multiple languages  
- Processes audio in real-time or batch modes  

### Key Considerations
- Audio quality significantly affects recognition accuracy  
- Background noise can reduce performance  
- Different accents may require model fine-tuning  
- Real-time processing requires efficient model selection  

## Speech Recognition Concepts
### Audio Preprocessing
- Sample rate: Whisper expects 16kHz audio  
- Audio format: WAV or similar  
- Noise reduction: Optional preprocessing to improve quality  

### Recognition Process
1. **Audio Input**: Capturing audio from microphones or other sources  
2. **Feature Extraction**: Converting audio to mel-spectrogram features  
3. **Model Processing**: Using Whisper to decode text  
4. **Post-processing**: Cleaning and formatting the recognized text  

### Confidence Metrics
- **Token-level timestamps**: When words were spoken  
- **Confidence scores**: Model’s recognition confidence  
- **No speech probability**: Likelihood that audio contains speech  

## Whisper API Integration
### Installation
```bash
pip install openai-whisper
Basic Usage
python
Copy code
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio_file.wav")
print(result["text"])
Advanced Configuration
python
Copy code
# Specific language
result = model.transcribe("audio_file.wav", language="en")

# Word-level timestamps
result = model.transcribe("audio_file.wav", word_timestamps=True)

# Custom temperature
result = model.transcribe("audio_file.wav", temperature=0.0)
Real-time Processing Notes
Process audio in chunks

Use streaming input

Handle buffer management

Implement voice activity detection

ROS 2 Action Sequence Generation
ROS 2 Action Architecture
Goal: Request to perform an action

Feedback: Progress updates

Result: Final outcome

Example Action Client
python
Copy code
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class VoiceToActionNode:
    def __init__(self):
        self.node = rclpy.create_node('voice_to_action')
        self.action_client = ActionClient(
            self.node,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_navigation_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        return future
API Contract for Speech Recognition
Input Contract
Audio format: WAV, 16kHz, mono

Maximum duration: 30 seconds

Encoding: PCM 16-bit

Processing Contract
Response time: <2 seconds for <5s audio

Confidence threshold: 0.7

Clear error messages for failure modes

Output Contract
Text response with confidence score

Mapped robot action sequence

Metadata: processing time, detected language

Example Response
json
Copy code
{
  "recognized_text": "Move forward 1 meter",
  "confidence": 0.92,
  "detected_language": "en",
  "processing_time": 1.23,
  "action_sequence": [
    {
      "action_type": "navigation",
      "parameters": {
        "direction": "forward",
        "distance": 1.0,
        "speed": 0.5
      }
    }
  ]
}
Complete Annotated Code Example
python
Copy code
import whisper
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import numpy as np
from threading import Lock

class VoiceToActionPipeline(Node):
    def __init__(self):
        super().__init__('voice_to_action_pipeline')
        self.whisper_model = whisper.load_model("base")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.audio_sub = self.create_subscription(AudioData, '/audio', self.audio_callback, 10)
        self.status_pub = self.create_publisher(String, '/voice_to_action/status', 10)
        self.processing_lock = Lock()
        self.confidence_threshold = 0.7
        self.command_mapping = {
            'move forward': self.execute_move_forward,
            'move backward': self.execute_move_backward,
            'turn left': self.execute_turn_left,
            'turn right': self.execute_turn_right,
            'stop': self.execute_stop,
        }

    def audio_callback(self, msg):
        with self.processing_lock:
            audio_array = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
            result = self.whisper_model.transcribe(audio_array)
            text = result['text'].strip().lower()
            confidence = result.get('avg_logprob', -1.0)
            status_msg = String()
            status_msg.data = f"Recognized: {text} (conf: {confidence:.2f})"
            self.status_pub.publish(status_msg)
            if confidence < self.confidence_threshold:
                self.get_logger().warn(f"Low confidence: {confidence}")
                return
            for cmd, func in self.command_mapping.items():
                if cmd in text:
                    func()
                    return

    def execute_move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)

    def execute_move_backward(self):
        cmd = Twist()
        cmd.linear.x = -0.5
        self.cmd_vel_pub.publish(cmd)

    def execute_turn_left(self):
        cmd = Twist()
        cmd.angular.z = 0.5
        self.cmd_vel_pub.publish(cmd)

    def execute_turn_right(self):
        cmd = Twist()
        cmd.angular.z = -0.5
        self.cmd_vel_pub.publish(cmd)

    def execute_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    pipeline = VoiceToActionPipeline()
    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
Implementation Steps
Install dependencies:

bash
Copy code
pip install openai-whisper torch rclpy
source /opt/ros/humble/setup.bash
Load Whisper model and test transcription

Connect audio input and preprocess

Map commands to robot actions

Set up ROS 2 node and publishers

Test pipeline with different commands

Troubleshooting
No audio: Check microphone and topics

Wrong recognition: Ensure quiet environment and correct format

High CPU usage: Use smaller Whisper models

Command errors: Verify mapping logic and ROS 2 topics

Exercises
Recognize simple commands: "move forward", "stop"

Add new commands and test

Adjust confidence thresholds and evaluate performance

Implement real-time audio streaming

Handle noisy audio and provide feedback