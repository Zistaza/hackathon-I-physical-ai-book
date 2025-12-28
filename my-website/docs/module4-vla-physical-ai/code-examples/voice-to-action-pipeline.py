"""
Complete Voice-to-Action Pipeline Example

This example demonstrates a complete voice-to-action pipeline using OpenAI Whisper
for speech recognition and ROS 2 for robot action execution.
"""

import whisper
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from audio_common_msgs.msg import AudioData
import numpy as np
import io
import wave
from threading import Lock

class VoiceToActionPipeline(Node):
    """
    Complete voice-to-action pipeline integrating OpenAI Whisper with ROS 2
    """

    def __init__(self):
        super().__init__('voice_to_action_pipeline')

        # Initialize Whisper model for speech recognition
        self.get_logger().info("Loading Whisper model...")
        self.whisper_model = whisper.load_model("base")
        self.get_logger().info("Whisper model loaded successfully")

        # Initialize ROS 2 components
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Audio subscription for voice input
        self.audio_sub = self.create_subscription(
            AudioData,
            '/audio',
            self.audio_callback,
            10
        )

        # Status publisher for system state
        self.status_pub = self.create_publisher(String, '/voice_to_action/status', 10)

        # Thread safety for processing
        self.processing_lock = Lock()

        # Confidence threshold for command execution
        self.confidence_threshold = 0.7

        # Supported commands mapping
        self.command_mapping = {
            'move forward': self.execute_move_forward,
            'move backward': self.execute_move_backward,
            'turn left': self.execute_turn_left,
            'turn right': self.execute_turn_right,
            'stop': self.execute_stop,
            'go forward': self.execute_move_forward,
            'go back': self.execute_move_backward,
        }

        self.get_logger().info("Voice-to-Action pipeline initialized")

    def audio_callback(self, msg):
        """
        Handle incoming audio data and process voice commands
        """
        with self.processing_lock:
            try:
                # Convert audio data to format suitable for Whisper
                audio_data = self.convert_audio_data(msg)

                # Perform speech recognition
                result = self.whisper_model.transcribe(audio_data)
                recognized_text = result['text'].strip().lower()
                confidence = result.get('avg_logprob', -1.0)  # Use log probability as confidence

                self.get_logger().info(f"Recognized: '{recognized_text}' with confidence: {confidence}")

                # Publish status
                status_msg = String()
                status_msg.data = f"Recognized: {recognized_text} (conf: {confidence:.2f})"
                self.status_pub.publish(status_msg)

                # Check confidence threshold
                if confidence < self.confidence_threshold:
                    self.get_logger().warn(f"Low confidence recognition: {confidence}")
                    return

                # Process the recognized command
                self.process_command(recognized_text)

            except Exception as e:
                self.get_logger().error(f"Error processing audio: {e}")

    def convert_audio_data(self, audio_msg):
        """
        Convert ROS AudioData message to format suitable for Whisper
        """
        # Convert byte data to numpy array
        audio_array = np.frombuffer(audio_msg.data, dtype=np.int16)

        # Convert to float32 in range [-1, 1]
        audio_float = audio_array.astype(np.float32) / 32768.0

        # Whisper expects 16kHz audio, so if input is different, resample
        # For simplicity, assuming input is already 16kHz
        # In practice, you might need to resample using librosa or similar

        return audio_float

    def process_command(self, recognized_text):
        """
        Process recognized text and execute appropriate robot action
        """
        # Simple command matching - in practice, this could use more sophisticated NLP
        for command, action_func in self.command_mapping.items():
            if command in recognized_text:
                self.get_logger().info(f"Executing command: {command}")
                action_func()
                return

        # If no command matched, log the unrecognized text
        self.get_logger().info(f"Unrecognized command: {recognized_text}")

    def execute_move_forward(self):
        """Execute forward movement command"""
        cmd = Twist()
        cmd.linear.x = 0.5  # 0.5 m/s forward
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Moving forward")

        # Stop after 1 second
        self.create_timer(1.0, self.execute_stop)

    def execute_move_backward(self):
        """Execute backward movement command"""
        cmd = Twist()
        cmd.linear.x = -0.5  # 0.5 m/s backward
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Moving backward")

        # Stop after 1 second
        self.create_timer(1.0, self.execute_stop)

    def execute_turn_left(self):
        """Execute left turn command"""
        cmd = Twist()
        cmd.angular.z = 0.5  # 0.5 rad/s counter-clockwise
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Turning left")

        # Stop after 1 second
        self.create_timer(1.0, self.execute_stop)

    def execute_turn_right(self):
        """Execute right turn command"""
        cmd = Twist()
        cmd.angular.z = -0.5  # 0.5 rad/s clockwise
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Turning right")

        # Stop after 1 second
        self.create_timer(1.0, self.execute_stop)

    def execute_stop(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Stopping")

def main(args=None):
    rclpy.init(args=args)

    # Create and run the voice-to-action pipeline
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