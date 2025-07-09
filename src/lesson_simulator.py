#!/usr/bin/env python3
"""
Lesson Simulator - ROS Node for simulating lesson content and states
to test the robot face bridge integration.

This node publishes:
1. Lesson content with different types
2. Lesson state changes
3. Progress updates
4. Speech requests
"""

import rospy
import json
import random
from std_msgs.msg import String, Float32

class LessonSimulator:
    def __init__(self):
        rospy.init_node('lesson_simulator', anonymous=True)
        
        # Publishers
        self.lesson_content_pub = rospy.Publisher('/lesson/content', String, queue_size=10)
        self.lesson_state_pub = rospy.Publisher('/lesson/state', String, queue_size=10)
        self.lesson_progress_pub = rospy.Publisher('/lesson/progress', Float32, queue_size=10)
        self.speech_pub = rospy.Publisher('/robot/speak', String, queue_size=10)
        self.motion_pub = rospy.Publisher('/motion_command', String, queue_size=10)
        
        # Lesson content templates
        self.lesson_content = {
            'introduction': {
                'type': 'introduction',
                'text': 'Hello! Welcome to our lesson today.',
                'motion': 'greeting'
            },
            'explanation': {
                'type': 'explanation',
                'text': 'Let me explain this concept to you.',
                'motion': 'explain'
            },
            'question': {
                'type': 'question',
                'text': 'What do you think about this?',
                'motion': 'think'
            },
            'success': {
                'type': 'success',
                'text': 'Excellent! You got it right!',
                'motion': 'wave'
            },
            'error': {
                'type': 'error',
                'text': 'That\'s not quite right. Let me help you.',
                'motion': 'think'
            },
            'challenge': {
                'type': 'challenge',
                'text': 'This is a challenging problem. Let\'s work through it together.',
                'motion': 'think'
            },
            'celebration': {
                'type': 'celebration',
                'text': 'Congratulations! You completed the lesson!',
                'motion': 'wave'
            }
        }
        
        # Lesson sequence
        self.lesson_sequence = [
            'introduction',
            'explanation',
            'question',
            'success',
            'explanation',
            'challenge',
            'error',
            'explanation',
            'success',
            'celebration'
        ]
        
        self.current_step = 0
        self.progress = 0.0
        
        # Start simulation
        rospy.Timer(rospy.Duration(5.0), self.simulate_lesson_step)
        
        rospy.loginfo("Lesson Simulator started")
    
    def simulate_lesson_step(self, event):
        """Simulate one step of the lesson"""
        if self.current_step >= len(self.lesson_sequence):
            # Reset lesson
            self.current_step = 0
            self.progress = 0.0
            rospy.loginfo("Restarting lesson simulation")
        
        # Get current lesson type
        lesson_type = self.lesson_sequence[self.current_step]
        content = self.lesson_content[lesson_type]
        
        # Update progress
        self.progress = (self.current_step + 1) / len(self.lesson_sequence)
        
        # Publish lesson state
        rospy.loginfo(f"Lesson step {self.current_step + 1}: {lesson_type}")
        self.lesson_state_pub.publish(lesson_type)
        
        # Publish lesson content
        content_msg = json.dumps(content)
        self.lesson_content_pub.publish(content_msg)
        
        # Publish progress
        self.lesson_progress_pub.publish(self.progress)
        
        # Publish speech after a short delay
        rospy.Timer(rospy.Duration(1.0), lambda e: self.speech_pub.publish(content['text']), oneshot=True)
        
        # Publish motion after speech
        rospy.Timer(rospy.Duration(3.0), lambda e: self.motion_pub.publish(content['motion']), oneshot=True)
        
        # Move to next step
        self.current_step += 1
    
    def run_interactive_mode(self):
        """Run in interactive mode for manual testing"""
        print("\n=== Lesson Simulator Interactive Mode ===")
        print("Available commands:")
        print("1. introduction")
        print("2. explanation")
        print("3. question")
        print("4. success")
        print("5. error")
        print("6. challenge")
        print("7. celebration")
        print("8. speak <text>")
        print("9. motion <motion_type>")
        print("10. progress <0.0-1.0>")
        print("11. quit")
        
        while not rospy.is_shutdown():
            try:
                command = input("\nEnter command: ").strip()
                
                if command == 'quit':
                    break
                elif command in self.lesson_content:
                    content = self.lesson_content[command]
                    self.lesson_state_pub.publish(command)
                    self.lesson_content_pub.publish(json.dumps(content))
                    self.speech_pub.publish(content['text'])
                    rospy.sleep(1.0)
                    self.motion_pub.publish(content['motion'])
                elif command.startswith('speak '):
                    text = command[6:]
                    self.speech_pub.publish(text)
                elif command.startswith('motion '):
                    motion = command[7:]
                    self.motion_pub.publish(motion)
                elif command.startswith('progress '):
                    try:
                        progress = float(command[9:])
                        self.lesson_progress_pub.publish(progress)
                    except ValueError:
                        print("Invalid progress value")
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Error in interactive mode: {e}")

def main():
    try:
        simulator = LessonSimulator()
        
        # Check if interactive mode is requested
        if len(rospy.myargv()) > 1 and rospy.myargv()[1] == '--interactive':
            simulator.run_interactive_mode()
        else:
            rospy.spin()
            
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")

if __name__ == '__main__':
    main() 