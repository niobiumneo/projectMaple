#!/usr/bin/env python3
"""
Robot Face Bridge - ROS Node for integrating pylips facial expressions
with robot motor states and lesson content.

This node provides:
1. Dynamic facial expressions based on lesson content
2. Face changes synchronized with motor movements
3. Emotional responses to different lesson states
4. Integration with existing motion system
"""

import rospy
import json
import threading
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from pylips.speech import RobotFace
from pylips.face import FacePresets
import numpy as np

class RobotFaceBridge:
    def __init__(self):
        rospy.init_node('robot_face_bridge', anonymous=True)
        
        # Initialize pylips face
        self.face = RobotFace(robot_name='Agent', voice_id=None)
        self.face.set_appearance(FacePresets.default)
        
        # Current state tracking
        self.current_lesson_state = "idle"
        self.current_emotion = "neutral"
        self.is_speaking = False
        self.is_moving = False
        self.lesson_progress = 0.0
        self.motor_states = {}
        
        # Face expression mapping
        self.face_expressions = {
            'happy': FacePresets.smile,
            'sad': FacePresets.frown,
            'surprised': FacePresets.surprised,
            'thinking': FacePresets.thinking,
            'excited': FacePresets.excited,
            'confused': FacePresets.confused,
            'neutral': FacePresets.default,
            'explaining': FacePresets.attentive,
            'listening': FacePresets.attentive,
            'success': FacePresets.smile,
            'error': FacePresets.frown,
            'waiting': FacePresets.default
        }
        
        # Lesson content to emotion mapping
        self.lesson_emotions = {
            'introduction': 'excited',
            'explanation': 'explaining',
            'question': 'thinking',
            'answer': 'listening',
            'success': 'success',
            'error': 'confused',
            'transition': 'neutral',
            'break': 'happy',
            'challenge': 'thinking',
            'celebration': 'excited'
        }
        
        # Motor movement to emotion mapping
        self.motor_emotions = {
            'explain': 'explaining',
            'greeting': 'happy',
            'wave': 'happy',
            'think': 'thinking',
            'sad': 'sad',
            'narrate': 'attentive',
            'turnleft': 'thinking',
            'turnright': 'thinking',
            'lookstraight': 'attentive',
            'headnode': 'attentive'
        }
        
        # Setup ROS publishers and subscribers
        self.setup_ros_communication()
        
        # Start face update thread
        self.face_thread = threading.Thread(target=self.face_update_loop, daemon=True)
        self.face_thread.start()
        
        rospy.loginfo("Robot Face Bridge initialized successfully")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        
        # Publishers
        self.face_state_pub = rospy.Publisher('/robot/face/state', String, queue_size=10)
        self.emotion_pub = rospy.Publisher('/robot/emotion', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/motion_command', String, self.motion_command_callback)
        rospy.Subscriber('/lesson/content', String, self.lesson_content_callback)
        rospy.Subscriber('/lesson/state', String, self.lesson_state_callback)
        rospy.Subscriber('/lesson/progress', Float32, self.lesson_progress_callback)
        rospy.Subscriber('/robot/speak', String, self.speak_callback)
        rospy.Subscriber('/robot/emotion/request', String, self.emotion_request_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Services (if needed)
        # self.set_emotion_service = rospy.Service('/robot/set_emotion', SetEmotion, self.set_emotion_callback)
    
    def motion_command_callback(self, msg):
        """Handle motion commands and update face accordingly"""
        motion_type = msg.data
        rospy.loginfo(f"Motion command received: {motion_type}")
        
        # Set emotion based on motion type
        if motion_type in self.motor_emotions:
            emotion = self.motor_emotions[motion_type]
            self.set_emotion(emotion)
            self.is_moving = True
            
            # Reset moving flag after a delay
            rospy.Timer(rospy.Duration(3.0), self.reset_moving_flag, oneshot=True)
    
    def lesson_content_callback(self, msg):
        """Handle lesson content updates"""
        content_data = json.loads(msg.data)
        lesson_type = content_data.get('type', 'explanation')
        
        # Map lesson type to emotion
        if lesson_type in self.lesson_emotions:
            emotion = self.lesson_emotions[lesson_type]
            self.set_emotion(emotion)
        
        # Update lesson state
        self.current_lesson_state = lesson_type
    
    def lesson_state_callback(self, msg):
        """Handle lesson state changes"""
        state = msg.data
        rospy.loginfo(f"Lesson state changed to: {state}")
        
        # Map state to emotion
        if state in self.lesson_emotions:
            emotion = self.lesson_emotions[state]
            self.set_emotion(emotion)
        
        self.current_lesson_state = state
    
    def lesson_progress_callback(self, msg):
        """Handle lesson progress updates"""
        progress = msg.data
        self.lesson_progress = progress
        
        # Adjust emotion based on progress
        if progress >= 0.9:
            self.set_emotion('success')
        elif progress >= 0.7:
            self.set_emotion('excited')
        elif progress <= 0.2:
            self.set_emotion('thinking')
    
    def speak_callback(self, msg):
        """Handle speech requests"""
        text = msg.data
        rospy.loginfo(f"Speaking: {text}")
        
        # Set speaking flag
        self.is_speaking = True
        
        # Analyze text sentiment for emotion
        emotion = self.analyze_text_sentiment(text)
        if emotion:
            self.set_emotion(emotion)
        
        # Speak the text
        self.face.say(text)
        self.face.wait()
        
        # Reset speaking flag
        self.is_speaking = False
    
    def emotion_request_callback(self, msg):
        """Handle direct emotion requests"""
        emotion = msg.data
        self.set_emotion(emotion)
    
    def joint_states_callback(self, msg):
        """Monitor joint states for movement detection"""
        # Store current motor states
        for i, name in enumerate(msg.name):
            if name.startswith('motor_'):
                motor_id = int(name.split('_')[1])
                self.motor_states[motor_id] = msg.position[i]
    
    def analyze_text_sentiment(self, text):
        """Simple sentiment analysis for text"""
        text_lower = text.lower()
        
        # Positive words
        positive_words = ['great', 'excellent', 'amazing', 'wonderful', 'good', 'correct', 'success', 'perfect']
        # Negative words
        negative_words = ['wrong', 'error', 'mistake', 'sorry', 'bad', 'incorrect', 'failed']
        # Question words
        question_words = ['what', 'how', 'why', 'when', 'where', 'which', '?']
        
        if any(word in text_lower for word in positive_words):
            return 'success'
        elif any(word in text_lower for word in negative_words):
            return 'confused'
        elif any(word in text_lower for word in question_words):
            return 'thinking'
        
        return None
    
    def set_emotion(self, emotion):
        """Set the robot's facial emotion"""
        if emotion in self.face_expressions:
            rospy.loginfo(f"Setting emotion to: {emotion}")
            self.current_emotion = emotion
            self.face.set_appearance(self.face_expressions[emotion])
            
            # Publish current state
            self.face_state_pub.publish(emotion)
            self.emotion_pub.publish(emotion)
        else:
            rospy.logwarn(f"Unknown emotion: {emotion}")
    
    def reset_moving_flag(self, event):
        """Reset the moving flag after motion completion"""
        self.is_moving = False
    
    def face_update_loop(self):
        """Main loop for updating face based on current state"""
        rate = rospy.Rate(2)  # 2 Hz update rate
        
        while not rospy.is_shutdown():
            try:
                # Determine appropriate emotion based on current state
                target_emotion = self.determine_target_emotion()
                
                # Update face if emotion changed
                if target_emotion != self.current_emotion:
                    self.set_emotion(target_emotion)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in face update loop: {e}")
                rate.sleep()
    
    def determine_target_emotion(self):
        """Determine the appropriate emotion based on current state"""
        
        # Priority order: speaking > moving > lesson state > default
        
        if self.is_speaking:
            if self.current_lesson_state == 'explanation':
                return 'explaining'
            elif self.current_lesson_state == 'question':
                return 'thinking'
            else:
                return 'attentive'
        
        elif self.is_moving:
            return self.current_emotion  # Keep current emotion during movement
        
        elif self.current_lesson_state in self.lesson_emotions:
            return self.lesson_emotions[self.current_lesson_state]
        
        else:
            return 'neutral'
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down Robot Face Bridge")
        self.face.set_appearance(FacePresets.default)

def main():
    try:
        bridge = RobotFaceBridge()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
    finally:
        if 'bridge' in locals():
            bridge.shutdown()

if __name__ == '__main__':
    main() 