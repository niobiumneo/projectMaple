#!/usr/bin/env python3
from pylips.speech import RobotFace
from pylips.face import FacePresets

class PyLipsStandalone:
    def __init__(self):
        # Initialize the RobotFace
        self.face = RobotFace(robot_name='Agent', voice_id=None)
        self.face.set_appearance(FacePresets.default)
        print("PyLipsStandalone ready.")

    def speak(self, text):
        """Make the robot speak."""
        print(f"[PyLips] Speaking: {text}")
        self.face.say(text)
        self.face.wait()

    def set_face(self, expression):
        """Set the robot's facial expression."""
        print(f"[PyLips] Setting face: {expression}")
        if expression == 'happy':
            self.face.set_appearance(FacePresets.smile)
        elif expression == 'sad':
            self.face.set_appearance(FacePresets.frown)
        else:
            self.face.set_appearance(FacePresets.default)

    def perform_gesture(self, gesture):
        """Perform a gesture based on a JSON file."""
        try:
            # Load gesture data from JSON file
            with open(gesture_file, "r") as file:
                gesture_data = json.load(file)

            # Execute motor movements
            for motor in gesture_data["motors"]:
                motor_id = motor["id"]
                for goal_position in motor["goal_positions"]:
                    print(f"Moving motor {motor_id} to position {goal_position}")
                    result, error = self.packetHandler.write4ByteTxRx(
                        self.portHandler, motor_id, 116, goal_position
                    )
                    if result != COMM_SUCCESS:
                        print(f"Error moving motor {motor_id}: {self.packetHandler.getTxRxResult(result)}")
                    if error != 0:
                        print(f"Hardware error on motor {motor_id}: {self.packetHandler.getRxPacketError(error)}")
                    time.sleep(0.5)  # Small delay between movements
        except Exception as e:
            print(f"Error performing gesture: {e}")
    
    def run(self):
        """Main loop to interact with the robot."""
        while True:
            print("\nChoose an action:")
            print("1. Speak")
            print("2. Set Face")
            print("3. Perform Gesture")
            print("4. Exit")
            choice = input("Enter your choice: ")

            if choice == '1':
                text = input("Enter text to speak: ")
                self.speak(text)
            elif choice == '2':
                expression = input("Enter face expression (happy, sad, default): ")
                self.set_face(expression)
            elif choice == '3':
                gesture_file = input("Enter the path to the gesture JSON file: ")
                self.perform_gesture(gesture_file)
            elif choice == '4':
                self.shutdown()
                break
            else:
                print("Invalid choice. Please try again.")

if __name__ == '__main__':
    PyLipsStandalone().run()