# Maple Robot Interaction App

This application allows the Maple robot to interact with users by rendering scenarios that include images, audio, and motion commands sent to ROS. The scenarios are defined in JSON files and can be modified to fit different interactive sessions.

## Prerequisites

Ensure you have the following set up before running the application:

1. **ROS (Robot Operating System)**

   Install and configure ROS (preferably ROS Noetic). Ensure that `roscore` is running before starting any ROS nodes.

2. **ROS Bridge**

   Install and run `rosbridge` to enable WebSocket communication between ROS and the React app. You can install `rosbridge` using the command sudo apt-get install ros-noetic-rosbridge-server. To start `rosbridge`, run the command roslaunch rosbridge_server rosbridge_websocket.launch.

3. **Node.js and npm**

   Install Node.js and npm to run the React application.

## Running the Application

### Step 1: Start roscore

Before running any ROS-dependent components, start the `roscore` by executing the command roscore in your terminal.

### Step 2: Start rosbridge

Run rosbridge to enable communication between the React app and ROS. You can do this by running roslaunch rosbridge_server rosbridge_websocket.launch.

### Step 3: Start the React Application

Navigate to the directory where your React app is located and run the following commands:

npm install
npm start

This will start the development server, and you can access the app in your browser at http://localhost:3000 or replace localhost with the IP of the server if accessing from another device under the same network.

### Step 4: Place Audio and Image Files

- **Audio Files**: Place the audio files in the public/[scenario_name]/audio/ directory. For example, if your scenario is named classroom_interaction, the audio files should be placed in public/classroom_interaction/audio/.

- **Image Files**: Place the image files in the public/[scenario_name]/img/ directory. For example, if your scenario is named classroom_interaction, the image files should be placed in public/classroom_interaction/img/.

### Step 5: Define the Scenario File

The scenario file should be a JSON file and placed in the public/[scenario_name]/ directory.

For an example scenario file structure, refer to scenario_config.txt under the public folder.

- **scenario_name**: The name of the scenario.
- **states**: An array of states that define what should be displayed, played, and sent as a motion command for each state.
- **image**: The filename of the image to be displayed.
- **audio**: The filename of the audio to be played.
- **motion**: The motion command to be sent to the robot, chosen from the names of predefined motions in the motion library.
- **transition**: Defines how the transition to the next state occurs chosen from time and feedback. If choose time, the state transition happens after a set duration; if choose feedback, the state transition happens automatically when the correct answer is selected.
- **options**: An array of options for the question, each of which shows up as a button
- **answer**: The correct answer among the options, which will trigger the state transition is clicked.

### Step 6: Modify the Code to Change the Rendered Scenario

To change the scenario being rendered, modify the line where the scenario file is loaded in the App component:

fetch('/classroom_interaction/scenario_config.json')

Replace classroom_interaction with the name of your desired scenario.
