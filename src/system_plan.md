# Plan to port this over to ROS2 

### Packages
**maple_core** → handles the orchestration and is the port of the maple_orchestrator.py
(will port this first i think)

**maple_control** → controller nodes  (robot_ctr.py port)

**maple_bringup** → launch files

**maple_ui_bridge** → handles connection to the websocket (unsure about this, if this is best way to do this)

#### List of all Nodes
maple_orchestrator — maple_orchestrator.py (rosnode name: "maple_orchestrator")

robot_move_node — robot_ctr.py (rosnode name: "robot_move_node")

demo robot_move_node — ros_sync_str_motor_demo.py (subscription uses motion_command_topic)
- ill probably port this after the orchestrator
- will put this in the controller package
- will subscribe to /motion_commad and /interaction_control

#### List of all Topics
- /motion_command — published by maple_orchestrator, subscribed by robot_ctr (used to trigger named motions)
    - publishes the motion name and what the robot needs to do (string format)

- /maple_action — published by web UI, subscribed by maple_orchestrator (JSON payload: motion/tts/expression/appearance etc.)

- /maple_expression — published by web UI, subscribed by maple_orchestrator (single-string expression)

- /maple_appearance — published by web UI, subscribed by maple_orchestrator (appearance JSON or preset name)

- /interaction_control — published by web UI, subscribed by robot_ctr (commands: pause, resume, stop)

- motion_command_topic — subscribed by ros_sync_str_motor_demo.py (demo; different topic name) **this will get removed and just replaced with a demo node**


Questions to ask
- can i scrap the whole demo node and just put it as an action in the web ui or should i keep it as its own node 
- 