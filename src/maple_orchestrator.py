#!/usr/bin/env python3
import json
import argparse
import rospy
from std_msgs.msg import String

# Requires PyLips to be installed in THIS Python env:
#   pip install pylips
from pylips.speech import RobotFace

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server_ip", default="http://127.0.0.1:8000", help="PyLips server URL")
    ap.add_argument("--robot_name", default="maple", help="Face name used by PyLips")
    args = ap.parse_args()

    rospy.init_node("maple_orchestrator", anonymous=True)

    face = RobotFace(robot_name=args.robot_name, server_ip=args.server_ip)

    motion_pub = rospy.Publisher("/motion_command", String, queue_size=10)

    def on_action(msg: String):
        try:
            a = json.loads(msg.data or "{}")
        except Exception as e:
            rospy.logerr(f"Bad /maple_action JSON: {e}")
            return

        motion = a.get("motion")
        tts    = a.get("tts", "")
        sync   = a.get("sync", "parallel")
        wait   = bool(a.get("wait_speech", False))

        def do_motion():
            if motion:
                motion_pub.publish(String(data=motion))
                rospy.loginfo(f"→ /motion_command: {motion}")

        def do_speech():
            if tts:
                rospy.loginfo(f"→ say: {tts}")
                face.say(tts, wait=wait)

        if sync == "speech_then_motion":
            do_speech(); do_motion()
        elif sync == "motion_then_speech":
            do_motion(); do_speech()
        else:
            do_motion(); do_speech()

    sub = rospy.Subscriber("/maple_action", String, on_action, queue_size=10)
    rospy.loginfo(f"Standalone orchestrator up. Sub: /maple_action  Pub: /motion_command  PyLips={args.server_ip}/face/{args.robot_name}")
    rospy.spin()

if __name__ == "__main__":
    main()
