#!/usr/bin/env python3
import json
import argparse
import rospy
from std_msgs.msg import String

# Requires PyLips to be installed in THIS Python env:
#   pip install pylips
from pylips.speech import RobotFace
from pylips.face import ExpressionPresets

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
        express = a.get("expression")
        tts    = a.get("tts", "")
        sync   = a.get("sync", "speech_then_motion")
        wait   = bool(a.get("wait_speech", True))

        # def do_expression():
        #     if express:
        #         face.ExpressionPresets
        #         rospy.loginfo(f"→ expression: {express}")

        def do_motion():
            if motion:
                motion_pub.publish(String(data=motion))
                rospy.loginfo(f"→ /motion_command: {motion}")

        def do_speech():
            if tts:
                rospy.loginfo(f"→ say: {tts}")
                # face.say(tts, wait=wait)
        
        # def do_express():
        #     if express:
        #         face.express(sad, time=1.0)
        #         rospy.loginfo(f"→ expression: {express}")
        #     else:
        #         rospy.logwarn(f"Unknown expression preset: {express}")

        if sync == "speech_then_motion":
            do_speech(); do_motion()
        elif sync == "motion_then_speech":
            do_motion(); do_speech()
        else:
            do_motion(); do_speech()

    # def on_say(msg: String):
    #     face.say(msg.data, wait=False)
    #     rospy.loginfo(f"→ say: {msg.data}")

    sub = rospy.Subscriber("/maple_action", String, on_action, queue_size=10)
    # tts_sub = rospy.Subscriber("/maple_tts", String, on_say, queue_size=10)
    rospy.loginfo("hello")

    rospy.loginfo(f"Standalone orchestrator up. Sub: /maple_action  Pub: /motion_command  PyLips={args.server_ip}/face/{args.robot_name}")
    rospy.spin()

if __name__ == "__main__":
    main()
