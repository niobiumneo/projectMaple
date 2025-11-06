#!/usr/bin/env python3
import json
import argparse
import rospy
from std_msgs.msg import String

# Requires PyLips to be installed in THIS Python env:
#   pip install pylips
from pylips.speech import RobotFace
from pylips.face import ExpressionPresets, FacePresets
custom2 = {
            "background_color": "#D7E4F5",
            "eyeball_color": "#fafafa",
            "iris_color": "#611313",
            "eye_size": 113.98375676801332,
            "eye_height": 67.27405247813411,
            "eye_separation": 485.506039150354,
            "iris_size": 58.87130362349021,
            "pupil_scale": 0.5522490628904623,
            "eye_shine": True,
            "nose_color": "#7a5e6c",
            "nose_vertical_position": -10.561299852289515,
            "nose_width": 0,
            "nose_height": 0.9495674192867694,
            "mouth_color": "#2c241b",
            "mouth_thickness": 13.214285714285715,
            "mouth_width": 301.27030403998333,
            "mouth_height": 25.837151187005414,
            "mouth_y": 150,
            "brow_color": "#2c241b",
            "brow_width": 130,
            "brow_height": 210,
            "brow_thickness": 18
        }

custom3 = {
  "background_color": "#d7e4f5",
  "eyeball_color": "#ffffff",
  "iris_color": "#72001b",
  "eye_size": 132.28766144621198,
  "eye_height": 80,
  "eye_separation": 400,
  "iris_size": 69.65244628812871,
  "pupil_scale": 0.6001223177106711,
  "eye_shine": True,
  "nose_color": "#ff99cc",
  "nose_vertical_position": 10,
  "nose_width": 0,
  "nose_height": 0,
  "mouth_color": "#2c241b",
  "mouth_thickness": 18,
  "mouth_width": 307.8845969864763,
  "mouth_height": 22.233218830562976,
  "mouth_y": 146.3187411859521,
  "brow_color": "#2c241b",
  "brow_width": 120.3609373902295,
  "brow_height": 245.1471762360204,
  "brow_thickness": 16.14892575240744
}
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server_ip", default="http://127.0.0.1:8000", help="PyLips server URL")
    ap.add_argument("--robot_name", default="maple", help="Face name used by PyLips")
    args = ap.parse_args()

    rospy.init_node("maple_orchestrator", anonymous=True)

    face = RobotFace(robot_name=args.robot_name, server_ip=args.server_ip)
    face.set_appearance(custom3)
    rospy.loginfo(f"Changed face to cutom appearance")

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
                face.stream_file_to_browser(tts)

        
        def do_express():
            if express:
                face.express(getattr(ExpressionPresets, express, None), time=1.0)
                rospy.loginfo(f"→ expression: {express}")
            else:
                rospy.logwarn(f"Unknown expression preset: {express}")

        if sync == "speech_then_motion":
            do_speech(); do_motion(); do_express()
        elif sync == "motion_then_speech":
            do_motion(); do_speech(); do_express()
        else:
            do_motion(); do_speech(); do_express()

    # def on_say(msg: String):
    #     face.say(msg.data, wait=False)
    #     rospy.loginfo(f"→ say: {msg.data}")

    sub = rospy.Subscriber("/maple_action", String, on_action, queue_size=10)
    # tts_sub = rospy.Subscriber("/maple_tts", String, on_say, queue_size=10)
    # rospy.loginfo("hello")

    rospy.loginfo(f"Standalone orchestrator up. Sub: /maple_action  Pub: /motion_command  PyLips={args.server_ip}/face/{args.robot_name}")
    rospy.spin()

if __name__ == "__main__":
    main()
