#!/usr/bin/env python3
import json
import argparse
import rospy
from std_msgs.msg import String

# Requires: pip install pylips
from pylips.speech import RobotFace
from pylips.face import ExpressionPresets, FacePresets

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

# -------- Helpers to deal with various PyLips preset shapes --------
def _iter_expression_names():
    names = set()
    # attributes that look like presets
    for n in dir(ExpressionPresets):
        if n.startswith("_"):
            continue
        try:
            val = getattr(ExpressionPresets, n)
        except Exception:
            continue
        if isinstance(val, dict) or callable(val):
            names.add(n)
    # mapping-style keys (if provided)
    keys = getattr(ExpressionPresets, "keys", None)
    if callable(keys):
        try:
            for k in ExpressionPresets.keys():
                names.add(str(k))
        except Exception:
            pass
    return sorted(names)

def _resolve_expression(name):
    """Return an AU->intensity dict for the given name, or None."""
    if not name:
        return None
    s = str(name).strip()
    # Try attribute names in multiple casings
    for cand in (s, s.upper(), s.lower(), s.title()):
        try:
            val = getattr(ExpressionPresets, cand)
            if callable(val):
                val = val()  # some presets may be factories
            if isinstance(val, dict):
                return val
        except Exception:
            pass
    # Try mapping-style lookup
    getitem = getattr(ExpressionPresets, "__getitem__", None)
    keys = getattr(ExpressionPresets, "keys", None)
    if callable(keys) and callable(getitem):
        try:
            # case-insensitive match across keys
            for k in ExpressionPresets.keys():
                if str(k).lower() == s.lower():
                    val = ExpressionPresets[k]
                    if callable(val):
                        val = val()
                    if isinstance(val, dict):
                        return val
        except Exception:
            pass
    return None

def _iter_face_names():
    names = set()
    for n in dir(FacePresets):
        if n.startswith("_"):
            continue
        try:
            val = getattr(FacePresets, n)
        except Exception:
            continue
        if isinstance(val, dict) or callable(val):
            names.add(n)
    keys = getattr(FacePresets, "keys", None)
    if callable(keys):
        try:
            for k in FacePresets.keys():
                names.add(str(k))
        except Exception:
            pass
    return sorted(names)

def apply_expression(face, name_or_none, duration_ms=1000):
    if not name_or_none:
        return
    preset = _resolve_expression(name_or_none)
    if preset is None:
        rospy.logwarn(f"Unknown expression preset: {name_or_none}.")
        return
    try:
        duration_ms = int(duration_ms)
    except Exception:
        duration_ms = 1000
    # RobotFace.express expects an AU dict and time in **milliseconds**
    face.express(preset, time=duration_ms)
    rospy.loginfo(f"→ expression: {name_or_none} for {duration_ms} ms")

def apply_appearance(face, val):
    if val is None:
        return
    if isinstance(val, dict):
        face.set_appearance(val)
        rospy.loginfo("→ set_appearance from dict")
        return
    s = str(val).strip()
    # JSON dict string?
    if s.startswith("{") and s.endswith("}"):
        try:
            d = json.loads(s)
            face.set_appearance(d)
            rospy.loginfo("→ set_appearance from JSON dict")
            return
        except Exception as e:
            rospy.logwarn(f"Bad appearance JSON: {e}")
            return
    # else try FacePresets (attr or mapping)
    for cand in (s, s.upper(), s.lower(), s.title()):
        try:
            p = getattr(FacePresets, cand)
            if callable(p):
                p = p()
            if isinstance(p, dict):
                face.set_appearance(p)
                rospy.loginfo(f"→ FacePreset: {cand}")
                return
        except Exception:
            pass
    keys = getattr(FacePresets, "keys", None)
    if callable(keys):
        try:
            for k in FacePresets.keys():
                if str(k).lower() == s.lower():
                    p = FacePresets[k]
                    if callable(p):
                        p = p()
                    if isinstance(p, dict):
                        face.set_appearance(p)
                        rospy.loginfo(f"→ FacePreset: {k}")
                        return
        except Exception:
            pass
    rospy.logwarn(f"Unknown FacePreset: {val}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server_ip", default="http://127.0.0.1:8000", help="PyLips server URL")
    ap.add_argument("--robot_name", default="maple", help="Face name used by PyLips")
    args = ap.parse_args()

    rospy.init_node("maple_orchestrator", anonymous=True)

    face = RobotFace(robot_name=args.robot_name, server_ip=args.server_ip)
    face.set_appearance(custom3)
    rospy.loginfo("Changed face to custom appearance")

    # Log available preset names (robust listing)
    expr_names = _iter_expression_names()
    face_names = _iter_face_names()
    rospy.loginfo("Expression presets found: %s", ", ".join(expr_names) if expr_names else "(none)")
    rospy.loginfo("Face presets found: %s", ", ".join(face_names) if face_names else "(none)")

    motion_pub = rospy.Publisher("/motion_command", String, queue_size=10)

    def on_action(msg: String):
        try:
            a = json.loads(msg.data or "{}")
        except Exception as e:
            rospy.logerr(f"Bad /maple_action JSON: {e}")
            return

        motion = a.get("motion")
        tts    = a.get("tts", "")

        # accept several keys for expression name
        express = (a.get("expression") or a.get("expr") or a.get("emotion")
                   or a.get("face_expression") or a.get("exp"))

        appearance = a.get("appearance")

        # # allow 'face_app' to map to either expression or appearance:
        # face_app = a.get("face_app")
        # if face_app is not None:
        #     # decide automatically based on which space it matches
        #     target = str(face_app).strip()
        #     if _resolve_expression(target) is not None and not express:
        #         express = target
        #     elif target and target not in (None, "") and appearance is None:
        #         appearance = target

        try:
            face_ms = int(float(a.get("face_ms", 1000)))
        except Exception:
            face_ms = 1000

        sync = str(a.get("sync", "speech_then_motion")).lower()
        wait = bool(a.get("wait_speech", True))

        def do_motion():
            if motion:
                motion_pub.publish(String(data=motion))
                rospy.loginfo(f"→ /motion_command: {motion}")

        def do_speech():
            if tts:
                rospy.loginfo(f"→ say: {tts}")
                # face.say(tts, wait=wait)  # alternative
                face.stream_file_to_browser(tts)

        def do_appearance():
            if appearance is not None:
                apply_appearance(face, appearance)

        def do_express():
            if express:
                apply_expression(face, express, duration_ms=face_ms)

        if sync == "speech_then_motion":
            do_speech(); do_motion(); do_appearance(); do_express()
        elif sync == "motion_then_speech":
            do_motion(); do_speech(); do_appearance(); do_express()
        else:
            do_motion(); do_speech(); do_appearance(); do_express()

    rospy.Subscriber("/maple_action", String, on_action, queue_size=10)

    # Simple topics (no JSON needed)
    def on_expression(msg: String):
        try:
            apply_expression(face, msg.data, duration_ms=1000)
        except Exception as e:
            rospy.logerr(f"/maple_expression error: {e}")

    def on_appearance(msg: String):
        try:
            apply_appearance(face, msg.data)
        except Exception as e:
            rospy.logerr(f"/maple_appearance error: {e}")

    rospy.Subscriber("/maple_expression", String, on_expression, queue_size=10)
    rospy.Subscriber("/maple_appearance", String, on_appearance, queue_size=10)

    rospy.loginfo(
        f"Standalone orchestrator up. Sub: /maple_action, /maple_expression, /maple_appearance  "
        f"Pub: /motion_command  PyLips={args.server_ip}/face/{args.robot_name}"
    )
    rospy.spin()

if __name__ == "__main__":
    main()
