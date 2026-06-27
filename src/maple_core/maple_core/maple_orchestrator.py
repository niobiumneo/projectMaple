#!/usr/bin/env python3
import json
import warnings

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Requires: pip install pylips
from pylips.speech import RobotFace
from pylips.face import ExpressionPresets, FacePresets

warnings.filterwarnings(
    "ignore",
    category=FutureWarning,
    message=r"You are using `torch\.load` with `weights_only=False`.*",
)

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
    "brow_thickness": 16.14892575240744,
}


def _iter_expression_names():
    """List the expression preset names exposed by PyLips, across preset shapes."""
    names = set()
    for n in dir(ExpressionPresets):
        if n.startswith("_"):
            continue
        try:
            val = getattr(ExpressionPresets, n)
        except Exception:
            continue
        if isinstance(val, dict) or callable(val):
            names.add(n)
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
    elif name == "happy":
        return {
            'AU1': 0.5,
            'AU2': 1,
            'AU4': 0,
            'AU5': 0,
            'AU7': 0.5,
            'AU12': 1,
            'AU15': 0,
        }
    elif name == "sad":
        return {
            'AU1': 1,
            'AU2': 0,
            'AU4': 0.5,
            'AU5': 1,
            'AU7': 0,
            'AU12': 0,
            'AU15': 1.5,
        }
    elif name == "default":
        return {
            'AU1': 0,
            'AU2': 0,
            'AU4': 0,
            'AU5': 0,
            'AU7': 0,
            'AU9': 0,
            'AU43': 0,
            'AU10': 0,
            'AU12': 0,
            'AU13': 0,
            'AU14': 0,
            'AU15': 0,
            'AU16': 0,
            'AU17': 0,
            'AU18': 0,
            'AU20': 0,
            'AU23': 0,
            'AU24': 0,
            'AU25': 0,
            'AU26': 0,
            'AU27': 0,
        }
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
    """List the face/appearance preset names exposed by PyLips."""
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


class MapleOrchestrator(Node):
    def __init__(self):
        super().__init__("maple_orchestrator")

        self.declare_parameter("server_ip", "http://127.0.0.1:8000")
        self.declare_parameter("robot_name", "maple")

        self.server_ip = self.get_parameter("server_ip").get_parameter_value().string_value
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        self.face = RobotFace(robot_name=self.robot_name, server_ip=self.server_ip)
        self.face.set_appearance(custom3)
        self.get_logger().info("Changed face to custom appearance")

        expr_names = _iter_expression_names()
        face_names = _iter_face_names()
        self.get_logger().info(
            "Expression presets found: %s" % (", ".join(expr_names) if expr_names else "(none)"))
        self.get_logger().info(
            "Face presets found: %s" % (", ".join(face_names) if face_names else "(none)"))

        self.motion_pub = self.create_publisher(String, "/motion_command", 10)

        self.action_sub = self.create_subscription(
            String, "/maple_action", self.on_action, 10)
        self.expression_sub = self.create_subscription(
            String, "/maple_expression", self.on_expression, 10)
        self.appearance_sub = self.create_subscription(
            String, "/maple_appearance", self.on_appearance, 10)

        self.get_logger().info(
            "Standalone orchestrator up. Sub: /maple_action, /maple_expression, "
            "/maple_appearance  Pub: /motion_command  "
            f"PyLips={self.server_ip}/face/{self.robot_name}")

    def apply_expression(self, name_or_none, duration_ms=1000):
        if not name_or_none:
            return
        preset = _resolve_expression(name_or_none)
        if preset is None:
            self.get_logger().warn(f"Unknown expression preset: {name_or_none}.")
            return
        try:
            duration_ms = int(duration_ms)
        except Exception:
            duration_ms = 1000
        # RobotFace.express expects an AU dict and time in **milliseconds**
        self.face.express(preset, time=duration_ms)
        self.get_logger().info(f"-> expression: {name_or_none} for {duration_ms} ms")

    def apply_appearance(self, val):
        if val is None:
            return
        if isinstance(val, dict):
            self.face.set_appearance(val)
            self.get_logger().info("-> set_appearance from dict")
            return
        s = str(val).strip()
        # JSON dict string?
        if s.startswith("{") and s.endswith("}"):
            try:
                d = json.loads(s)
                self.face.set_appearance(d)
                self.get_logger().info("-> set_appearance from JSON dict")
                return
            except Exception as e:
                self.get_logger().warn(f"Bad appearance JSON: {e}")
                return
        # else try FacePresets (attr or mapping)
        for cand in (s, s.upper(), s.lower(), s.title()):
            try:
                p = getattr(FacePresets, cand)
                if callable(p):
                    p = p()
                if isinstance(p, dict):
                    self.face.set_appearance(p)
                    self.get_logger().info(f"-> FacePreset: {cand}")
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
                            self.face.set_appearance(p)
                            self.get_logger().info(f"-> FacePreset: {k}")
                            return
            except Exception:
                pass
        self.get_logger().warn(f"Unknown FacePreset: {val}")

    def on_action(self, msg: String):
        try:
            a = json.loads(msg.data or "{}")
        except Exception as e:
            self.get_logger().error(f"Bad /maple_action JSON: {e}")
            return

        motion = a.get("motion")
        tts = a.get("tts", "")

        # accept several keys for expression name
        express = (a.get("expression") or a.get("expr") or a.get("emotion")
                   or a.get("face_expression") or a.get("exp"))

        appearance = a.get("appearance")

        try:
            face_ms = int(float(a.get("face_ms", 1000)))
        except Exception:
            face_ms = 1000

        sync = str(a.get("sync", "speech_then_motion")).lower()

        def do_speech():
            if tts:
                self.get_logger().info(f"-> say: {tts}")
                try:
                    # self.face.say(tts, wait=True)  # alternative
                    self.face.stream_file_to_browser(tts)
                except Exception as e:
                    self.get_logger().warn(
                        f"TTS skipped for '{tts}': {e}. "
                        "Generate phrases with pylips or place .wav files in pylips_phrases/."
                    )

        def do_appearance():
            if appearance is not None:
                try:
                    self.apply_appearance(appearance)
                except Exception as e:
                    self.get_logger().warn(f"Appearance skipped: {e}")

        def do_express():
            if express:
                try:
                    self.apply_expression(express, duration_ms=face_ms)
                except Exception as e:
                    self.get_logger().warn(f"Expression skipped: {e}")

        def do_motion():
            if motion:
                self.motion_pub.publish(String(data=motion))
                self.get_logger().info(f"-> /motion_command: {motion}")

        if sync == "speech_then_motion":
            do_speech()
            do_motion()
            do_appearance()
            do_express()
        elif sync == "motion_then_speech":
            do_motion()
            do_speech()
            do_appearance()
            do_express()
        else:
            do_motion()
            do_speech()
            do_appearance()
            do_express()

    def on_expression(self, msg: String):
        try:
            self.apply_expression(msg.data, duration_ms=1000)
        except Exception as e:
            self.get_logger().error(f"/maple_expression error: {e}")

    def on_appearance(self, msg: String):
        try:
            self.apply_appearance(msg.data)
        except Exception as e:
            self.get_logger().error(f"/maple_appearance error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapleOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
