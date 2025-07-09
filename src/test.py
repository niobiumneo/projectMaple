from pylips.speech import RobotFace
from pylips.face import FacePresets, ExpressionPresets
import time

# 1) (In another terminal) start server:
#    python3 -m pylips.face.start

# # 2) In your code:
face = RobotFace(voice_id = "yue")
face.set_appearance(FacePresets.default)
face.set_appearance(ExpressionPresets.sad)

# 3) Say it and wait
face.say("Hello, welcome to pylips!", wait=True)

# 4) (Optional) keep the process alive a little longer
# face.wait()

face.say("Hope you are doing well!", wait=True)

# face.tts.list_voices()