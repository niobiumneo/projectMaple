import sounddevice as sd
import soundfile as sf
import pickle

from pylips.speech import RobotFace
from pylips.speech.system_tts import IPA2VISEME

from allosaurus.app import read_recognizer

# sound recording parameters
duration = 3  # seconds
sd.default.samplerate = 44100
sd.default.channels = 1

# load allosaurus for phoneme recognition
phoneme_model = read_recognizer()

wav_path = 'pylips_phrases/intro1.wav'
out = phoneme_model.recognize(wav_path, timestamp=True, lang_id='eng')

# parse lines, skip empties
lines = [ln.strip() for ln in out.split('\n') if ln.strip()]

# first token is start time, last token is IPA phone
times = [float(ln.split()[0]) for ln in lines]
visemes = [IPA2VISEME.get(ln.split()[-1], 'IDLE') for ln in lines]

# add a small idle tail so the face rests after the last viseme
last_time = times[-1] if times else 0.0
times.append(last_time + 0.2)
visemes.append('IDLE')

# save
with open('pylips_phrases/intro1.pkl', 'wb') as f:
    pickle.dump((times, visemes), f)
