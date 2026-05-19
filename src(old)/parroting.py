import sounddevice as sd
import pickle
import os

from pylips.speech.system_tts import IPA2VISEME
from allosaurus.app import read_recognizer

# sound recording parameters
sd.default.samplerate = 44100
sd.default.channels = 1

# load allosaurus for phoneme recognition
phoneme_model = read_recognizer()

wav_dir = 'wav_conversion'
output_dir = 'pylips_phrases'

# Ensure the output directory exists
os.makedirs(output_dir, exist_ok=True)

# Iterate through all .wav files in the directory
for wav_file in os.listdir(wav_dir):
    if wav_file.endswith('.wav'):
        wav_path = os.path.join(wav_dir, wav_file)
        out = phoneme_model.recognize(wav_path, timestamp=True, lang_id='eng')

        # Parse lines, skip empties
        lines = [ln.strip() for ln in out.split('\n') if ln.strip()]

        # First token is start time, last token is IPA phone
        times = [float(ln.split()[0]) for ln in lines]
        visemes = [IPA2VISEME.get(ln.split()[-1], 'IDLE') for ln in lines]

        # Add a small idle tail so the face rests after the last viseme
        last_time = times[-1] if times else 0.0
        times.append(last_time + 0.2)
        visemes.append('IDLE')

        # Save to a .pkl file
        pkl_filename = os.path.splitext(wav_file)[0] + '.pkl'
        pkl_path = os.path.join(output_dir, pkl_filename)
        with open(pkl_path, 'wb') as f:
            pickle.dump((times, visemes), f)

        print(f"Converted {wav_file} to {pkl_filename}")
