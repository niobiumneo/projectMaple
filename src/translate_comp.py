#!/usr/bin/env python3
"""
translate_comp.py
=================
Child / adult translation companion with CLI-selectable STT backend.

Roles
-----
- a = child speaks
- b = adult speaks
- q = quit

STT backends
------------
- faster_whisper
- cohere_hf

Behavior
--------
- If STT backend is faster_whisper:
    * child language is auto-detected on child turns
    * no child-language argument is required
- If STT backend is cohere_hf:
    * child language is required or prompted once at startup
- Ollama is used only when a real translation is needed
- Adult to child TTS uses edge-playback directly
- Child to adult has no TTS

Examples
--------
python translate_comp.py --stt-backend faster_whisper
python translate_comp.py --stt-backend cohere_hf --child-language fr
python translate_comp.py --stt-backend faster_whisper --tts
"""

import argparse
import json
import logging
import os
import select
import shutil
import subprocess
import sys
import termios
import tty
from contextlib import contextmanager

import numpy as np
import requests
import sounddevice as sd

logging.basicConfig(level=logging.WARNING)

LANG_NAMES = {
    "en": "English",
    "fr": "French",
    "es": "Spanish",
    "de": "German",
    "zh": "Chinese",
    "ja": "Japanese",
    "ar": "Arabic",
    "ko": "Korean",
    "pt": "Portuguese",
    "it": "Italian",
    "el": "Greek",
    "nl": "Dutch",
    "pl": "Polish",
    "vi": "Vietnamese",
}

DEFAULT_VOICE_BY_LANG = {
    "en": "en-US-AriaNeural",
    "fr": "fr-FR-DeniseNeural",
    "es": "es-ES-ElviraNeural",
    "de": "de-DE-KatjaNeural",
    "zh": "zh-CN-XiaoxiaoNeural",
    "ja": "ja-JP-NanamiNeural",
    "ar": "ar-SA-ZariyahNeural",
    "ko": "ko-KR-SunHiNeural",
    "pt": "pt-BR-FranciscaNeural",
    "it": "it-IT-IsabellaNeural",
    "el": "el-GR-AthinaNeural",
    "nl": "nl-NL-ColetteNeural",
    "pl": "pl-PL-ZofiaNeural",
    "vi": "vi-VN-HoaiMyNeural",
}

OLLAMA_READY = False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Child/adult translator with toggleable STT backend."
    )

    parser.add_argument(
        "--stt-backend",
        choices=["faster_whisper", "cohere_hf"],
        default="faster_whisper",
        help="Speech-to-text backend to use.",
    )

    parser.add_argument(
        "--child-language",
        choices=sorted(LANG_NAMES.keys()),
        default=None,
        help="Child language code. Required only for cohere_hf.",
    )

    # Faster Whisper options
    parser.add_argument("--whisper-model", default="small")
    parser.add_argument("--whisper-device", default="cpu")
    parser.add_argument("--whisper-compute", default="int8")

    # Cohere HF options
    parser.add_argument(
        "--cohere-model",
        default="CohereLabs/cohere-transcribe-03-2026",
    )

    # Ollama options
    parser.add_argument(
        "--ollama-url",
        default="http://localhost:11434/api/chat",
    )
    parser.add_argument(
        "--ollama-model",
        default=os.getenv("OLLAMA_MODEL", "aya-expanse:8b-q3_K_S"),
    )

    # Audio options
    parser.add_argument("--mic-sample-rate", type=int, default=16000)
    parser.add_argument("--mic-chunk-secs", type=float, default=0.4)
    parser.add_argument("--mic-device", default=None)

    # TTS options
    parser.add_argument(
        "--tts",
        action="store_true",
        help="Enable text-to-speech for adult to child output.",
    )
    parser.add_argument(
        "--child-english-voice",
        default=os.getenv("CHILD_ENGLISH_VOICE_OVERRIDE", "en-US-AriaNeural"),
        help="English voice for adult to child English playback.",
    )

    return parser.parse_args()


def normalize_text(value) -> str:
    if value is None:
        return ""

    if isinstance(value, str):
        return value.strip()

    if isinstance(value, (list, tuple)):
        parts = []
        for item in value:
            text = normalize_text(item)
            if text:
                parts.append(text)
        return " ".join(parts).strip()

    return str(value).strip()


@contextmanager
def cbreak_mode(fileobj):
    fd = fileobj.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key_nonblocking() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def get_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def resolve_mic_device(raw_device):
    if raw_device is None:
        return None
    if isinstance(raw_device, str) and raw_device.isdigit():
        return int(raw_device)
    return raw_device


def record_utterance_until_enter(args: argparse.Namespace) -> np.ndarray | None:
    chunk = int(args.mic_sample_rate * args.mic_chunk_secs)
    buffers = []

    print("  Speak now. Press Enter to end the turn.", flush=True)

    with cbreak_mode(sys.stdin):
        with sd.InputStream(
            samplerate=args.mic_sample_rate,
            channels=1,
            dtype="float32",
            blocksize=chunk,
            device=resolve_mic_device(args.mic_device),
        ) as stream:
            while True:
                data, _ = stream.read(chunk)
                buffers.append(data[:, 0].copy())

                key = read_key_nonblocking()
                if key in ("\n", "\r"):
                    break

    if not buffers:
        return None

    audio = np.concatenate(buffers).astype(np.float32, copy=False)

    if audio.size == 0:
        return None
    if float(np.max(np.abs(audio))) < 1e-5:
        return None

    return audio


def load_stt(args: argparse.Namespace):
    if args.stt_backend == "faster_whisper":
        try:
            from faster_whisper import WhisperModel
        except ImportError as exc:
            raise RuntimeError(
                "faster-whisper is not installed.\n"
                "Run:\n"
                "  pip install faster-whisper"
            ) from exc

        print(
            f"Loading Faster Whisper '{args.whisper_model}' "
            f"on {args.whisper_device} ({args.whisper_compute})...",
            end=" ",
            flush=True,
        )

        model = WhisperModel(
            args.whisper_model,
            device=args.whisper_device,
            compute_type=args.whisper_compute,
        )

        print("ready.")
        return {
            "backend": "faster_whisper",
            "model": model,
        }

    if args.stt_backend == "cohere_hf":
        try:
            import torch
            from transformers import AutoProcessor, CohereAsrForConditionalGeneration
        except ImportError as exc:
            raise RuntimeError(
                "transformers / torch are not installed.\n"
                "Run:\n"
                "  pip install transformers torch huggingface_hub"
            ) from exc

        print(f"Loading HF model '{args.cohere_model}'...", end=" ", flush=True)

        processor = AutoProcessor.from_pretrained(args.cohere_model)

        if torch.cuda.is_available():
            device = "cuda"
            dtype = torch.float16
        else:
            device = "cpu"
            dtype = torch.float32

        model = CohereAsrForConditionalGeneration.from_pretrained(
            args.cohere_model,
            torch_dtype=dtype,
        ).to(device)

        print(f"ready on {device}.")
        return {
            "backend": "cohere_hf",
            "processor": processor,
            "model": model,
            "torch": torch,
        }

    raise RuntimeError(f"Unsupported STT backend: {args.stt_backend}")


def transcribe(stt, audio: np.ndarray, force_language: str | None, args: argparse.Namespace) -> tuple[str, str]:
    if stt["backend"] == "faster_whisper":
        return _transcribe_faster_whisper(stt["model"], audio, force_language)

    if stt["backend"] == "cohere_hf":
        return _transcribe_cohere_hf(
            processor=stt["processor"],
            model=stt["model"],
            torch_mod=stt["torch"],
            audio=audio,
            force_language=force_language,
            args=args,
        )

    raise RuntimeError(f"Unknown STT backend: {stt['backend']}")


def _transcribe_faster_whisper(model, audio: np.ndarray, force_language: str | None) -> tuple[str, str]:
    segments, info = model.transcribe(
        audio,
        language=force_language,
        beam_size=5,
        vad_filter=True,
        vad_parameters={"min_silence_duration_ms": 300},
    )
    text = " ".join(seg.text.strip() for seg in segments).strip()
    lang = normalize_text(info.language or force_language or "en").lower()
    return text, lang


def _transcribe_cohere_hf(processor, model, torch_mod, audio: np.ndarray, force_language: str | None, args: argparse.Namespace) -> tuple[str, str]:
    lang = normalize_text(force_language or "en").lower()

    with torch_mod.inference_mode():
        inputs = processor(
            audio,
            sampling_rate=args.mic_sample_rate,
            return_tensors="pt",
            language=lang,
        )

        audio_chunk_index = None
        if "audio_chunk_index" in inputs:
            audio_chunk_index = inputs["audio_chunk_index"]
            del inputs["audio_chunk_index"]

        moved_inputs = {}
        for key, value in inputs.items():
            if isinstance(value, torch_mod.Tensor):
                if torch_mod.is_floating_point(value):
                    moved_inputs[key] = value.to(device=model.device, dtype=model.dtype)
                else:
                    moved_inputs[key] = value.to(device=model.device)
            else:
                moved_inputs[key] = value

        outputs = model.generate(**moved_inputs, max_new_tokens=256)

        decoded = None

        try:
            if audio_chunk_index is not None:
                decoded = processor.decode(
                    outputs,
                    skip_special_tokens=True,
                    audio_chunk_index=audio_chunk_index,
                    language=lang,
                )
        except Exception:
            decoded = None

        if decoded is None:
            try:
                decoded = processor.batch_decode(outputs, skip_special_tokens=True)
            except Exception:
                try:
                    decoded = processor.decode(outputs[0], skip_special_tokens=True)
                except Exception:
                    decoded = ""

    text = normalize_text(decoded)
    return text, lang


def languages_match(source_lang: str, target_lang: str) -> bool:
    return normalize_text(source_lang).lower() == normalize_text(target_lang).lower()


def ensure_ollama_ready(args: argparse.Namespace):
    global OLLAMA_READY

    if OLLAMA_READY:
        return

    try:
        response = requests.get("http://localhost:11434/api/tags", timeout=3)
        response.raise_for_status()

        models = [m["name"] for m in response.json().get("models", [])]
        base = args.ollama_model.split(":")[0]

        if not any(base in model_name for model_name in models):
            raise RuntimeError(
                f"Model '{args.ollama_model}' not found in Ollama.\n"
                f'Set a different model with:\n  export OLLAMA_MODEL="your_model_name"'
            )

        OLLAMA_READY = True

    except requests.ConnectionError as exc:
        raise RuntimeError(
            "Ollama is not running.\n"
            "Start it with: ollama serve"
        ) from exc
    except Exception as exc:
        raise RuntimeError(f"Failed to query Ollama: {exc}") from exc


def translate(text, source_lang: str, target_lang: str, args: argparse.Namespace) -> str:
    text = normalize_text(text)
    source_lang = normalize_text(source_lang)
    target_lang = normalize_text(target_lang)

    if languages_match(source_lang, target_lang):
        return text

    ensure_ollama_ready(args)

    prompt = (
        f"Translate the following text from {source_lang} to {target_lang}. "
        f"Output only the translation, no explanations, no quotes.\n\n"
        f"{text}"
    )

    payload = {
        "model": args.ollama_model,
        "messages": [{"role": "user", "content": prompt}],
        "stream": True,
        "keep_alive": 0,
        "options": {
            "num_ctx": 512,
            "temperature": 0.1,
        },
    }

    parts = []

    with requests.post(args.ollama_url, json=payload, stream=True, timeout=60) as response:
        if not response.ok:
            try:
                body = response.text
            except Exception:
                body = "<unable to read response body>"

            extra_help = ""
            if "requires more system memory" in body:
                extra_help = (
                    f"\n\nYour current Ollama translation model '{args.ollama_model}' is too large "
                    f"for the memory currently available on this machine.\n"
                    f"Try a smaller model or free memory before translating.\n"
                )

            raise RuntimeError(
                f"Ollama /api/chat failed with status {response.status_code}.\n"
                f"Response body:\n{body}{extra_help}"
            )

        for line in response.iter_lines():
            if not line:
                continue

            chunk = json.loads(line)
            token = chunk.get("message", {}).get("content", "")
            if token:
                parts.append(token)

            if chunk.get("done"):
                break

    return normalize_text("".join(parts))


def check_tts_player(args: argparse.Namespace):
    if not args.tts:
        return

    if shutil.which("edge-playback") is None:
        print(
            "TTS is enabled, but 'edge-playback' was not found.\n"
            "Install it with:\n"
            "  pip install edge-tts\n"
        )

    if shutil.which("mpv") is None:
        print(
            "TTS is enabled, but 'mpv' was not found.\n"
            "On Linux, edge-playback needs mpv.\n"
            "Install it with:\n"
            "  sudo apt install mpv\n"
        )


def resolve_voice(lang_code: str, args: argparse.Namespace, prefer_child_english: bool = False) -> str:
    if prefer_child_english and lang_code == "en":
        return args.child_english_voice
    return DEFAULT_VOICE_BY_LANG.get(lang_code, DEFAULT_VOICE_BY_LANG["en"])


def speak_text(text, lang_code: str, args: argparse.Namespace, prefer_child_english: bool = False) -> None:
    if not args.tts:
        return

    text = normalize_text(text)
    if not text:
        return

    if shutil.which("edge-playback") is None:
        print("TTS skipped: edge-playback not found.")
        return

    if shutil.which("mpv") is None:
        print("TTS skipped: mpv not found.")
        return

    voice = resolve_voice(lang_code, args, prefer_child_english=prefer_child_english)

    try:
        subprocess.run(
            ["edge-playback", "--voice", voice, "--text", text],
            check=True,
        )
    except subprocess.CalledProcessError as exc:
        print(f"TTS playback failed: {exc}")
    except Exception as exc:
        print(f"TTS failed: {exc}")


def prompt_child_language() -> str:
    print("Supported child languages:")
    print("  " + ", ".join(f"{code}={name}" for code, name in LANG_NAMES.items()))
    print()

    while True:
        code = input("Enter child language code: ").strip().lower()
        if code in LANG_NAMES:
            return code
        print("Invalid code. Try again.\n")


def main():
    args = parse_args()

    check_tts_player(args)
    stt = load_stt(args)

    # Faster Whisper auto-detects child language, so we do not ask up front.
    if args.stt_backend == "faster_whisper":
        child_lang_code = None
        child_language = None
    else:
        child_lang_code = args.child_language or prompt_child_language()
        child_language = LANG_NAMES[child_lang_code]

    # print("\n" + "=" * 50)
    # print("Maple Translate | Child and Adult Mode")
    # print("=" * 50)
    # print()
    print(f"STT backend: {args.stt_backend}")
    if child_lang_code:
        print(f"Child language: {child_language} ({child_lang_code})")
    else:
        print("Child language: auto-detect on child turns")
    print(f"TTS enabled: {args.tts}")
    print("a  -> Child speaks")
    print("b  -> Adult speaks in English")
    print("q  -> quit")
    print("During recording, press Enter to end the turn.")
    print()

    while True:
        print("Press a / b / q ... ", end="", flush=True)
        key = get_key()
        print(key)

        if key == "q":
            print("\nGoodbye.")
            break

        elif key == "a":
            audio = record_utterance_until_enter(args)
            if audio is None:
                print("  No usable audio captured.\n")
                continue

            print("  Transcribing...", end=" ", flush=True)

            # Faster Whisper auto-detects on child turns.
            # Cohere HF uses fixed child language.
            child_force_language = None if args.stt_backend == "faster_whisper" else child_lang_code

            text, detected_lang = transcribe(
                stt=stt,
                audio=audio,
                force_language=child_force_language,
                args=args,
            )

            if not text:
                print("(couldn't hear anything)")
                continue

            if args.stt_backend == "faster_whisper":
                child_lang_code = detected_lang or "en"
                child_language = LANG_NAMES.get(child_lang_code, child_lang_code)

            print("done.\n")
            print(f"  Child [{child_language}]: {text}")

            if child_lang_code == "en":
                print("  For Adult [English]: same as above\n")
            else:
                print("  Translating to English...", end=" ", flush=True)
                english = translate(text, child_language, "English", args)
                print(f"\n  For Adult [English]: {english}\n")

        elif key == "b":
            if child_lang_code is None or child_language is None:
                print("\n  Child language is still unknown.")
                print("  Press 'a' first so the script can detect it.\n")
                continue

            audio = record_utterance_until_enter(args)
            if audio is None:
                print("  No usable audio captured.\n")
                continue

            print("  Transcribing...", end=" ", flush=True)
            text, _ = transcribe(
                stt=stt,
                audio=audio,
                force_language="en",
                args=args,
            )

            if not text:
                print("(couldn't hear anything)")
                continue

            print("done.\n")
            print(f"  Adult [English]: {text}")

            if child_lang_code == "en":
                print("  For Child [English]: same as above")
                speak_text(text, "en", args, prefer_child_english=True)
                print()
            else:
                print(f"  Translating to {child_language}...", end=" ", flush=True)
                translated = translate(text, "English", child_language, args)
                print(f"\n  For Child [{child_language}]: {translated}")
                speak_text(translated, child_lang_code, args, prefer_child_english=False)
                print()

        else:
            print("Unknown key. Use a, b, or q.\n")


if __name__ == "__main__":
    main()