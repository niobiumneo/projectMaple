#!/usr/bin/env python3
"""

Backends
--------
STT:
- faster_whisper
- cohere_hf

Translation:
- ollama_aya
- hf_tiny_aya

TTS:
- edge-playback (optional, adult -> child only)

Roles
-----
- a = child speaks
- b = adult speaks in English
- q = quit

Examples
--------
python translate_comp.py --stt-backend faster_whisper --translation-backend ollama_aya
python translate_comp.py --stt-backend cohere_hf --child-language fr --translation-backend ollama_aya
python translate_comp.py --stt-backend faster_whisper --translation-backend hf_tiny_aya --tts
python translate_comp.py --stt-backend cohere_hf --child-language es --translation-backend hf_tiny_aya --tts
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
    "hi": "Hindi",
    "ru": "Russian",
    "uk": "Ukrainian",
    "tr": "Turkish",
    "fa": "Persian",
    "he": "Hebrew",
    "id": "Indonesian",
    "ro": "Romanian",
    "cs": "Czech",
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
        description="Child/adult translator with toggleable STT, translation, and optional edge-playback TTS."
    )

    # STT backend
    parser.add_argument(
        "--stt-backend",
        choices=["faster_whisper", "cohere_hf"],
        default="faster_whisper",
        help="Speech-to-text backend.",
    )

    # Child language only required for Cohere HF STT
    parser.add_argument(
        "--child-language",
        choices=sorted(LANG_NAMES.keys()),
        default=None,
        help="Child language code. Required for cohere_hf STT, not needed for faster_whisper.",
    )

    # Faster Whisper options
    parser.add_argument("--whisper-model", default="small")
    parser.add_argument("--whisper-device", default="cpu")
    parser.add_argument("--whisper-compute", default="int8")

    # Cohere HF STT options
    parser.add_argument(
        "--cohere-stt-model",
        default="CohereLabs/cohere-transcribe-03-2026",
    )
    parser.add_argument(
        "--cohere-stt-device",
        choices=["auto", "cpu", "cuda"],
        default="auto",
    )

    # Translation backend
    parser.add_argument(
        "--translation-backend",
        choices=["ollama_aya", "hf_tiny_aya"],
        default="ollama_aya",
        help="Translation backend.",
    )

    # Ollama Aya options
    parser.add_argument(
        "--ollama-url",
        default="http://localhost:11434/api/chat",
    )
    parser.add_argument(
        "--ollama-model",
        default=os.getenv("OLLAMA_MODEL", "aya-expanse:8b-q4_K_S"),
        help="Quantized Aya tag for Ollama.",
    )

    # Tiny Aya HF options
    parser.add_argument(
        "--tiny-aya-model",
        default="CohereLabs/tiny-aya-global",
    )
    parser.add_argument(
        "--tiny-aya-device",
        choices=["auto", "cpu", "cuda"],
        default="auto",
    )

    # Generation
    parser.add_argument("--max-new-tokens", type=int, default=256)
    parser.add_argument("--temperature", type=float, default=0.0)

    # Audio
    parser.add_argument("--mic-sample-rate", type=int, default=16000)
    parser.add_argument("--mic-chunk-secs", type=float, default=0.4)
    parser.add_argument("--mic-device", default=None)

    # Prompt injection
    parser.add_argument(
        "--system-prompt",
        default=None,
        help="Optional custom system prompt for translation.",
    )
    parser.add_argument(
        "--system-prompt-file",
        default=None,
        help="Optional file containing the system prompt.",
    )

    # TTS
    parser.add_argument(
        "--tts",
        action="store_true",
        help="Enable adult-to-child TTS with edge-playback.",
    )
    parser.add_argument(
        "--child-english-voice",
        default=os.getenv("CHILD_ENGLISH_VOICE_OVERRIDE", "en-US-AriaNeural"),
        help="English voice for adult-to-child English playback.",
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


# =====================================================================
# STT loaders
# =====================================================================

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
        return {"backend": "faster_whisper", "model": model}

    if args.stt_backend == "cohere_hf":
        try:
            import torch
            from transformers import AutoProcessor, CohereAsrForConditionalGeneration
        except ImportError as exc:
            raise RuntimeError(
                "transformers / torch are not installed.\n"
                "Run:\n"
                "  pip install transformers torch huggingface_hub sentencepiece protobuf librosa"
            ) from exc

        print(f"Loading Cohere Transcribe '{args.cohere_stt_model}'...", end=" ", flush=True)

        processor = AutoProcessor.from_pretrained(args.cohere_stt_model)

        if args.cohere_stt_device == "auto":
            model = CohereAsrForConditionalGeneration.from_pretrained(
                args.cohere_stt_model,
                device_map="auto",
            )
        else:
            dtype = torch.float16 if args.cohere_stt_device == "cuda" else torch.float32
            model = CohereAsrForConditionalGeneration.from_pretrained(
                args.cohere_stt_model,
                torch_dtype=dtype,
            ).to(args.cohere_stt_device)

        print("ready.")
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
            audio=audio,
            sampling_rate=args.mic_sample_rate,
            return_tensors="pt",
            language=lang,
        )

        audio_chunk_index = inputs.get("audio_chunk_index")
        inputs = inputs.to(model.device, dtype=model.dtype)

        outputs = model.generate(**inputs, max_new_tokens=args.max_new_tokens)

        decoded = processor.decode(
            outputs,
            skip_special_tokens=True,
            audio_chunk_index=audio_chunk_index,
            language=lang,
        )

    text = normalize_text(decoded)
    return text, lang


# =====================================================================
# Prompt injection
# =====================================================================

def load_system_prompt(args: argparse.Namespace) -> str:
    if args.system_prompt_file:
        with open(args.system_prompt_file, "r", encoding="utf-8") as f:
            return f.read().strip()

    if args.system_prompt:
        return args.system_prompt.strip()

    return (
        "You are a careful multilingual translation assistant. "
        "Preserve meaning, tone, and speaker intent naturally. "
        "Output only the translation. Do not explain. Do not add quotes. "
        "Do not add notes, labels, or commentary."
    )


def build_translation_messages(
    text: str,
    source_lang: str,
    target_lang: str,
    system_prompt: str,
) -> list[dict[str, str]]:
    """
    Put your prompt injection here.

    Edit:
    - system_prompt for global behavior
    - user_prompt for task wording
    """
    user_prompt = (
        f"Translate the following text from {source_lang} to {target_lang}.\n"
        f"Return only the translated text.\n\n"
        f"{text}"
    )

    return [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt},
    ]


# =====================================================================
# Translation loaders
# =====================================================================

def load_translation_backend(args: argparse.Namespace):
    if args.translation_backend == "ollama_aya":
        return {"backend": "ollama_aya"}

    if args.translation_backend == "hf_tiny_aya":
        try:
            import torch
            from transformers import AutoTokenizer, AutoModelForCausalLM
        except ImportError as exc:
            raise RuntimeError(
                "transformers / torch are not installed.\n"
                "Run:\n"
                "  pip install transformers torch huggingface_hub"
            ) from exc

        print(f"Loading Tiny Aya '{args.tiny_aya_model}'...", end=" ", flush=True)

        tokenizer = AutoTokenizer.from_pretrained(args.tiny_aya_model)

        if args.tiny_aya_device == "auto":
            model = AutoModelForCausalLM.from_pretrained(
                args.tiny_aya_model,
                torch_dtype="auto",
                device_map="auto",
            )
        else:
            dtype = torch.float16 if args.tiny_aya_device == "cuda" else torch.float32
            model = AutoModelForCausalLM.from_pretrained(
                args.tiny_aya_model,
                torch_dtype=dtype,
            ).to(args.tiny_aya_device)

        model.eval()

        print("ready.")
        return {
            "backend": "hf_tiny_aya",
            "tokenizer": tokenizer,
            "model": model,
            "torch": torch,
        }

    raise RuntimeError(f"Unsupported translation backend: {args.translation_backend}")


# =====================================================================
# Translation dispatch
# =====================================================================

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
                f'Set a different model with:\n  --ollama-model "your_model_tag"'
            )

        OLLAMA_READY = True

    except requests.ConnectionError as exc:
        raise RuntimeError("Ollama is not running.\nStart it with: ollama serve") from exc
    except Exception as exc:
        raise RuntimeError(f"Failed to query Ollama: {exc}") from exc


def translate(
    backend,
    text,
    source_lang: str,
    target_lang: str,
    system_prompt: str,
    args: argparse.Namespace,
) -> str:
    text = normalize_text(text)
    source_lang = normalize_text(source_lang)
    target_lang = normalize_text(target_lang)

    if languages_match(source_lang, target_lang):
        return text

    messages = build_translation_messages(
        text=text,
        source_lang=source_lang,
        target_lang=target_lang,
        system_prompt=system_prompt,
    )

    if backend["backend"] == "ollama_aya":
        return translate_with_ollama(messages, args)

    if backend["backend"] == "hf_tiny_aya":
        return translate_with_tiny_aya(messages, backend, args)

    raise RuntimeError(f"Unknown translation backend: {backend['backend']}")


def translate_with_ollama(messages, args: argparse.Namespace) -> str:
    ensure_ollama_ready(args)

    payload = {
        "model": args.ollama_model,
        "messages": messages,
        "stream": True,
        "keep_alive": 0,
        "options": {
            "num_ctx": 512,
            "temperature": args.temperature,
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
                    f"\n\nYour current Ollama model '{args.ollama_model}' is too large "
                    f"for the memory currently available on this machine.\n"
                    f"Try a smaller quantized tag or free memory.\n"
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


def translate_with_tiny_aya(messages, backend, args: argparse.Namespace) -> str:
    tokenizer = backend["tokenizer"]
    model = backend["model"]
    torch = backend["torch"]

    with torch.inference_mode():
        input_ids = tokenizer.apply_chat_template(
            messages,
            tokenize=True,
            add_generation_prompt=True,
            return_tensors="pt",
        )

        device = next(model.parameters()).device
        input_ids = input_ids.to(device)

        gen_kwargs = {"max_new_tokens": args.max_new_tokens}

        if args.temperature and args.temperature > 0:
            gen_kwargs["do_sample"] = True
            gen_kwargs["temperature"] = args.temperature
            gen_kwargs["top_p"] = 0.95
        else:
            gen_kwargs["do_sample"] = False

        output_ids = model.generate(input_ids, **gen_kwargs)

        new_tokens = output_ids[0][input_ids.shape[-1]:]
        decoded = tokenizer.decode(new_tokens, skip_special_tokens=True)

    return normalize_text(decoded)


# =====================================================================
# edge-playback TTS
# =====================================================================

def check_tts(args: argparse.Namespace):
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


def resolve_tts_voice(lang_code: str, args: argparse.Namespace, prefer_child_english: bool = False) -> str:
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

    voice = resolve_tts_voice(lang_code, args, prefer_child_english=prefer_child_english)

    try:
        subprocess.run(
            ["edge-playback", "--voice", voice, "--text", text],
            check=True,
        )
    except subprocess.CalledProcessError as exc:
        print(f"TTS playback failed: {exc}")
    except Exception as exc:
        print(f"TTS failed: {exc}")


# =====================================================================
# Main
# =====================================================================

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

    system_prompt = load_system_prompt(args)
    stt_backend = load_stt(args)
    translation_backend = load_translation_backend(args)
    check_tts(args)

    if args.stt_backend == "faster_whisper":
        child_lang_code = None
        child_language = None
    else:
        child_lang_code = args.child_language or prompt_child_language()
        child_language = LANG_NAMES[child_lang_code]

    print("\n" + "=" * 50)
    print("Maple Translate | STT + Translation + edge-playback")
    print("=" * 50)
    print()
    print(f"STT backend: {args.stt_backend}")
    print(f"Translation backend: {args.translation_backend}")
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

            child_force_language = None if args.stt_backend == "faster_whisper" else child_lang_code
            text, detected_lang = transcribe(
                stt_backend,
                audio,
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
                english = translate(
                    backend=translation_backend,
                    text=text,
                    source_lang=child_language,
                    target_lang="English",
                    system_prompt=system_prompt,
                    args=args,
                )
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
                stt_backend,
                audio,
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
                translated = translate(
                    backend=translation_backend,
                    text=text,
                    source_lang="English",
                    target_lang=child_language,
                    system_prompt=system_prompt,
                    args=args,
                )
                print(f"\n  For Child [{child_language}]: {translated}")
                speak_text(translated, child_lang_code, args, prefer_child_english=False)
                print()

        else:
            print("Unknown key. Use a, b, or q.\n")


if __name__ == "__main__":
    main()