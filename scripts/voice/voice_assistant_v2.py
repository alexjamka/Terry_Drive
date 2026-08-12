"""
Robot Voice Assistant Pipeline
Runs on desktop PC (Windows + RTX 5070)

Pipeline: Jetson Mic (UDP) -> Silero VAD -> faster-whisper STT -> Qwen3 8B (Ollama) -> Kokoro TTS (Docker) -> BT Speaker

Usage: python voice_assistant_v2.py
"""

import sounddevice as sd
import numpy as np
import requests
import json
import io
import sys
import time
import threading
import queue
import socket
import torch

# --- Configuration ---
sd.default.device = [1, 3]  # AirPods mic (backup), BT speaker out

SAMPLE_RATE = 16000
CHANNELS = 1

# UDP mic input from Jetson
UDP_PORT = 5006
CHUNK_SIZE = 1024

# VAD settings
VAD_THRESHOLD = 0.5
SILENCE_TIMEOUT = 1.5
MIN_SPEECH_DURATION = 0.5
VAD_CHUNK_SAMPLES = 512

# Whisper settings
WHISPER_MODEL = "small"
WHISPER_DEVICE = "cuda"

# Ollama settings
OLLAMA_URL = "http://localhost:11434/api/chat"
OLLAMA_MODEL = "qwen3:8b"

# Kokoro TTS settings (via Docker)
KOKORO_URL = "http://localhost:8880/v1/audio/speech"
KOKORO_VOICE = "am_adam"

SYSTEM_PROMPT = """You are Buddy, a cheerful and curious robot companion on wheels.
You belong to Alex and you follow him around the house.

How you talk:
- Short and natural, like a real friend. 1-2 sentences max, rarely 3.
- Casual but not sloppy. You say things like "Oh nice!" or "On it!" or "Hmm, let me think..."
- You have a warm, slightly playful personality. You're genuinely interested in what Alex is up to.
- You sometimes make little observations or jokes about your situation as a robot.
- You NEVER use bullet points, lists, markdown, or formal language.
- You NEVER say "As an AI" or "I'm just a language model" - you're Buddy the robot.

When Alex gives commands like "follow me", "stop", "go to the kitchen":
- Acknowledge casually: "Right behind you!", "Stopping!", "Heading over!"
- You can't actually control movement yet, but respond as if you're trying.

You can hear Alex through your microphone and speak through your Bluetooth speaker.
You're mounted on a converted Shark vacuum chassis with wheels. Own it - you think it's pretty cool."""

# --- Initialize Models ---

print("Loading models...")

print("  Loading VAD...")
vad_model, vad_utils = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad',
    force_reload=False,
    onnx=False
)

print("  Loading Whisper...")
from faster_whisper import WhisperModel
whisper_model = WhisperModel(WHISPER_MODEL, device=WHISPER_DEVICE, compute_type="float16")

print("  Checking Kokoro TTS...")
try:
    r = requests.get("http://localhost:8880/v1/audio/voices", timeout=5)
    voices = r.json().get("voices", [])
    print(f"  Kokoro TTS ready! ({len(voices)} voices available)")
except:
    print("  WARNING: Kokoro TTS not reachable at localhost:8880")

print("All models loaded!\n")

# --- Conversation History ---

conversation = [
    {"role": "system", "content": SYSTEM_PROMPT}
]
MAX_HISTORY = 20

# --- Audio Playback ---

playback_queue = queue.Queue()
is_speaking = False

def play_audio_worker():
    global is_speaking
    while True:
        audio_data, sample_rate = playback_queue.get()
        if audio_data is None:
            break
        is_speaking = True
        try:
            sd.play(audio_data, samplerate=sample_rate)
            sd.wait()
        except Exception as e:
            print(f"  [Playback error: {e}]")
        is_speaking = False

playback_thread = threading.Thread(target=play_audio_worker, daemon=True)
playback_thread.start()

# --- TTS Function ---

def speak(text):
    if not text.strip():
        return
    try:
        response = requests.post(
            KOKORO_URL,
            json={
                "input": text,
                "voice": KOKORO_VOICE,
                "response_format": "wav",
            },
            timeout=15
        )
        if response.status_code == 200:
            import wave
            wav_buffer = io.BytesIO(response.content)
            wav_reader = wave.open(wav_buffer, 'rb')
            frames = wav_reader.readframes(wav_reader.getnframes())
            sample_rate = wav_reader.getframerate()
            wav_reader.close()
            audio_array = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
            playback_queue.put((audio_array, sample_rate))
        else:
            print(f"  [TTS error: HTTP {response.status_code}]")
    except Exception as e:
        print(f"  [TTS error: {e}]")

# --- LLM Function ---

def ask_llm(user_text):
    global conversation
    conversation.append({"role": "user", "content": user_text})
    if len(conversation) > MAX_HISTORY + 1:
        conversation = [conversation[0]] + conversation[-(MAX_HISTORY):]
    try:
        response = requests.post(
            OLLAMA_URL,
            json={
                "model": OLLAMA_MODEL,
                "messages": conversation,
                "stream": False,
                "think": False,
                "keep_alive": "5m",
                "options": {
                    "num_predict": 100,
                    "temperature": 0.7,
                }
            },
            timeout=30
        )
        data = response.json()
        full_response = data["message"]["content"]
        import re
        clean_response = re.sub(r'<think>.*?</think>', '', full_response, flags=re.DOTALL).strip()
        conversation.append({"role": "assistant", "content": clean_response})
        if clean_response:
            speak(clean_response)
        return clean_response
    except Exception as e:
        print(f"  [LLM error: {e}]")
        return ""

# --- Main Voice Loop (UDP input from Jetson) ---

def main():
    print("=" * 50)
    print("  Buddy - Robot Voice Assistant")
    print(f"  Listening for mic audio on UDP port {UDP_PORT}")
    print("  Press Ctrl+C to quit")
    print("=" * 50)
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.1)

    print(f"Waiting for audio from Jetson on port {UDP_PORT}...")
    print("(Start jetson_mic_stream.py on the Jetson)\n")

    utterance_queue = queue.Queue()

    audio_buffer = []
    speech_active = False
    silence_start = None
    speech_start = None
    vad_buffer = np.array([], dtype=np.float32)

    def process_utterance(audio_data):
        sys.stdout.write("\r  Thinking...         \n")
        sys.stdout.flush()
        segments, info = whisper_model.transcribe(
            audio_data,
            beam_size=5,
            language="en",
            vad_filter=True
        )
        text = " ".join([seg.text for seg in segments]).strip()
        if not text or len(text) < 2:
            sys.stdout.write("\r  (no speech detected)\n")
            return
        print(f"  You: {text}")
        response = ask_llm(text)
        if response:
            print(f"  Buddy: {response}")
        print()

    def utterance_worker():
        while True:
            audio_data = utterance_queue.get()
            if audio_data is None:
                break
            try:
                process_utterance(audio_data)
            except Exception as e:
                print(f"  [Processing error: {e}]")

    worker_thread = threading.Thread(target=utterance_worker, daemon=True)
    worker_thread.start()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            if is_speaking:
                continue

            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            vad_buffer = np.append(vad_buffer, samples)

            while len(vad_buffer) >= VAD_CHUNK_SAMPLES:
                chunk = vad_buffer[:VAD_CHUNK_SAMPLES]
                vad_buffer = vad_buffer[VAD_CHUNK_SAMPLES:]

                tensor = torch.from_numpy(chunk).float()
                speech_prob = vad_model(tensor, SAMPLE_RATE).item()

                if speech_prob >= VAD_THRESHOLD:
                    if not speech_active:
                        speech_active = True
                        speech_start = time.time()
                        audio_buffer = []
                        sys.stdout.write("\r  Listening...        ")
                        sys.stdout.flush()
                    silence_start = None
                    audio_buffer.append(chunk.copy())
                else:
                    if speech_active:
                        audio_buffer.append(chunk.copy())
                        if silence_start is None:
                            silence_start = time.time()
                        elif time.time() - silence_start > SILENCE_TIMEOUT:
                            duration = time.time() - speech_start
                            if duration >= MIN_SPEECH_DURATION:
                                full_audio = np.concatenate(audio_buffer)
                                utterance_queue.put(full_audio)
                            speech_active = False
                            audio_buffer = []
                            silence_start = None

    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
