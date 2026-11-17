#!/usr/bin/env python3
"""
Swappable voice backend: speak(), transcribe(), chat().

Local by default, cloud by environment variable, same calling code either way.
The point is that choosing where inference runs is a config line and not a
rewrite, so local and cloud can be A/B'd on the real robot.

    BUDDY_TTS   piper (default) | openai | gemini
    BUDDY_STT   sherpa (default)
    BUDDY_LLM_BASE_URL   default http://localhost:11434/v1  (Ollama, llama.cpp,
                         vLLM and LM Studio all speak this; so does OpenAI and
                         Google's compatibility endpoint)
    BUDDY_LLM_MODEL      default qwen2.5:1.5b-instruct
    BUDDY_LLM_API_KEY    ignored by local servers, required by cloud

NOTHING here handles commands. "stop" and "follow me" are matched in
buddy_intents.py and never touch this module, so they keep working when the
network is down. That is deliberate - see the note there.

MUST be run with the speech venv interpreter, with ROS sourced:

    source /opt/ros/humble/setup.bash && source install/setup.bash
    ~/buddy_speech_venv/bin/python ...

rclpy then comes from ROS's PYTHONPATH and piper/sherpa from the venv. Both are
Python 3.10, so they coexist.
"""
import os
import re
from typing import Iterator, List, Optional

MODELS = os.environ.get('BUDDY_SPEECH_MODELS', '/home/alex/buddy_speech_models')
PIPER_DIR = f'{MODELS}/vits-piper-en_US-lessac-medium'
STT_DIR = f'{MODELS}/sherpa-onnx-streaming-zipformer-en-20M-2023-02-17'

# Split on sentence ends so the first sentence can be spoken while the rest is
# still being synthesised. This is where perceived latency actually comes from:
# a long reply then starts no later than a short one.
_SENTENCE = re.compile(r'(?<=[.!?])\s+')


def split_sentences(text: str) -> List[str]:
    return [s.strip() for s in _SENTENCE.split((text or '').strip()) if s.strip()]


class PiperTTS:
    """Local TTS. Measured 86-300 ms to first audio on the Jetson."""

    def __init__(self, model_dir: str = PIPER_DIR):
        from piper import PiperVoice
        self.voice = PiperVoice.load(
            f'{model_dir}/en_US-lessac-medium.onnx',
            config_path=f'{model_dir}/en_US-lessac-medium.onnx.json')
        self.sample_rate = None

    def stream(self, text: str) -> Iterator[bytes]:
        """Yield int16 PCM a sentence at a time."""
        for sentence in split_sentences(text) or [text]:
            for chunk in self.voice.synthesize(sentence):
                self.sample_rate = chunk.sample_rate
                yield chunk.audio_int16_bytes


class OpenAITTS:
    """Cloud TTS over the OpenAI audio API. Also fits any compatible server."""

    def __init__(self, model='gpt-4o-mini-tts', voice='alloy',
                 base_url=None, api_key=None):
        from openai import OpenAI
        self.client = OpenAI(base_url=base_url or os.environ.get('BUDDY_TTS_BASE_URL'),
                             api_key=api_key or os.environ.get('BUDDY_TTS_API_KEY')
                             or os.environ.get('OPENAI_API_KEY'))
        self.model, self.voice = model, voice
        self.sample_rate = 24000

    def stream(self, text: str) -> Iterator[bytes]:
        for sentence in split_sentences(text) or [text]:
            r = self.client.audio.speech.create(
                model=self.model, voice=self.voice, input=sentence,
                response_format='pcm')
            yield r.content


class SherpaSTT:
    """Local streaming recogniser. Measured 7.9-12.4x realtime."""

    def __init__(self, model_dir: str = STT_DIR, endpoint_silence: float = 0.6):
        import sherpa_onnx
        self.rec = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=f'{model_dir}/tokens.txt',
            encoder=f'{model_dir}/encoder-epoch-99-avg-1.int8.onnx',
            decoder=f'{model_dir}/decoder-epoch-99-avg-1.onnx',
            joiner=f'{model_dir}/joiner-epoch-99-avg-1.int8.onnx',
            num_threads=4, enable_endpoint_detection=True,
            # The dominant latency term in conversation. The old desktop
            # pipeline used 1.5 s, which is 1.5 s of dead air after every
            # sentence before anything else could start.
            rule2_min_trailing_silence=endpoint_silence)

    def transcribe(self, samples, sample_rate: int) -> str:
        """One-shot transcription of a complete utterance."""
        import numpy as np
        st = self.rec.create_stream()
        chunk = int(sample_rate * 0.1)
        for i in range(0, len(samples), chunk):
            st.accept_waveform(sample_rate, samples[i:i + chunk])
            while self.rec.is_ready(st):
                self.rec.decode_stream(st)
        st.accept_waveform(sample_rate, np.zeros(int(sample_rate * 0.5),
                                                 dtype='float32'))
        st.input_finished()
        while self.rec.is_ready(st):
            self.rec.decode_stream(st)
        return self.rec.get_result(st).strip()


class Chat:
    """
    Conversation, via the OpenAI chat API.

    This one protocol covers everything: Ollama and llama.cpp on the robot, a
    model on the desktop over the LAN, OpenAI, and Gemini's compatibility
    endpoint. Only base_url and the key change, so switching is a config line.
    Local servers ignore the key but the client requires one, hence 'local'.
    """

    SYSTEM = ("You are Buddy, a small home robot that rolls around the house. "
              "You are warm, curious and a little playful. Keep replies to one "
              "or two short sentences - you are speaking out loud, not writing. "
              "Never claim to be an AI language model; you are Buddy.")

    def __init__(self, base_url=None, model=None, api_key=None):
        from openai import OpenAI
        self.base_url = base_url or os.environ.get(
            'BUDDY_LLM_BASE_URL', 'http://localhost:11434/v1')
        self.model = model or os.environ.get('BUDDY_LLM_MODEL',
                                             'qwen2.5:1.5b-instruct')
        self.client = OpenAI(
            base_url=self.base_url,
            api_key=api_key or os.environ.get('BUDDY_LLM_API_KEY', 'local'))

    def reply(self, user_text: str, history: Optional[List[dict]] = None,
              timeout: float = 8.0) -> str:
        msgs = [{'role': 'system', 'content': self.SYSTEM}]
        msgs += history or []
        msgs.append({'role': 'user', 'content': user_text})
        r = self.client.chat.completions.create(
            model=self.model, messages=msgs, max_tokens=80,
            temperature=0.7, timeout=timeout)
        return (r.choices[0].message.content or '').strip()


def make_tts():
    kind = os.environ.get('BUDDY_TTS', 'piper').lower()
    if kind == 'piper':
        return PiperTTS()
    if kind in ('openai', 'gemini'):
        return OpenAITTS()
    raise ValueError(f'unknown BUDDY_TTS={kind!r}')


def make_stt():
    kind = os.environ.get('BUDDY_STT', 'sherpa').lower()
    if kind == 'sherpa':
        return SherpaSTT()
    raise ValueError(f'unknown BUDDY_STT={kind!r}')
