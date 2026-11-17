#!/usr/bin/env python3
"""
Measure the local voice stack's latency on this machine. No audio hardware
needed - speech is synthesised to a buffer and fed straight back to the
recogniser.

Run the speech half with the venv interpreter and the LLM half with the system
one, because the Jetson's CUDA torch lives in ~/.local and the speech stack
lives in its own venv:

    ~/buddy_speech_venv/bin/python scripts/voice/bench_voice_stack.py --part speech
    python3 scripts/voice/bench_voice_stack.py --part llm --model Qwen/Qwen2.5-1.5B-Instruct

WHY THE NUMBERS LOOK THE WAY THEY DO
------------------------------------
The figure that decides whether a robot feels alive is TIME TO FIRST AUDIO, not
total generation time. With sentence-by-sentence TTS the robot starts speaking
as soon as the first sentence exists, so a long reply starts no later than a
short one. Measure first-audio, not throughput.

MEMORY: on a Jetson, GPU memory is shared with system RAM. A desktop session
with VS Code and a language server can hold ~2.3 GB, which is the difference
between a 1.5B model fitting and not. Benchmark headless.
"""
import argparse, time, sys

MODELS = '/home/alex/buddy_speech_models'


def bench_speech():
    import numpy as np
    print('--- TTS (piper) ---')
    from piper import PiperVoice
    D = f'{MODELS}/vits-piper-en_US-lessac-medium'
    v = PiperVoice.load(f'{D}/en_US-lessac-medium.onnx',
                        config_path=f'{D}/en_US-lessac-medium.onnx.json')
    firsts = []
    for _ in range(5):
        t0 = time.time(); next(iter(v.synthesize('Nice to meet you.')))
        firsts.append(time.time()-t0)
    print(f'  first audio: min {min(firsts)*1000:.0f} ms  median {sorted(firsts)[2]*1000:.0f} ms')

    long = "Hello, I don't think we've met. What's your name?"
    t0 = time.time(); chunks = list(v.synthesize(long)); dt = time.time()-t0
    sr = chunks[0].sample_rate
    n = sum(len(c.audio_int16_bytes)//2 for c in chunks)
    print(f'  full utterance: {n/sr:.2f}s audio in {dt:.2f}s ({(n/sr)/dt:.1f}xRT)')

    pcm = b''.join(c.audio_int16_bytes for c in chunks)
    samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)/32768

    print('--- STT (sherpa-onnx streaming zipformer) ---')
    import sherpa_onnx
    S = f'{MODELS}/sherpa-onnx-streaming-zipformer-en-20M-2023-02-17'
    rec = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=f'{S}/tokens.txt',
        encoder=f'{S}/encoder-epoch-99-avg-1.int8.onnx',
        decoder=f'{S}/decoder-epoch-99-avg-1.onnx',
        joiner=f'{S}/joiner-epoch-99-avg-1.int8.onnx',
        num_threads=4, enable_endpoint_detection=True,
        rule2_min_trailing_silence=0.6)
    st = rec.create_stream()
    chunk = int(sr*0.1); t0 = time.time()
    for i in range(0, len(samples), chunk):
        st.accept_waveform(sr, samples[i:i+chunk])
        while rec.is_ready(st):
            rec.decode_stream(st)
    st.accept_waveform(sr, np.zeros(int(sr*0.5), dtype=np.float32))
    st.input_finished()
    while rec.is_ready(st):
        rec.decode_stream(st)
    dt = time.time()-t0
    print(f'  {n/sr:.2f}s audio decoded in {dt:.2f}s ({(n/sr)/dt:.1f}xRT)')
    print(f'  said : {long}')
    print(f'  heard: {rec.get_result(st).strip()}')
    print()
    print(f'  budget: ~400 ms endpoint + ~50 ms decode + {min(firsts)*1000:.0f} ms TTS')
    print(f'       => ~{400+50+min(firsts)*1000:.0f} ms to first sound, no LLM')


def bench_llm(model_id):
    import torch
    import transformers.modeling_utils as mu
    # transformers pre-allocates the whole model as ONE contiguous block before
    # loading. On Jetson's shared CPU/GPU memory that single allocation fails
    # even when the total is free. Weights stream in fine without it.
    mu.caching_allocator_warmup = lambda *a, **k: None
    from transformers import AutoModelForCausalLM, AutoTokenizer

    free, total = [x/1e9 for x in torch.cuda.mem_get_info()]
    print(f'GPU free {free:.2f}/{total:.2f} GB   model={model_id}')
    if free < 3.5 and '1.5B' in model_id:
        print('  NOTE: a 1.5B fp16 model needs ~3.1 GB plus activations and CUDA')
        print('        context. Close VS Code / language servers and run headless.')

    tok = AutoTokenizer.from_pretrained(model_id)
    t0 = time.time()
    model = AutoModelForCausalLM.from_pretrained(
        model_id, dtype=torch.float16, device_map={'': 0},
        low_cpu_mem_usage=True).eval()
    print(f'  loaded in {time.time()-t0:.1f}s; GPU allocated '
          f'{torch.cuda.memory_allocated()/1e9:.2f} GB')

    SYS = ('You are Buddy, a small home robot with a warm, playful personality. '
           'Keep replies to one or two short sentences.')

    def ask(q, n=40):
        enc = tok.apply_chat_template(
            [{'role': 'system', 'content': SYS}, {'role': 'user', 'content': q}],
            add_generation_prompt=True, return_tensors='pt',
            return_dict=True).to('cuda')
        plen = enc['input_ids'].shape[-1]
        t0 = time.time()
        with torch.no_grad():
            out = model.generate(**enc, max_new_tokens=n, do_sample=True,
                                 temperature=0.7, top_p=0.9,
                                 pad_token_id=tok.eos_token_id)
        torch.cuda.synchronize()
        dt = time.time()-t0
        k = out.shape[-1]-plen
        return k, dt, tok.decode(out[0][plen:], skip_special_tokens=True).strip()

    ask('warmup', 4)
    for q in ['Hi Buddy, what are you up to?',
              'Follow me to the kitchen.',
              "I don't think we've met. What should I call you?"]:
        k, dt, txt = ask(q)
        print(f'> {q}')
        print(f'  {k} tok in {dt:.2f}s = {k/dt:.1f} tok/s')
        print(f'  "{txt[:120]}"')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--part', choices=['speech', 'llm'], required=True)
    ap.add_argument('--model', default='Qwen/Qwen2.5-0.5B-Instruct')
    a = ap.parse_args()
    if a.part == 'speech':
        bench_speech()
    else:
        bench_llm(a.model)
