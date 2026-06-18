"""
Jetson Mic Streamer
Captures audio from Brio webcam mic and streams to desktop PC over UDP.

Run on Jetson: python3 jetson_mic_stream.py
"""

import socket
import subprocess
import sys

# --- Configuration ---
PC_IP = "192.168.0.104"  # Your desktop PC's Wi-Fi IP - UPDATE THIS
PC_PORT = 5006           # Port the PC listens on for audio
DEVICE = "plughw:0,0"    # Brio webcam mic
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = "S16_LE"
CHUNK_SIZE = 1024        # bytes per UDP packet

def main():
    print(f"Streaming mic audio to {PC_IP}:{PC_PORT}")
    print(f"Device: {DEVICE}, Rate: {SAMPLE_RATE}Hz")
    print("Press Ctrl+C to stop")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Use arecord to capture audio and pipe it
    cmd = [
        "arecord",
        "-D", DEVICE,
        "-f", FORMAT,
        "-r", str(SAMPLE_RATE),
        "-c", str(CHANNELS),
        "-t", "raw",
        "--buffer-size", "512",
    ]

    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        print("Streaming...")

        while True:
            data = proc.stdout.read(CHUNK_SIZE)
            if not data:
                break
            sock.sendto(data, (PC_IP, PC_PORT))

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        proc.terminate()
        sock.close()

if __name__ == "__main__":
    main()
