#!/usr/bin/env python3
"""MPU-9250/6500 serial decoding and stationary gyro calibration (SI units)."""
from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class ImuSample:
    micros: int
    acceleration: tuple
    gyro: tuple


def parse_imu_line(line):
    if not line.startswith('I:'):
        raise ValueError('not an IMU frame')
    fields = line[2:].split(',')
    if len(fields) != 7:
        raise ValueError('expected timestamp and six axes')
    values = [int(x) for x in fields]
    if not 0 <= values[0] < 2**32:
        raise ValueError('timestamp out of range')
    if any(not -32768 <= x <= 32767 for x in values[1:]):
        raise ValueError('raw sensor value out of range')
    accel = tuple(x * 9.80665 / 16384.0 for x in values[1:4])
    gyro = tuple(math.radians(x / 131.0) for x in values[4:7])
    return ImuSample(values[0], accel, gyro)


class ImuProcessor:
    """Use the MCU clock between frames; recalibrate after reset/data outage.

    Calibration requires 2 s warm-up then a contiguous stationary window.
    Encoder motion, a drive command, excessive gyro or non-gravity acceleration
    resets that window. Never silently learn a new bias during an actual turn.
    """
    def __init__(self, samples=200, warmup=2.0):
        if samples < 2 or warmup < 0:
            raise ValueError('invalid calibration window')
        self.samples = samples
        self.warmup_us = int(warmup * 1e6)
        self.reset()

    def reset(self):
        self.previous_us = None
        self.elapsed_us = 0
        self.anchor_ns = None
        self.bias = None
        self.window = []

    def update(self, sample, host_ns, stationary):
        if self.previous_us is None:
            self.anchor_ns = host_ns
        else:
            delta = (sample.micros - self.previous_us) % 2**32
            if delta == 0:
                return None
            if delta > 1_000_000:
                self.reset()
                self.anchor_ns = host_ns
            else:
                self.elapsed_us += delta
        self.previous_us = sample.micros
        stamp_ns = min(self.anchor_ns + self.elapsed_us * 1000, host_ns)
        if self.bias is None:
            accel_norm = math.sqrt(sum(x*x for x in sample.acceleration))
            gyro_norm = math.sqrt(sum(x*x for x in sample.gyro))
            if (not stationary or self.elapsed_us < self.warmup_us or
                    not 8.5 < accel_norm < 11.0 or gyro_norm > 0.15):
                self.window.clear()
            else:
                self.window.append(sample.gyro)
                if len(self.window) >= self.samples:
                    axes = list(zip(*self.window))
                    if max(statistics.stdev(axis) for axis in axes) < 0.01:
                        self.bias = tuple(statistics.mean(axis) for axis in axes)
                    self.window.clear()
        corrected = (tuple(x-b for x, b in zip(sample.gyro, self.bias))
                     if self.bias is not None else None)
        return stamp_ns, corrected
