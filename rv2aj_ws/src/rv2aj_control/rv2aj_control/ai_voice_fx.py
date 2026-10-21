"""
Jarvis-style voice processing and interface sounds, pure numpy/scipy (no sox).

Voice: Piper speaks slightly fast, the audio is then stretched by 12/11, which
lowers the pitch about 1.5 semitones while the pace ends up calm and a little
slow. A subtle double (the "AI" timbre), a short room reverb, some presence
and a soft limiter finish it.

Sounds: short synthesized cues (startup, listen, think, confirm, done, error,
stop), cached as WAV files and played with aplay.
"""

import io
import os
import subprocess
import threading
import wave

import numpy as np
from scipy import signal

SR = 22050
PITCH_UP, PITCH_DOWN = 12, 11          # stretch factor 12/11: about -1.5 semitones
CALM_LENGTH = 1.06                    # final pace relative to Piper's normal speed
CACHE_DIR = os.path.expanduser("~/.cache/rv2aj_sounds")


# -- voice ---------------------------------------------------------------------------

def synth_length_scale():
    """Piper length_scale that, after the stretch, gives the calm pace."""
    return CALM_LENGTH * PITCH_DOWN / PITCH_UP


def _reverb_ir(sr, seconds=0.32, seed=7):
    rng = np.random.default_rng(seed)
    n = int(sr * seconds)
    ir = rng.standard_normal(n) * np.exp(-np.linspace(0, 7.0, n))
    b, a = signal.butter(2, 3500 / (sr / 2), "low")
    ir = signal.lfilter(b, a, ir)
    return ir / np.abs(ir).sum()


def jarvis(samples, sr=SR):
    """int16 mono Piper audio -> processed int16 audio."""
    x = samples.astype(np.float64) / 32768.0
    x = signal.resample_poly(x, PITCH_UP, PITCH_DOWN)            # lower pitch, longer
    b, a = signal.butter(2, 85 / (sr / 2), "high")                # no rumble
    x = signal.lfilter(b, a, x)
    d1, d2 = int(0.011 * sr), int(0.023 * sr)                     # subtle "AI" double
    y = x.copy()
    y[d1:] += 0.20 * x[:-d1]
    y[d2:] += 0.12 * x[:-d2]
    bh, ah = signal.butter(2, 2600 / (sr / 2), "high")            # presence
    y += 0.22 * signal.lfilter(bh, ah, y)
    wet = signal.fftconvolve(y, _reverb_ir(sr))[: len(y)]
    y = 0.88 * y + 0.35 * wet
    tail = np.zeros(int(0.15 * sr))
    y = np.concatenate([y, 0.35 * signal.fftconvolve(np.concatenate([y[-int(0.3 * sr):], tail]),
                                                      _reverb_ir(sr))[int(0.3 * sr):int(0.3 * sr) + len(tail)]])
    y = np.tanh(1.6 * y / (np.abs(y).max() + 1e-9)) / np.tanh(1.6)  # soft limiter
    return (y * 0.9 * 32767).astype(np.int16)


def wav_bytes(samples, sr=SR):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(samples.astype(np.int16).tobytes())
    return buf.getvalue()


# -- interface sounds ------------------------------------------------------------------

def _t(seconds):
    return np.arange(int(SR * seconds)) / SR


def _env(n, attack=0.005, release=0.08):
    e = np.ones(n)
    a, r = int(attack * SR), int(release * SR)
    e[:a] = np.linspace(0, 1, a)
    e[-r:] *= np.linspace(1, 0, r)
    return e


def _tone(freq, seconds, harmonics=(1.0, 0.25, 0.08), release=0.08):
    t = _t(seconds)
    x = sum(h * np.sin(2 * np.pi * freq * (k + 1) * t) for k, h in enumerate(harmonics))
    return x * _env(len(t), release=release)


def _sweep(f0, f1, seconds):
    t = _t(seconds)
    phase = 2 * np.pi * (f0 * t + (f1 - f0) * t ** 2 / (2 * seconds))
    return (np.sin(phase) + 0.2 * np.sin(2 * phase)) * _env(len(t), release=seconds * 0.4)


def _seq(parts, gap=0.0):
    out = []
    for p in parts:
        out.append(p)
        out.append(np.zeros(int(gap * SR)))
    return np.concatenate(out)


def _mix_at(base, add, start_s):
    i = int(start_s * SR)
    end = i + len(add)
    if end > len(base):
        base = np.concatenate([base, np.zeros(end - len(base))])
    base[i:end] += add
    return base


def _finish(x, level=0.5):
    wet = signal.fftconvolve(x, _reverb_ir(SR, 0.5, seed=3))[: len(x) + int(0.35 * SR)]
    x = np.concatenate([x, np.zeros(len(wet) - len(x))]) + 0.9 * wet
    return (x / (np.abs(x).max() + 1e-9) * level * 32767).astype(np.int16)


def make(name):
    if name == "startup":                                           # rising power-up + chord
        x = _sweep(180, 900, 0.45) * 0.5
        for k, f in enumerate((659.3, 830.6, 987.8, 1318.5)):      # E5 G#5 B5 E6
            x = _mix_at(x, _tone(f, 0.55, release=0.4) * 0.45, 0.30 + 0.07 * k)
        return _finish(x, 0.45)
    if name == "listen":                                            # two quick rising blips
        return _finish(_seq([_tone(880, 0.06, release=0.03), _tone(1320, 0.08, release=0.05)], gap=0.02), 0.35)
    if name == "send":
        return _finish(_tone(1175, 0.07, release=0.05), 0.3)
    if name == "think":                                             # soft processing shimmer
        x = _sweep(700, 1050, 0.18) * 0.4
        return _finish(x, 0.22)
    if name == "confirm":                                           # question chime
        return _finish(_seq([_tone(1174.7, 0.10, release=0.06), _tone(1568.0, 0.18, release=0.14)], gap=0.03), 0.38)
    if name == "done":                                              # rising triad
        return _finish(_seq([_tone(f, 0.09, release=0.06) for f in (784.0, 987.8, 1174.7)], gap=0.015), 0.38)
    if name == "error":                                             # two low tones
        return _finish(_seq([_tone(392, 0.12, (1, 0.5, 0.3)), _tone(311, 0.20, (1, 0.5, 0.3), 0.15)], gap=0.04), 0.4)
    if name == "stop":                                              # falling sweep
        return _finish(_sweep(1100, 220, 0.28), 0.45)
    raise ValueError(name)


SOUNDS = ("startup", "listen", "send", "think", "confirm", "done", "error", "stop")


def sound_file(name):
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, f"{name}.wav")
    if not os.path.exists(path):
        with open(path, "wb") as f:
            f.write(wav_bytes(make(name)))
    return path


def play_sound(name, block=False):
    """Play an interface sound with aplay (non-blocking by default)."""
    try:
        proc = subprocess.Popen(["aplay", "-q", sound_file(name)], stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
        if block:
            proc.wait(timeout=5)
        return proc
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def prebuild():
    threading.Thread(target=lambda: [sound_file(n) for n in SOUNDS], daemon=True).start()


# -- light "digital assistant" touch for natural (neural) voices -------------------------

def light(samples, sr):
    """Very subtle: rumble cut, a little presence and a short room. No pitch
    change and no metallic doubling (those made the voice sound worse)."""
    x = samples.astype(np.float64) / 32768.0
    b, a = signal.butter(2, 80 / (sr / 2), "high")
    x = signal.lfilter(b, a, x)
    bh, ah = signal.butter(2, 3000 / (sr / 2), "high")
    x = x + 0.12 * signal.lfilter(bh, ah, x)
    wet = signal.fftconvolve(x, _reverb_ir(sr, 0.22))[: len(x) + int(0.18 * sr)]
    x = np.concatenate([x, np.zeros(len(wet) - len(x))])
    y = 0.94 * x + 0.16 * wet
    y = np.tanh(1.2 * y / (np.abs(y).max() + 1e-9)) / np.tanh(1.2)
    return (y * 0.9 * 32767).astype(np.int16)


# -- Microsoft neural voices through edge-tts (online) -------------------------------------

EDGE_CACHE = os.path.expanduser("~/.cache/rv2aj_tts")


def edge_speech(text, voice="tr-TR-AhmetNeural", rate="-5%", pitch="-2Hz", timeout=8.0):
    """(int16 samples, sample rate) of `text` spoken by a Microsoft neural voice.
    Results are cached on disk, so repeated phrases play instantly. Raises on
    any network or decoding problem (the caller falls back to Piper)."""
    import asyncio
    import hashlib
    import soundfile as sf
    os.makedirs(EDGE_CACHE, exist_ok=True)
    key = hashlib.sha1(f"{voice}|{rate}|{pitch}|{text}".encode()).hexdigest()
    path = os.path.join(EDGE_CACHE, key + ".mp3")
    if not os.path.exists(path):
        import edge_tts

        async def fetch():
            await edge_tts.Communicate(text, voice, rate=rate, pitch=pitch).save(path + ".part")

        asyncio.run(asyncio.wait_for(fetch(), timeout))
        os.replace(path + ".part", path)
    data, sr = sf.read(path, dtype="int16")
    if data.ndim > 1:
        data = data[:, 0]
    loud = np.flatnonzero(np.abs(data) > 400)  # trim leading/trailing silence: speaks sooner
    if len(loud):
        data = data[max(0, loud[0] - int(0.04 * sr)): loud[-1] + int(0.12 * sr)]
    return data, sr
