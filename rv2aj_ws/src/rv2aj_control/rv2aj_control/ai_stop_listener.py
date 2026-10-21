"""
Always-on "dur" listener (offline, Vosk small Turkish model).

An extra stop layer next to the STOP button, Esc, push-to-talk "dur" and the
hardware E-stop - never a replacement for them. A small grammar recognizer
limited to dur / durdur / stop listens continuously and proposes a stop; only
then a free recognizer re-reads the last 2.5 s and vetoes it if it clearly
heard a longer "dur..." word (durum, durumu, durdum ...). Running the free
recognizer only on proposals keeps the CPU use low.
Measured on synthetic noisy speech: "dur dur", "hemen dur", "robotu durdur"
are caught reliably; a single short "dur" is sometimes missed.

While the robot itself is speaking (and for a moment after) detections are
ignored, so its own voice cannot stop it.
"""

import json
import os
import threading
import time

MODEL_DIR = os.path.expanduser("~/.local/share/vosk/vosk-model-small-tr-0.3")
STOP_WORDS = {"dur", "durdur", "dursun", "durun", "stop"}
GRAMMAR = ["dur", "durdur", "stop", "[unk]"]
MIN_CONF = 0.8
COOLDOWN_S = 2.0
AFTER_SPEECH_S = 1.0
RATE = 16000
BLOCK = 3200  # 0.2 s


def available():
    try:
        import sounddevice  # noqa: F401
        import vosk  # noqa: F401
    except ImportError:
        return False
    return os.path.isdir(MODEL_DIR)


class StopListener:
    def __init__(self, on_stop, on_error=lambda msg: None):
        self.on_stop, self.on_error = on_stop, on_error
        self._run = False
        self._thread = None
        self._muted_until = 0.0
        self._speaking = False
        self._last_trigger = 0.0

    def set_speaking(self, speaking):
        if self._speaking and not speaking:
            self._muted_until = time.monotonic() + AFTER_SPEECH_S
        self._speaking = speaking

    def start(self):
        if self._run:
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._run = False

    def _muted(self):
        return self._speaking or time.monotonic() < self._muted_until

    def _trigger(self, heard):
        now = time.monotonic()
        if self._muted() or now - self._last_trigger < COOLDOWN_S:
            return
        self._last_trigger = now
        self.on_stop(heard)

    def _loop(self):
        try:
            import sounddevice as sd
            import vosk
            vosk.SetLogLevel(-1)
            model = vosk.Model(MODEL_DIR)
            grammar = vosk.KaldiRecognizer(model, RATE, json.dumps(GRAMMAR))
            grammar.SetWords(True)
            ring = []  # the last ~2.5 s of audio, checked by a free recognizer only on a proposal
            with sd.RawInputStream(samplerate=RATE, blocksize=BLOCK, dtype="int16", channels=1) as stream:
                while self._run:
                    data, _overflow = stream.read(BLOCK)
                    data = bytes(data)
                    ring = (ring + [data])[-int(2.5 * RATE / BLOCK):]
                    if not grammar.AcceptWaveform(data):
                        continue
                    hits = json.loads(grammar.Result()).get("result", [])
                    if not any(h["word"] in STOP_WORDS and h.get("conf", 0) >= MIN_CONF for h in hits):
                        continue
                    free = vosk.KaldiRecognizer(model, RATE)
                    free.AcceptWaveform(b"".join(ring))
                    words = json.loads(free.FinalResult()).get("text", "").split()
                    veto = any(w.startswith("dur") and w not in STOP_WORDS for w in words)  # durum, durdum ...
                    if not veto:
                        self._trigger(" ".join(words) or "dur")
        except Exception as e:  # no microphone, model missing ...
            self._run = False
            self.on_error(str(e))
