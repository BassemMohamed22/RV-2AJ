"""Background speech listener using Google's web speech API.

Audio capture and recognition run on speech_recognition's own threads;
results reach the GUI only through Qt signals.
"""

import threading

from PyQt5.QtCore import QObject, pyqtSignal

try:
    import speech_recognition as sr
except ImportError:  # voice is optional; the GUI disables it
    sr = None

LANGUAGE = "en-US"
PHRASE_TIME_LIMIT_S = 4
CALIBRATION_S = 1.5


def available():
    return sr is not None


def microphone_names():
    if sr is None:
        return []
    try:
        return list(sr.Microphone.list_microphone_names())
    except Exception:
        return []


class VoiceListener(QObject):
    heard = pyqtSignal(str)
    status = pyqtSignal(str, str)  # text, kind (idle/ok/warn/error)

    def __init__(self):
        super().__init__()
        self._stopper = None
        self._generation = 0
        self._lock = threading.Lock()

    def start(self, device_index=None):
        with self._lock:
            self._generation += 1
            generation = self._generation
        threading.Thread(target=self._start_blocking, args=(device_index, generation),
                         daemon=True).start()

    def stop(self):
        with self._lock:
            self._generation += 1
            stopper, self._stopper = self._stopper, None
        if stopper is not None:
            stopper(wait_for_stop=False)
        self.status.emit("Voice off", "idle")

    def _start_blocking(self, device_index, generation):
        recognizer = sr.Recognizer()
        recognizer.dynamic_energy_threshold = True
        recognizer.energy_threshold = 150
        try:
            microphone = sr.Microphone(device_index=device_index)
            self.status.emit("Calibrating… stay quiet", "warn")
            with microphone as source:
                recognizer.adjust_for_ambient_noise(source, duration=CALIBRATION_S)
        except Exception as e:
            self.status.emit(f"Microphone error: {e}", "error")
            return

        callback = lambda r, audio: self._recognize(r, audio, generation)
        with self._lock:
            if generation != self._generation:
                return
            self._stopper = recognizer.listen_in_background(
                microphone, callback, phrase_time_limit=PHRASE_TIME_LIMIT_S
            )
        self.status.emit("Listening", "ok")

    def _recognize(self, recognizer, audio, generation):
        try:
            text = recognizer.recognize_google(audio, language=LANGUAGE)
        except sr.UnknownValueError:
            return
        except sr.RequestError as e:
            self.status.emit(f"Google API error: {e}", "error")
            return
        if generation == self._generation and text:
            self.heard.emit(text)
