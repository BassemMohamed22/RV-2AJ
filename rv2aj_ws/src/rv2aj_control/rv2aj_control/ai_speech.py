"""
Offline speech: push-to-talk recording, faster-whisper transcription on the
CPU (keeps the 4 GB GPU free for the language model), and text-to-speech.

TTS engines, first available wins: piper (if MELFA_PIPER_VOICE points at a
voice .onnx), pyttsx3, then speech-dispatcher's spd-say (espeak-ng).
"""

import os
import re
import queue
import shutil
import subprocess
import tempfile
import threading

SAMPLE_RATE = 16000
MAX_RECORD_S = 20
WHISPER_MODEL = os.environ.get("MELFA_WHISPER_MODEL", "base")  # "small" is more accurate, slower
WHISPER_LANGUAGE = os.environ.get("MELFA_WHISPER_LANGUAGE", "en") or None  # "" = auto-detect
PIPER_VOICE = os.environ.get("MELFA_PIPER_VOICE", "")
WHISPER_PROMPT = None  # vocabulary hint for whisper (set by the GUI for Turkish)

try:
    import numpy as np
    import sounddevice as sd
except ImportError:  # the panel falls back to text input
    np = sd = None


def recording_available():
    return sd is not None


class Recorder:
    """Records mono 16 kHz audio between start() and stop()."""

    def __init__(self):
        self._chunks = []
        self._stream = None
        self._lock = threading.Lock()

    def start(self):
        if sd is None:
            raise RuntimeError("sounddevice is not installed")
        with self._lock:
            self._chunks = []
            self._stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32",
                                          callback=self._callback)
            self._stream.start()

    def _callback(self, data, frames, time_info, status):
        if sum(len(c) for c in self._chunks) < SAMPLE_RATE * MAX_RECORD_S:
            self._chunks.append(data[:, 0].copy())

    def stop(self):
        with self._lock:
            if self._stream is not None:
                self._stream.stop()
                self._stream.close()
                self._stream = None
            chunks, self._chunks = self._chunks, []
        return np.concatenate(chunks) if chunks else np.zeros(0, dtype="float32")


def _preload_cuda_libs():
    """ctranslate2 needs cuBLAS/cuDNN 12; the pip packages that come with torch
    (nvidia-*-cu12) have them, but not on the library path. Loading them by full
    path first lets ctranslate2 find them. True if they could be loaded."""
    import ctypes
    import glob
    try:
        import nvidia
    except ImportError:
        return False
    root = os.path.dirname(nvidia.__path__[0]) + "/nvidia"
    names = ("cuda_runtime/lib/libcudart.so.12", "cublas/lib/libcublasLt.so.12", "cublas/lib/libcublas.so.12",
             "cudnn/lib/libcudnn.so.9")
    try:
        for name in names:
            ctypes.CDLL(os.path.join(root, name), mode=ctypes.RTLD_GLOBAL)
        for lib in sorted(glob.glob(os.path.join(root, "cudnn/lib/libcudnn_*.so.9"))):
            ctypes.CDLL(lib, mode=ctypes.RTLD_GLOBAL)
        return True
    except OSError:
        return False


QUIET_RMS = 0.0025  # below this the recording is silence / room noise

# Text Whisper produces from silence or noise (subtitle credits, video outros).
_HALLUCINATIONS = re.compile(
    r"altyaz|abone ol|izlediğiniz için|izlediginiz icin|bir sonraki video|videoyu beğen|kanalıma|"
    r"^\W*m\.?\s?k\.?\W*$|^\W*(müzik|muzik|alkış|alkis|♪)+\W*$|subtitles by|thanks for watching|amara\.org",
    re.I)


def is_hallucination(text):
    return bool(text) and bool(_HALLUCINATIONS.search(text.strip()))


class Transcriber:
    """faster-whisper, loaded once (call load() early to warm it up). On the GPU
    next to the LLM it transcribes a 4 s command in ~0.2 s instead of ~1.8 s on
    the CPU; the CPU is the fallback. Offline: only the local model cache is used."""

    def __init__(self, device="auto"):
        self._model = None
        self._lock = threading.Lock()
        self.want = device       # "auto": the GPU if it works, else the CPU; "cpu": always the CPU
        self.device = "cpu"

    def load(self):
        with self._lock:
            if self._model is None:
                from faster_whisper import WhisperModel
                if self.want != "cpu" and _preload_cuda_libs():
                    try:
                        self._model = WhisperModel(WHISPER_MODEL, device="cuda", compute_type="int8_float16",
                                                   local_files_only=True)
                        self.device = "cuda"
                    except Exception:  # no GPU memory, driver or library problem: use the CPU
                        self._model = None
                if self._model is None:
                    self._model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8", local_files_only=True)
                    self.device = "cpu"
        return self._model

    def transcribe(self, audio):
        """Text of `audio`, or "" when there is no real speech in it. Whisper invents
        text on silence or noise ("Altyazı M.K.", "İzlediğiniz için teşekkürler" are
        subtitle credits from its training data), and a clear command now runs without
        confirmation, so silence, non-speech segments and those phrases are dropped."""
        if audio is None or len(audio) < SAMPLE_RATE * 0.3:
            return ""
        if float(np.sqrt(np.mean(np.square(audio)))) < QUIET_RMS:
            return ""  # nothing louder than a quiet room
        try:
            segments = list(self._segments(self.load(), audio))
        except RuntimeError as e:  # e.g. CUDA out of memory mid-run: redo it on the CPU
            if self.device != "cuda":
                raise
            from faster_whisper import WhisperModel
            with self._lock:
                self._model = WhisperModel(WHISPER_MODEL, device="cpu", compute_type="int8", local_files_only=True)
                self.device = "cpu"
            segments = list(self._segments(self._model, audio))
        kept = [seg.text.strip() for seg in segments
                if not (seg.no_speech_prob > 0.6 and seg.avg_logprob < -1.0) and seg.compression_ratio < 2.4]
        text = " ".join(kept).strip()
        return "" if is_hallucination(text) else text

    @staticmethod
    def _segments(model, audio):
        segments, _info = model.transcribe(audio, language=WHISPER_LANGUAGE, beam_size=1,
                                           condition_on_previous_text=False, initial_prompt=WHISPER_PROMPT,
                                           vad_filter=True, vad_parameters={"threshold": 0.35})
        return segments


def _tts_engine():
    if PIPER_VOICE and os.path.exists(PIPER_VOICE) and shutil.which("piper") and shutil.which("aplay"):
        return "piper"
    try:
        import pyttsx3  # noqa: F401
        return "pyttsx3"
    except ImportError:
        pass
    if shutil.which("spd-say"):
        return "spd-say"
    return None


class Speaker:
    """Speaks queued sentences one at a time on a background thread."""

    def __init__(self):
        self.engine = _tts_engine()
        self.enabled = True
        self._queue = queue.Queue()
        threading.Thread(target=self._run, daemon=True).start()

    def say(self, text):
        if self.enabled and self.engine and text:
            self._queue.put(text)

    def _run(self):
        tts = None
        while True:
            text = self._queue.get()
            try:
                if self.engine == "pyttsx3":
                    if tts is None:
                        import pyttsx3
                        tts = pyttsx3.init()
                    tts.say(text)
                    tts.runAndWait()
                elif self.engine == "piper":
                    with tempfile.NamedTemporaryFile(suffix=".wav") as wav:
                        subprocess.run(["piper", "--model", PIPER_VOICE, "--output_file", wav.name],
                                       input=text.encode(), check=True, capture_output=True, timeout=30)
                        subprocess.run(["aplay", "-q", wav.name], check=False, timeout=60)
                elif self.engine == "spd-say":
                    subprocess.run(["spd-say", "-w", "--", text], check=False, timeout=60,
                                   capture_output=True)
            except Exception:
                pass  # speech is a convenience; the panel always shows the text
