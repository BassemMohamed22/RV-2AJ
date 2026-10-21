#!/usr/bin/env bash
# Checks what the AI control layer needs and reports what is missing.
# It only reads and reports: it installs, downloads and deletes nothing.
#
#   ./setup_ai.sh             check with the default model (rv2aj-qwen)
#   MELFA_AI_MODEL=llama3.2:3b ./setup_ai.sh

MODEL="${MELFA_AI_MODEL:-rv2aj-qwen}"
WHISPER="${MELFA_WHISPER_MODEL:-base}"
OLLAMA_URL="${OLLAMA_HOST:-http://127.0.0.1:11434}"
[[ "$OLLAMA_URL" == http* ]] || OLLAMA_URL="http://$OLLAMA_URL"

# Approximate sizes in MB.
OLLAMA_MB=3200          # Ollama binary + bundled CUDA libraries (Linux, /usr/local)
MODEL_MB=2900           # rv2aj-qwen (Qwen3-4B fine-tune, Q4_K_M)
WHISPER_BASE_MB=145
WHISPER_SMALL_MB=470
PIP_MB=50               # pyttsx3 etc.; faster-whisper and ctranslate2 already installed
HEADROOM_MB=1024        # keep at least 1 GB free so the system stays healthy

ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
miss() { printf '  \033[33m✗\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }

need_mb=$HEADROOM_MB
todo=()

echo "== Disk space (df -h)"
df -h / "$HOME" | awk 'NR==1 || !seen[$0]++'
avail_mb=$(df --output=avail -m "$HOME" | tail -1 | tr -d ' ')
echo

echo "== Memory / GPU"
free -h | sed -n '1,2p'
if command -v nvidia-smi >/dev/null; then
    nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader | sed 's/^/  GPU: /'
    info "a 3B model at Q4 (~2 GB) fits the 4 GB VRAM; do NOT pull a 7B model (spills into RAM)"
else
    miss "nvidia-smi not found: Ollama would run on the CPU (slow)"
fi
echo

echo "== Ollama (local language model server)"
OLLAMA_BIN="$(command -v ollama || echo "$HOME/.local/ollama/bin/ollama")"
if [ -x "$OLLAMA_BIN" ]; then
    ok "ollama installed: $OLLAMA_BIN ($("$OLLAMA_BIN" --version 2>/dev/null | grep -o '[0-9][0-9.]*' | tail -1))"
    if curl -fsS --max-time 3 "$OLLAMA_URL/api/tags" -o /tmp/.melfa_tags.json 2>/dev/null; then
        ok "server running at $OLLAMA_URL"
        if grep -q "\"$MODEL\"" /tmp/.melfa_tags.json; then
            ok "model $MODEL pulled"
        else
            miss "model $MODEL not pulled (~${MODEL_MB} MB)"
            need_mb=$((need_mb + MODEL_MB)); todo+=("ollama pull $MODEL")
        fi
    else
        miss "server not running at $OLLAMA_URL"
        todo+=("./run.sh starts it automatically, or: $OLLAMA_BIN serve &")
        need_mb=$((need_mb + MODEL_MB)); todo+=("ollama pull $MODEL   # if not pulled yet")
    fi
else
    miss "ollama not installed (~${OLLAMA_MB} MB) + model $MODEL (~${MODEL_MB} MB)"
    need_mb=$((need_mb + OLLAMA_MB + MODEL_MB))
    todo+=("curl -fsSL https://ollama.com/install.sh | sh")
    todo+=("ollama pull $MODEL")
fi
echo

echo "== Speech to text (faster-whisper, CPU)"
if python3 -c "import faster_whisper" 2>/dev/null; then
    ok "faster-whisper $(python3 -c 'import faster_whisper; print(faster_whisper.__version__)')"
else
    miss "faster-whisper not installed"; need_mb=$((need_mb + 300)); todo+=("pip install --user faster-whisper")
fi
if ls -d "$HOME/.cache/huggingface/hub/models--Systran--faster-whisper-$WHISPER" >/dev/null 2>&1; then
    ok "whisper '$WHISPER' model cached ($(du -sh "$HOME/.cache/huggingface/hub/models--Systran--faster-whisper-$WHISPER" | cut -f1))"
else
    size=$WHISPER_BASE_MB; [ "$WHISPER" = small ] && size=$WHISPER_SMALL_MB
    miss "whisper '$WHISPER' model not cached (~${size} MB, downloaded automatically on first use)"
    need_mb=$((need_mb + size))
fi
for mod in sounddevice numpy requests; do
    if python3 -c "import $mod" 2>/dev/null; then ok "python: $mod"; else
        miss "python: $mod missing"; todo+=("pip install --user $mod"); fi
done
echo

echo "== Text to speech (offline)"
tts=""
if [ -n "$MELFA_PIPER_VOICE" ] && [ -f "$MELFA_PIPER_VOICE" ] && command -v piper >/dev/null; then
    ok "piper with voice $MELFA_PIPER_VOICE"; tts=piper
fi
if python3 -c "import pyttsx3" 2>/dev/null; then ok "pyttsx3"; tts=${tts:-pyttsx3}
else info "pyttsx3 not installed (optional: pip install --user pyttsx3, <1 MB)"; fi
if command -v spd-say >/dev/null; then ok "spd-say (speech-dispatcher / espeak-ng)"; tts=${tts:-spd-say}; fi
[ -z "$tts" ] && { miss "no TTS engine: the panel will show text only"; todo+=("sudo apt install speech-dispatcher espeak-ng"); }
echo

echo "== Summary"
echo "  free on $HOME: ${avail_mb} MB, needed for what is missing (incl. ${HEADROOM_MB} MB headroom): ${need_mb} MB"
if [ "$avail_mb" -lt "$need_mb" ]; then
    printf '  \033[31mNOT ENOUGH SPACE\033[0m: free %s MB more first. Largest candidates (not deleted):\n' $((need_mb - avail_mb))
    du -sh "$HOME/.local/share/Trash" "$HOME/.cache" 2>/dev/null | sed 's/^/    /'
    du -sh /var/cache/apt/archives 2>/dev/null | sed 's/^/    /'
    journalctl --disk-usage 2>/dev/null | sed 's/^/    /'
    command -v snap >/dev/null && snap list --all 2>/dev/null | awk '/disabled/{print "    old snap revision: "$1" "$3}'
else
    ok "enough disk space"
fi
if [ ${#todo[@]} -gt 0 ]; then
    echo "  To do:"
    printf '    %s\n' "${todo[@]}"
    echo "  Recommended Ollama settings for 4 GB VRAM / 8 GB RAM (systemctl edit ollama):"
    echo "    Environment=OLLAMA_MAX_LOADED_MODELS=1 OLLAMA_NUM_PARALLEL=1"
else
    ok "everything is in place: start the GUI with ./run.sh"
fi
