#!/usr/bin/env bash
# VS Code is installed as a snap here, and its terminal leaks snap GTK paths
# that crash PyQt5 (glibc symbol mismatch). Strip them before launching.
cd "$(dirname "$0")"

# AI Control needs the local Ollama server. It is installed per-user in
# ~/.local/ollama (no sudo); start it if it is not already running. Limits
# keep the 4 GB GPU / 8 GB RAM machine to one model and one request at a time.
OLLAMA_BIN="$HOME/.local/ollama/bin/ollama"
command -v ollama >/dev/null && OLLAMA_BIN="$(command -v ollama)"
if [ -x "$OLLAMA_BIN" ] && ! curl -fs --max-time 1 http://127.0.0.1:11434/api/version >/dev/null; then
    if command -v systemd-run >/dev/null; then
        # LLAMA_ARG_CACHE_RAM=0: the runner's RAM prompt cache (8 GB by default) grew until the
        # OOM killer struck. The unit's memory cap is a second safety net: only Ollama restarts.
        systemctl --user reset-failed rv2aj-ollama 2>/dev/null
        systemd-run --user --unit=rv2aj-ollama --collect -p MemoryMax=1536M -p MemorySwapMax=256M \
            -p Restart=on-failure -p RestartSec=2 \
            --setenv=OLLAMA_MAX_LOADED_MODELS=1 --setenv=OLLAMA_NUM_PARALLEL=1 --setenv=OLLAMA_HOST=127.0.0.1:11434 \
            --setenv=OLLAMA_FLASH_ATTENTION=1 --setenv=OLLAMA_KV_CACHE_TYPE=q8_0 --setenv=OLLAMA_CONTEXT_LENGTH=4096 --setenv=LLAMA_ARG_CACHE_RAM=0 \
            "$OLLAMA_BIN" serve >/dev/null
    else
        OLLAMA_MAX_LOADED_MODELS=1 OLLAMA_NUM_PARALLEL=1 OLLAMA_HOST=127.0.0.1:11434 OLLAMA_FLASH_ATTENTION=1 \
            OLLAMA_KV_CACHE_TYPE=q8_0 OLLAMA_CONTEXT_LENGTH=4096 LLAMA_ARG_CACHE_RAM=0 \
            nohup "$OLLAMA_BIN" serve >"$HOME/.local/ollama/serve.log" 2>&1 &
    fi
fi

exec env -u GTK_PATH -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE \
    -u LOCPATH -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE -u SNAP_LIBRARY_PATH \
    -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX \
    /usr/bin/python3 main.py "$@"
