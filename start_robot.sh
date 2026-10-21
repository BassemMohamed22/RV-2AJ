#!/usr/bin/env bash
# Start the RV-2AJ system: Ollama (AI model server) + ROS 2 stack (driver, MoveIt,
# RViz, GUI with AI panel). Everything runs offline.
#
#   ./start_robot.sh          real robot on /dev/ttyUSB0
#   ./start_robot.sh sim      simulation (no robot needed)
#   ./start_robot.sh real /dev/ttyUSB1   real robot on another port
#   add "norviz" anywhere to skip RViz (saves RAM)
#
# Stop everything with ./stop_robot.sh
set -eo pipefail
MODE=real
PORT=/dev/ttyUSB0
RVIZ=true
for arg in "$@"; do
    case "$arg" in
        sim) MODE=sim ;;
        real) MODE=real ;;
        norviz) RVIZ=false ;;
        /dev/*) PORT="$arg" ;;
        *) echo "unknown option: $arg"; exit 1 ;;
    esac
done
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$HERE/rv2aj_ws"
LOG="$HOME/.ros/rv2aj_stack.log"
mkdir -p "$HOME/.ros"

if systemctl --user is-active --quiet rv2aj-stack.scope; then
    echo "Already running. Stop it first with ./stop_robot.sh"
    exit 1
fi
if [ "$MODE" = real ] && [ ! -e "$PORT" ]; then
    echo "Robot serial port $PORT not found: plug in the USB-serial adapter (or use: ./start_robot.sh sim)"
    exit 1
fi

# AI model server in its own memory-capped unit (see ai_node._start_ollama).
OLLAMA_BIN="$(command -v ollama || echo "$HOME/.local/ollama/bin/ollama")"
if ! curl -fs --max-time 1 http://127.0.0.1:11434/api/version >/dev/null; then
    systemctl --user reset-failed rv2aj-ollama 2>/dev/null || true
    systemd-run --user --unit=rv2aj-ollama --collect -p MemoryMax=1536M -p MemorySwapMax=256M \
        -p Restart=on-failure -p RestartSec=2 \
        --setenv=OLLAMA_MAX_LOADED_MODELS=1 --setenv=OLLAMA_NUM_PARALLEL=1 --setenv=OLLAMA_HOST=127.0.0.1:11434 \
        --setenv=OLLAMA_FLASH_ATTENTION=1 --setenv=OLLAMA_KV_CACHE_TYPE=q8_0 --setenv=OLLAMA_CONTEXT_LENGTH=4096 \
        --setenv=LLAMA_ARG_CACHE_RAM=0 "$OLLAMA_BIN" serve >/dev/null
fi

FAKE=false
[ "$MODE" = sim ] && FAKE=true
# The ROS stack in its own scope, so it survives if VS Code or the terminal closes.
# A scope left from the last run (still stopping) blocks the name: wait for it.
# systemd keeps the name briefly even after it is inactive ("already loaded"), so wait
# until it is unloaded.
systemctl --user reset-failed rv2aj-stack.scope 2>/dev/null || true
for _ in $(seq 1 30); do
    [ "$(systemctl --user show -p LoadState --value rv2aj-stack.scope 2>/dev/null)" = "not-found" ] && break
    systemctl --user reset-failed rv2aj-stack.scope 2>/dev/null || true
    sleep 1
done
cd "$WS"
setsid nohup systemd-run --user --scope --unit=rv2aj-stack --collect \
    env -u GTK_PATH -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE -u LOCPATH -u GIO_MODULE_DIR \
        -u GTK_IM_MODULE_FILE -u SNAP_LIBRARY_PATH -u GSETTINGS_SCHEMA_DIR -u GTK_EXE_PREFIX HF_HUB_OFFLINE=1 \
    bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec ros2 launch rv2aj_control bringup.launch.py use_fake_hardware:=$FAKE port:=$PORT rviz:=$RVIZ voice:=false" \
    >"$LOG" 2>&1 </dev/null &
disown

echo "Starting RV-2AJ ($MODE) ... the window opens in ~15 s."
echo "  Log:  $LOG"
echo "  Stop: ./stop_robot.sh"
