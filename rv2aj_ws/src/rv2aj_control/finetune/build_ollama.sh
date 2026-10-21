#!/usr/bin/env bash
# Turn the trained LoRA into the Ollama model "rv2aj-qwen" (Q4_K_M, ~2.5 GB).
# Disk-frugal order for a small disk: merge in place (base shards are replaced),
# convert to Q8_0 GGUF, delete the merged HF weights, quantize to Q4_K_M, import.
# Afterwards the HF base is gone: re-download it before training again.
set -euo pipefail
FT="${FT:-$HOME/rv2aj_ft}"
HERE="$(cd "$(dirname "$0")" && pwd)"
LLAMA="$FT/llama.cpp"
OLLAMA_BIN="$(command -v ollama || echo "$HOME/.local/ollama/bin/ollama")"

if ls "$FT/base"/model-merged-*.safetensors >/dev/null 2>&1; then
    echo "== already merged"   # never merge twice
else
    echo "== merge LoRA into the base weights"
    python3 "$HERE/merge_lora.py" --base "$FT/base" --lora "$FT/lora"
fi

echo "== convert to GGUF (Q8_0)"
PYTHONPATH="$LLAMA/gguf-py" python3 "$LLAMA/convert_hf_to_gguf.py" "$FT/base" --outtype q8_0 \
    --outfile "$FT/rv2aj-qwen-q8_0.gguf"
# Check the GGUF really holds the model before the merged weights are deleted.
N=$(PYTHONPATH="$LLAMA/gguf-py" python3 -c "import gguf,sys; print(len(gguf.GGUFReader(sys.argv[1]).tensors))" "$FT/rv2aj-qwen-q8_0.gguf")
echo "GGUF tensors: $N"
if [ "$N" -lt 300 ]; then echo "GGUF is incomplete ($N tensors): keeping $FT/base"; exit 1; fi
rm -rf "$FT/base"

echo "== quantize to Q4_K_M"
"$LLAMA/build/bin/llama-quantize" --allow-requantize "$FT/rv2aj-qwen-q8_0.gguf" "$FT/rv2aj-qwen-q4_k_m.gguf" Q4_K_M
rm -f "$FT/rv2aj-qwen-q8_0.gguf"

echo "== create the Ollama model"
cat > "$FT/Modelfile" <<EOF
FROM $FT/rv2aj-qwen-q4_k_m.gguf
TEMPLATE """{{- range .Messages }}<|im_start|>{{ .Role }}
{{ .Content }}<|im_end|>
{{ end }}<|im_start|>assistant
"""
PARAMETER stop <|im_start|>
PARAMETER stop <|im_end|>
PARAMETER temperature 0
PARAMETER num_ctx 4096
PARAMETER num_gpu 99
EOF
if ! curl -fs --max-time 1 http://127.0.0.1:11434/api/version >/dev/null; then
    OLLAMA_MAX_LOADED_MODELS=1 OLLAMA_NUM_PARALLEL=1 OLLAMA_FLASH_ATTENTION=1 OLLAMA_KV_CACHE_TYPE=q8_0 LLAMA_ARG_CACHE_RAM=0 \
        setsid nohup "$OLLAMA_BIN" serve >"$HOME/.local/ollama/serve.log" 2>&1 < /dev/null &
    sleep 5
fi
"$OLLAMA_BIN" create rv2aj-qwen -f "$FT/Modelfile"
rm -f "$FT/rv2aj-qwen-q4_k_m.gguf"
"$OLLAMA_BIN" list
