#!/usr/bin/env python3
"""
QLoRA fine-tuning of Qwen3-4B-Instruct-2507 on the generated RV-2AJ data,
sized for a 4 GB GPU / 6 GB RAM laptop:
  - 4-bit NF4 base, LoRA r=16 on all linear layers, gradient checkpointing
  - batch 1 with gradient accumulation, examples sorted into length buckets
  - loss only on the answer: the lm_head runs on answer positions only, so
    no (tokens x 151k vocabulary) logits tensor is ever built

    python3 train_lora.py --base ~/rv2aj_ft/base --data data/train.jsonl --out ~/rv2aj_ft/lora
Stop Ollama and the ROS stack first (GPU memory).
"""

import argparse
import json
import math
import os
import random
import time

os.environ.setdefault("HF_DEACTIVATE_ASYNC_LOAD", "1")  # load tensor by tensor: the parallel loader overflows 4 GB
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import bitsandbytes as bnb_optim  # noqa: E402
import torch  # noqa: E402
from peft import LoraConfig, get_peft_model
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig


def encode(tok, messages, max_len):
    prompt = tok.apply_chat_template(messages[:-1], tokenize=False, add_generation_prompt=True)
    answer = messages[-1]["content"] + "<|im_end|>\n"
    p = tok(prompt, add_special_tokens=False)["input_ids"]
    a = tok(answer, add_special_tokens=False)["input_ids"]
    ids = (p + a)[:max_len]
    n_prompt = min(len(p), len(ids))
    return ids, n_prompt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default=os.path.expanduser("~/rv2aj_ft/base"))
    ap.add_argument("--data", default="data/train.jsonl")
    ap.add_argument("--out", default=os.path.expanduser("~/rv2aj_ft/lora"))
    ap.add_argument("--epochs", type=float, default=1.0)
    ap.add_argument("--lr", type=float, default=2e-4)
    ap.add_argument("--accum", type=int, default=16)
    ap.add_argument("--rank", type=int, default=16)
    ap.add_argument("--max_len", type=int, default=2048)
    ap.add_argument("--limit", type=int, default=0, help="use only the first N examples (smoke test)")
    ap.add_argument("--save_every", type=int, default=100, help="optimizer steps between checkpoints")
    args = ap.parse_args()

    torch.manual_seed(0)
    random.seed(0)
    tok = AutoTokenizer.from_pretrained(args.base)
    rows = [json.loads(line) for line in open(args.data)]
    if args.limit:
        rows = rows[:args.limit]
    data = [encode(tok, r["messages"], args.max_len) for r in rows]
    # Length buckets: shuffle, then sort inside chunks so neighbouring steps have
    # similar lengths (steadier memory), while the order stays mixed.
    random.shuffle(data)
    chunk = 64
    data = [x for i in range(0, len(data), chunk) for x in sorted(data[i:i + chunk], key=lambda d: len(d[0]))]
    total_steps = math.ceil(len(data) * args.epochs / args.accum)
    print(f"{len(data)} examples, {sum(len(d[0]) for d in data)} tokens, {total_steps} optimizer steps", flush=True)

    bnb = BitsAndBytesConfig(load_in_4bit=True, bnb_4bit_quant_type="nf4", bnb_4bit_use_double_quant=True,
                             bnb_4bit_compute_dtype=torch.bfloat16)
    model = AutoModelForCausalLM.from_pretrained(args.base, quantization_config=bnb, dtype=torch.bfloat16,
                                                 device_map={"": 0}, low_cpu_mem_usage=True)
    model.config.use_cache = False
    # prepare_model_for_kbit_training would cast the frozen bf16 embeddings to fp32
    # (+1.5 GB): on 4 GB only gradient checkpointing and input grads are set up.
    for p in model.parameters():
        p.requires_grad_(False)
    model.gradient_checkpointing_enable(gradient_checkpointing_kwargs={"use_reentrant": False})
    model.enable_input_require_grads()
    lora = LoraConfig(r=args.rank, lora_alpha=2 * args.rank, lora_dropout=0.0, bias="none", task_type="CAUSAL_LM",
                      target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"])
    model = get_peft_model(model, lora)
    model.print_trainable_parameters()
    inner = model.base_model.model          # Qwen3ForCausalLM
    body, head = inner.model, inner.lm_head

    params = [p for p in model.parameters() if p.requires_grad]
    opt = bnb_optim.optim.PagedAdamW8bit(params, lr=args.lr, weight_decay=0.0)  # 8-bit states, paged to RAM
    print(f"loaded: {torch.cuda.memory_allocated() / 2**30:.2f} GB on the GPU", flush=True)
    warmup = max(5, total_steps // 20)

    def lr_at(step):
        if step < warmup:
            return args.lr * (step + 1) / warmup
        return args.lr * 0.5 * (1 + math.cos(math.pi * (step - warmup) / max(1, total_steps - warmup)))

    model.train()
    step, micro, t0, run_loss, run_n = 0, 0, time.time(), 0.0, 0
    order = list(range(len(data))) * math.ceil(args.epochs)
    order = order[:int(len(data) * args.epochs)]
    for idx in order:
        ids, n_prompt = data[idx]
        x = torch.tensor([ids], device="cuda")
        with torch.autograd.graph.save_on_cpu(pin_memory=True):  # saved activations wait in RAM, not on the GPU
            hidden = body(input_ids=x).last_hidden_state[0]      # (T, H)
        pos = torch.arange(n_prompt - 1, len(ids) - 1, device="cuda")   # positions that predict answer tokens
        logits = head(hidden[pos]).float()
        target = x[0, n_prompt:]
        loss = torch.nn.functional.cross_entropy(logits, target)
        (loss / args.accum).backward()
        run_loss += loss.item()
        run_n += 1
        micro += 1
        if micro % args.accum == 0:
            for g in opt.param_groups:
                g["lr"] = lr_at(step)
            torch.nn.utils.clip_grad_norm_(params, 1.0)
            opt.step()
            opt.zero_grad(set_to_none=True)
            step += 1
            if step % 5 == 0 or step == 1:
                el = time.time() - t0
                eta = el / step * (total_steps - step)
                print(f"step {step}/{total_steps} loss {run_loss / run_n:.4f} lr {lr_at(step):.2e} "
                      f"{el / 60:.1f} min, eta {eta / 60:.0f} min, mem {torch.cuda.max_memory_allocated() / 2**30:.2f} GB",
                      flush=True)
                run_loss, run_n = 0.0, 0
            if step % args.save_every == 0:
                model.save_pretrained(args.out)
    model.save_pretrained(args.out)
    tok.save_pretrained(args.out)
    print("saved", args.out, flush=True)


if __name__ == "__main__":
    main()
