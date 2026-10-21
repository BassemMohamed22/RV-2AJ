#!/usr/bin/env python3
"""
Merge a PEFT LoRA adapter into bf16 safetensors shards, one tensor at a time
(W += scale * B @ A), so it runs in a few hundred MB of RAM. Each merged shard
is written in ~1 GB pieces and the original shard is deleted (the disk here is small).

    python3 merge_lora.py --base ~/rv2aj_ft/base --lora ~/rv2aj_ft/lora
"""

import argparse
import json
import os

import torch
from safetensors import safe_open
from safetensors.torch import save_file


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--lora", required=True)
    args = ap.parse_args()
    cfg = json.load(open(os.path.join(args.lora, "adapter_config.json")))
    scale = cfg["lora_alpha"] / cfg["r"]
    lora = {}
    with safe_open(os.path.join(args.lora, "adapter_model.safetensors"), "pt") as f:
        for k in f.keys():
            lora[k] = f.get_tensor(k)
    # "base_model.model.model.layers.0.self_attn.q_proj.lora_A.weight" -> "model.layers.0.self_attn.q_proj.weight"
    pairs = {}
    for k in lora:
        if ".lora_A." in k:
            target = k.replace("base_model.model.", "", 1).replace(".lora_A.weight", ".weight")
            pairs[target] = (lora[k], lora[k.replace(".lora_A.", ".lora_B.")])
    index = json.load(open(os.path.join(args.base, "model.safetensors.index.json")))
    missing = [k for k in pairs if k not in index["weight_map"]]
    if missing or not pairs:  # check before touching anything: the merge deletes the original shards
        raise SystemExit(f"LoRA weights do not match the base model: {missing[:3]} (of {len(pairs)})")
    shards = sorted(set(index["weight_map"].values()))
    done, part, buf, size, weight_map = 0, 0, {}, 0, {}

    def flush():
        nonlocal part, buf, size
        if not buf:
            return
        part += 1
        name = f"model-merged-{part:05d}.safetensors"  # the GGUF converter only reads model*.safetensors
        save_file(buf, os.path.join(args.base, name), metadata={"format": "pt"})
        for k in buf:
            weight_map[k] = name
        buf, size = {}, 0

    for shard in shards:
        path = os.path.join(args.base, shard)
        with safe_open(path, "pt") as f:
            for k in f.keys():
                t = f.get_tensor(k)
                if k in pairs:
                    a, b = pairs[k]
                    t = (t.float() + scale * (b.float() @ a.float())).to(t.dtype)
                    done += 1
                buf[k] = t.clone()
                size += t.numel() * t.element_size()
                if size > 1 << 30:  # write ~1 GB pieces: little RAM needed
                    flush()
        flush()
        os.remove(path)  # the disk is small: the original shard goes once it is merged
        print("merged", shard, flush=True)
    index["weight_map"] = weight_map
    json.dump(index, open(os.path.join(args.base, "model.safetensors.index.json"), "w"), indent=2)
    print(f"{done} of {len(pairs)} LoRA pairs merged (scale {scale:g})")
    if done != len(pairs):
        raise SystemExit("some LoRA weights were not found in the base model")


if __name__ == "__main__":
    main()
