#!/usr/bin/env python3
"""
Score an Ollama model on the held-out generated test set.

    python3 eval_model.py --model qwen3:4b              # current setup: long prompt + examples
    python3 eval_model.py --model rv2aj-qwen --compact  # fine-tuned model: compact prompt

Commands must match exactly (after normalising). For knowledge/chat answers
(no commands) the numbers in the reference answer must appear in "say".
The same intent guards and repairs as ai_node are applied before scoring.
"""

import argparse
import collections
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from rv2aj_control import ai_brain as b  # noqa: E402
from gen_dataset import SKILL_BANK  # noqa: E402


def canon(cmds):
    out = []
    for c in cmds:
        c = b.normalize(c).replace(" = ", "=")
        if c.startswith("DEFINE_SKILL"):
            name, _, body = c.partition(":")
            c = name + ": " + " | ".join(b.normalize(x) for x in b.split_skill_body(body))
        out.append(c)
    return out


def world_of(meta):
    return b.World(joints={"J1": -30.0, "J2": 20.0, "J3": 90.0, "J4": 0.0, "J5": 40.0, "J6": 0.0},
                   connected=True, servo_on=True, gripper="open",
                   points={n: {"joints": [-30, 10, 95, 0, 35, 0], "gripper": ""} for n in meta["points"]},
                   home=[-30.0, 0.0, 100.0, 0.0, 30.0, 0.0], skills={k: SKILL_BANK[k] for k in meta["skills"]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="qwen3:4b")
    ap.add_argument("--data", default=os.path.join(HERE, "data", "test.jsonl"))
    ap.add_argument("--compact", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--show", type=int, default=25, help="misses to print")
    args = ap.parse_args()
    rows = [json.loads(line) for line in open(args.data)]
    if args.limit:
        rows = rows[:args.limit]
    memory = None
    if not args.compact:
        with open(os.path.join(os.path.dirname(HERE), "config", "ai_examples.json")) as f:
            memory = b.Memory("/tmp/rv2aj_eval_mem.json", json.load(f))
        memory.skills, memory.learned = {}, []
    llm = b.Ollama(model=args.model)
    llm.warm_up()
    per = collections.defaultdict(lambda: [0, 0])
    misses, times, turkish_say = [], [], 0
    for r in rows:
        m = r["meta"]
        want = canon(m["commands"])
        ref_say = json.loads(r["messages"][-1]["content"])["say"]
        world = world_of(m)
        history = m["history"]
        if args.compact:
            messages = r["messages"][:-1]
        else:
            messages = b.build_messages(m["request"], world, b.DEFAULT_LIMITS, memory, history=history)
        t = time.monotonic()
        try:
            _raw, parsed = llm.chat(messages)
            cmds, say = parsed["commands"], parsed["say"]
        except b.LLMError as e:
            cmds, say = ["<error>"], str(e)
        times.append(time.monotonic() - t)
        prev = {"request": history[0]["request"], "commands": history[0]["commands"]} if history else None
        if cmds and cmds != ["<error>"]:
            cmds, _ = b.repair_directions(m["request"], cmds)
            cmds, _ = b.repair_numbers(m["request"], cmds)
        got = canon(cmds) if cmds != ["<error>"] else cmds
        if want:
            ok = got == want
        else:
            nums = re.findall(r"\d+(?:[.,]\d+)?", ref_say)
            ok = not got and all(n in say for n in nums)
        turkish_say += b.is_turkish(say) or not say
        per[m["gen"]][0] += ok
        per[m["gen"]][1] += 1
        if not ok:
            misses.append((m["gen"], m["request"], want, got, say))
    n = len(rows)
    correct = sum(v[0] for v in per.values())
    times.sort()
    print(f"\n{args.model} ({'compact' if args.compact else 'long prompt + examples'}): {correct}/{n} = "
          f"{100 * correct / n:.1f}% | Turkish replies {100 * turkish_say / n:.0f}% | "
          f"median {times[n // 2]:.2f} s, p90 {times[int(n * .9)]:.2f} s")
    for g, (c, t) in sorted(per.items(), key=lambda kv: kv[1][0] / kv[1][1]):
        print(f"  {g:18} {c}/{t}")
    for g, req, want, got, say in misses[:args.show]:
        print(f"MISS {g}: {req!r}\n     want {want}\n     got  {got}  say {say[:90]!r}")


if __name__ == "__main__":
    main()
