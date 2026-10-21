#!/usr/bin/env python3
"""
Measure how well the local model turns requests into commands.

  source install/setup.bash
  python3 src/rv2aj_control/test/ai_eval.py [--model qwen2.5:3b] [--k 8] [--show]

The requests in ai_eval_set.json are worded differently from the teaching
examples in config/ai_examples.json, so this measures understanding, not
recall. A request counts as correct only if the command list matches one of
the expected lists exactly (after normalization). It also reports how many
plans would have passed the validator, i.e. what could actually have reached
the robot after the operator's confirmation.
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from rv2aj_control import ai_brain as b  # noqa: E402

EVAL_POSE = {"J1": -30.0, "J2": 20.0, "J3": 90.0, "J4": 0.0, "J5": 40.0, "J6": 0.0}


def eval_world():
    return b.World(
        joints=EVAL_POSE, connected=True, servo_on=True, gripper="open",
        points={"P1": {"joints": [-20, 10, 95, 0, 35, 0], "gripper": ""},
                "P2": {"joints": [-40, 25, 85, 0, 40, 0], "gripper": "closed"},
                "P3": {"joints": [-10, 15, 90, 0, 30, 10], "gripper": "open"}},
        home=[-30.0, 0.0, 100.0, 0.0, 30.0, 0.0],
        skills={"wave": ["MOVE_JOINT_REL J6 +30", "MOVE_JOINT_REL J6 -60", "MOVE_JOINT_REL J6 +30"]},
    )


def canon(commands):
    flat = []
    for c in commands:
        if "|" in c and not c.strip().upper().startswith("DEFINE_SKILL"):
            flat.extend(b.split_skill_body(c))
        else:
            flat.append(c)
    out = []
    for c in flat:
        c = b.normalize(c).replace(" = ", "=")
        if c.startswith("DEFINE_SKILL"):
            name, _, body = c.partition(":")
            c = name + ": " + " | ".join(b.normalize(x) for x in b.split_skill_body(body))
        out.append(c)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="qwen3:4b")
    ap.add_argument("--k", type=int, default=8, help="examples shown per request")
    ap.add_argument("--show", action="store_true", help="print every answer, not only misses")
    ap.add_argument("--set", default="ai_eval_set.json", help="eval file in test/")
    ap.add_argument("--compact", action="store_true", help="fine-tuned model format (short prompt, no examples)")
    args = ap.parse_args()

    with open(os.path.join(os.path.dirname(HERE), "config", "ai_examples.json")) as f:
        memory = b.Memory("/tmp/rv2aj_ai_eval_memory.json", json.load(f))
    memory.skills, memory.learned = {}, []
    with open(os.path.join(HERE, args.set)) as f:
        cases = json.load(f)

    llm = b.Ollama(model=args.model)
    llm.warm_up()
    world = eval_world()
    correct = blocked = asked_right = 0
    dangerous = []
    times = []
    for case in cases:
        messages = b.build_messages(case["request"], world, b.DEFAULT_LIMITS, memory, k_examples=args.k,
                                    compact=args.compact)
        t = time.monotonic()
        try:
            _raw, parsed = llm.chat(messages)
            cmds, _ = b.repair_directions(case["request"], parsed["commands"])
            cmds, _ = b.repair_numbers(case["request"], cmds)  # the same repairs ai_node applies
            got = canon(cmds)
        except b.LLMError as e:
            got, parsed = None, {"say": str(e)}
        times.append(time.monotonic() - t)
        ok = got is not None and any(got == canon(exp) for exp in case["expect"])
        correct += ok
        # Wrong plans: blocked by the validator, shown as a guess ("ben şöyle
        # anladım, doğru mu?") because the guards doubt it, or shown as certain.
        valid = bool(got) and b.validate(got, world).ok
        doubted = valid and b.intent_check(case["request"], got) is not None
        if ok and doubted:
            asked_right += 1
        if not ok:
            if valid and not doubted:
                dangerous.append(case["request"])
            else:
                blocked += 1
        if not ok or args.show:
            print(f"{'ok  ' if ok else 'MISS'} {case['request']!r}\n     got {got}  want {case['expect'][0]}"
                  f"\n     say {parsed.get('say', '')!r}")
    n = len(cases)
    times.sort()
    print(f"\n{args.set} {args.model}, k={args.k}: {correct}/{n} correct ({100 * correct / n:.0f}%), "
          f"median {times[n // 2]:.2f} s, max {times[-1]:.2f} s")
    print(f"  right but shown as a guess (operator asked): {asked_right}")
    print(f"  wrong but blocked or shown as a guess: {blocked}")
    print(f"  wrong and shown as certain: {len(dangerous)} {dangerous}")


if __name__ == "__main__":
    main()
