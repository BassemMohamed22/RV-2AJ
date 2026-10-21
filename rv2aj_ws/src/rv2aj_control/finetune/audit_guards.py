#!/usr/bin/env python3
"""Run the intent guards and the repairs over correct labels: every doubt is a
place where the guard would needlessly ask, every repair a place where it
would corrupt a right answer. Usage: python3 audit_guards.py data/train.jsonl"""
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rv2aj_control import ai_brain as b  # noqa: E402

rows = [json.loads(line) for line in open(sys.argv[1])]
found = collections.defaultdict(list)
total = 0
for r in rows:
    m = r["meta"]
    if not m["commands"]:
        continue
    total += 1
    text = b.fix_transcript(m["request"])
    prev = {"request": m["history"][0]["request"], "commands": m["history"][0]["commands"]} if m["history"] else None
    guard = b.intent_check(text, m["commands"], prev)
    if guard:
        found["doubt " + m["gen"]].append((m["request"], m["commands"], guard))
    c2, ch = b.repair_directions(text, m["commands"])
    _c3, ch2 = b.repair_numbers(text, c2)
    if ch or ch2:
        found["REPAIR " + m["gen"]].append((m["request"], m["commands"], str(ch + ch2)))
doubts = sum(len(v) for k, v in found.items() if k.startswith("doubt"))
repairs = sum(len(v) for k, v in found.items() if k.startswith("REPAIR"))
print(f"{total} labelled plans: {doubts} doubted ({100 * doubts / total:.1f}%), {repairs} wrongly repaired")
for k, v in sorted(found.items(), key=lambda kv: -len(kv[1])):
    print(f"\n== {k} {len(v)}")
    for x in v[:int(sys.argv[2]) if len(sys.argv) > 2 else 4]:
        print("  ", x[0][:62], x[1], "|", x[2][:90])
