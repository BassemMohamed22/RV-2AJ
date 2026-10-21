"""Append-only log of everything the AI layer does: transcripts, raw model
output, every validator decision, confirmations, execution replies, stops."""

import json
import os
import threading
from datetime import datetime

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ai_control_log.txt")
_lock = threading.Lock()


def log_event(event, **data):
    line = f"{datetime.now():%Y-%m-%d %H:%M:%S.%f}"[:-3] + f"  {event:<16} " + json.dumps(
        data, ensure_ascii=False, default=str)
    with _lock:
        try:
            with open(LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except OSError:
            pass  # logging must never break control; the GUI log still shows events


def log_plan(plan, source):
    log_event("VALIDATOR", source=source, ok=plan.ok, reason=plan.reason,
              steps=[{"model": s.source, "command": s.command,
                      "kind": s.kind.value if s.kind else None,
                      "ok": s.ok, "reason": s.reason} for s in plan.steps])
