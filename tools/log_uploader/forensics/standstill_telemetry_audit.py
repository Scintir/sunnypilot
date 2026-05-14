"""Investigate gpt-5.5 red flag #1: does evLimiterStandstillSetEmitted increment
ONLY when state==STANDSTILL_PRELAUNCH_SET, or does it leak into other states
(due to sticky _was_in_prelaunch flag)?"""
from __future__ import annotations
from pathlib import Path
from collections import defaultdict, Counter
from openpilot.tools.lib.logreader import LogReader

def audit(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  prev_standstill_emit = 0
  prev_set_emit = 0
  state_at_increment = Counter()
  state_at_set_increment = Counter()
  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception: continue
    for msg in lr:
      if msg.which() != "carStateSP": continue
      sp = msg.carStateSP
      cur_standstill = int(sp.evLimiterStandstillSetEmitted)
      cur_set = int(sp.evLimiterSetEmitted)
      cur_state = int(sp.evLimiterState)
      if cur_standstill != prev_standstill_emit:
        delta = cur_standstill - prev_standstill_emit
        state_at_increment[cur_state] += delta
      if cur_set != prev_set_emit:
        delta = cur_set - prev_set_emit
        state_at_set_increment[cur_state] += delta
      prev_standstill_emit = cur_standstill
      prev_set_emit = cur_set
  return state_at_increment, state_at_set_increment

STATE_NAMES = {0:"IDLE", 1:"STANDSTILL_HOLD", 2:"PRELAUNCH_SET", 3:"SOFT_CAP",
               4:"RECOVERY", 5:"OVERRIDE_SET", 6:"OVERRIDE_RES", 7:"BUS_FAILSAFE", 8:"DISABLED"}

import sys
for rd in sys.argv[1:]:
  print(f"=== {Path(rd).name} ===")
  ss, all_set = audit(Path(rd))
  print("State at standstill_set_emitted increments:")
  for s, n in sorted(ss.items(), key=lambda x: -x[1]):
    print(f"  state {s} ({STATE_NAMES.get(s, '?')}): {n}")
  print(f"  TOTAL standstill_emit increments: {sum(ss.values())}")
  print()
  print("State at all set_emitted increments (sanity):")
  for s, n in sorted(all_set.items(), key=lambda x: -x[1]):
    print(f"  state {s} ({STATE_NAMES.get(s, '?')}): {n}")
  print(f"  TOTAL set_emit increments: {sum(all_set.values())}")
  print()
