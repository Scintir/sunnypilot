#!/usr/bin/env python3
# ruff: noqa: E701, E702
"""Comm-fault scan: which services went not-alive / off-frequency at each commIssue,
deviceState/CAN gaps, process deaths and alert transitions for one route.

Usage: python3 comm_fault_scan.py <route_dir_prefix>   (e.g. .log_routes/x/00000015--99c9d4c0d0)
"""
import sys
from pathlib import Path
sys.path.insert(0, 'tools/log_uploader/forensics')
from openpilot.tools.lib.logreader import LogReader

route = Path(sys.argv[1])
segs = sorted(route.parent.glob(route.name + '--*'), key=lambda p: int(p.name.rsplit('--',1)[1]))
t0 = None
events = []          # (t, seg, set of event names)
comm_logs = []       # (t, seg, dict)
can_err = []         # (t, canErrorCounter)
proc_deaths = []
alerts = []
prev_ev = set()
last_can_err = None
for seg in segs:
  rl = seg / 'rlog.zst'
  if not rl.exists(): rl = seg / 'rlog'
  if not rl.exists(): continue
  for m in LogReader(str(rl)):
    t = m.logMonoTime / 1e9
    if t0 is None: t0 = t
    w = m.which()
    if w == 'onroadEvents':
      names = {str(e.name) for e in m.onroadEvents}
      new = names - prev_ev
      if new: events.append((t - t0, seg.name.rsplit('--',1)[1], sorted(new)))
      prev_ev = names
    elif w == 'logMessage' or w == 'errorLogMessage':
      s = m.logMessage if w == 'logMessage' else m.errorLogMessage
      if 'commIssue' in s or 'not_alive' in s or 'died' in s.lower() or 'timeout' in s.lower() and 'can' in s.lower():
        comm_logs.append((t - t0, seg.name.rsplit('--',1)[1], s[:400]))
    elif w == 'carState':
      c = m.carState.canErrorCounter
      if last_can_err is not None and c != last_can_err:
        can_err.append((t - t0, c))
      last_can_err = c
    elif w == 'selfdriveState':
      a = str(m.selfdriveState.alertText1)
      if a and (not alerts or alerts[-1][1] != a):
        alerts.append((t - t0, a, str(m.selfdriveState.alertText2)))
    elif w == 'managerState':
      for p in m.managerState.processes:
        if not p.running and p.shouldBeRunning:
          proc_deaths.append((t - t0, p.name, p.exitCode))

def ts(t): return f"{int(t//60):02d}:{t%60:05.1f}"
print("== onroadEvents newly raised (comm/can related) ==")
for t, seg, new in events:
  if any(k in n for n in new for k in ('comm','can','process','Mismatch','controlsMismatch','Fault','fault')):
    print(ts(t), 'seg', seg, new)
print("\n== cloudlog commIssue / death messages ==")
for t, seg, s in comm_logs[:60]:
  print(ts(t), 'seg', seg, s)
print(f"... total {len(comm_logs)}")
print("\n== canErrorCounter changes ==", len(can_err))
for t, c in can_err[:30]: print(ts(t), c)
print("\n== managerState not-running-but-should ==")
seen = set()
for t, n, e in proc_deaths:
  if n not in seen: print(ts(t), n, e); seen.add(n)
print("\n== alerts (distinct transitions) ==")
for t, a, b in alerts:
  print(ts(t), a, '|', b)
