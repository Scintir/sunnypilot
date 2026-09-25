#!/usr/bin/env python3
# ruff: noqa: E701, E702
"""hardwared stall scan: deviceState gap table, what else stalled, procLog CPU/iowait for
hardwared across each gap, and every `hardwared slow tick` event (StallWatchdog output).

Usage: python3 hardwared_stall_scan.py <route_dir_prefix>   (e.g. .log_routes/log_routes/00000015--99c9d4c0d0)
"""
import sys
import bisect
import json
from pathlib import Path
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader

route = Path(sys.argv[1])
segs = sorted(route.parent.glob(route.name + '--*'), key=lambda p: int(p.name.rsplit('--', 1)[1]))
WATCH = ['deviceState', 'sendcan', 'carState', 'can', 'modelV2', 'pandaStates', 'managerState', 'driverStateV2', 'controlsState']
last = {}; gaps = defaultdict(list); t0 = None; ticks = []; procs = []; slow = []
for seg in segs:
  rl = seg / 'rlog.zst'
  if not rl.exists(): rl = seg / 'rlog'
  if not rl.exists(): continue
  for m in LogReader(str(rl)):
    t = m.logMonoTime / 1e9
    if t0 is None: t0 = t
    w = m.which()
    if w in ('logMessage', 'errorLogMessage'):
      s = m.logMessage if w == 'logMessage' else m.errorLogMessage
      if 'hardwared slow tick' in s:
        try: d = json.loads(s); msg = d.get('msg', {})
        except Exception: msg = s
        slow.append((t - t0, msg))
      continue
    if w == 'procLog':
      p = m.procLog; ct = p.cpuTimes
      iow = sum(c.iowait for c in ct); tot = sum(c.user + c.nice + c.system + c.idle + c.iowait + c.irq + c.softirq for c in ct)
      hw = [(pr.cpuUser + pr.cpuSystem, pr.state) for pr in p.procs if any('hardwared' in c for c in pr.cmdline)]
      dst = [pr.name for pr in p.procs if pr.state in (b'D', 'D')]
      procs.append((t - t0, iow, tot, hw[0] if hw else (0., '?'), dst))
      continue
    if w in WATCH:
      if w in last:
        dt = t - last[w]
        if w == 'deviceState': ticks.append(dt)
        if dt > 0.6: gaps[w].append((last[w] - t0, t - t0, dt))
      last[w] = t

def ts(t): return f"{int(t//60):02d}:{t%60:05.1f}"
ticks.sort(); n = len(ticks)
if n:
  slow_n = sum(1 for x in ticks if x > 0.7)
  pct = f"p50 {ticks[n//2]:.3f} p99 {ticks[int(n*.99)]:.3f} p99.9 {ticks[int(n*.999)]:.3f} max {ticks[-1]:.2f}"
  print(f"deviceState tick spacing: n={n} {pct}  (>0.7s: {slow_n})")
print("\n== gaps >0.6s per service ==")
for w in WATCH:
  if gaps[w]: print(f"  {w}: " + ', '.join(f"{ts(a)}({dt:.1f}s)" for a, b, dt in gaps[w][:20]))
pl = [x[0] for x in procs]
print("\n== procLog across deviceState gaps: iowait%, hardwared cpu per window, D-state procs ==")
for a, b, dt in gaps['deviceState']:
  i = bisect.bisect_left(pl, a - 4); j = bisect.bisect_right(pl, b + 4)
  print(f"--- gap {ts(a)}->{ts(b)} ({dt:.1f}s)")
  prev = None
  for row in procs[i:j]:
    if prev:
      pct = 100 * (row[1] - prev[1]) / max(1, row[2] - prev[2])
      print(f"   {ts(row[0])} iowait {pct:4.1f}%  hardwared cpu +{row[3][0]-prev[3][0]:.2f}s  D: {row[4][:5]}")
    prev = row
print(f"\n== hardwared slow tick events: {len(slow)} ==")
for t, msg in slow:
  if isinstance(msg, dict):
    print(f"--- {ts(t)} period={msg.get('period')} started={msg.get('started')} slowest={msg.get('slowest_phases')}")
    st = msg.get('stacks') or ''
    if st: print('   ' + st.replace('\n', '\n   '))
  else:
    print(f"--- {ts(t)} {str(msg)[:400]}")
