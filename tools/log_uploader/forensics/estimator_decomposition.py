#!/usr/bin/env python3
# ruff: noqa: E501, E701, E702
"""Power-estimator decomposition: per-speed-bucket distribution of the published HUD power vs an
alternative aEgo+grade estimator with an efficiency map and aux baseline (drive 2026-09-24 analysis).

Usage: python3 estimator_decomposition.py <route_dir_prefix>
"""
import sys
from pathlib import Path
from collections import defaultdict
sys.path.insert(0, 'tools/log_uploader/forensics')
from openpilot.tools.lib.logreader import LogReader
MPH=2.23694; MASS=1950; g=9.81
route = Path(sys.argv[1])
segs = sorted(route.parent.glob(route.name + '--*'), key=lambda p: int(p.name.rsplit('--',1)[1]))
t0=None; cs=None; rows=[]; kmsg=[]
FAULTS=(644.4, 986.4, 1773.8)
def eta(v):
  mph=v*MPH
  return 0.72 + (0.90-0.72)*min(1.0, mph/45.0)
def road(v): return 0.011*MASS*g*v + 0.5*1.225*0.75*v**3
aego_f=0.0; alt_f=None; hud_f=None
for seg in segs:
  for m in LogReader(str(seg/'rlog.zst')):
    t=m.logMonoTime/1e9
    if t0 is None: t0=t
    w=m.which()
    if w=='androidLog' and any(abs(t-t0-f)<20 for f in FAULTS):
      kmsg.append((t-t0, m.androidLog.message[:170]))
    if w=='carState': cs=(m.carState.vEgo, m.carState.aEgo)
    elif w=='carStateSP' and cs:
      s=m.carStateSP; v,a=cs
      aego_f += 0.05*(a-aego_f)   # ~0.2 s LP at 100 Hz
      p_wheel = MASS*v*max(0.0, aego_f + s.evLimiterGradeAccel) + road(v)
      alt = p_wheel/eta(v) + 2500.0
      alt_f = alt if alt_f is None else alt_f + 0.02*(alt-alt_f)  # symmetric 0.5 s
      rows.append((t-t0, v, s.estPowerInstantW, s.estPowerW, alt, alt_f))
def pct(v,p):
  v=sorted(v); return v[min(len(v)-1,int(p*len(v)))] if v else float('nan')
b=defaultdict(list)
for r in rows:
  if r[1]*MPH<3: continue
  b[int(r[1]*MPH//10*10)].append(r)
print("mph     n   current HUD p50/p90/p99   alt(sym 0.5s) p50/p90/p99   [kW]")
for k in sorted(b):
  L=b[k]
  print(f"{k:3d} {len(L):6d}   {pct([x[3] for x in L],.5)/1e3:5.1f}/{pct([x[3] for x in L],.9)/1e3:5.1f}/{pct([x[3] for x in L],.99)/1e3:5.1f}      {pct([x[5] for x in L],.5)/1e3:5.1f}/{pct([x[5] for x in L],.9)/1e3:5.1f}/{pct([x[5] for x in L],.99)/1e3:5.1f}")
def ts(t): return f"{int(t//60):02d}:{t%60:04.1f}"
print("\nkernel/android log near faults:", len(kmsg))
for r in kmsg[:40]: print(ts(r[0]), r[1])
