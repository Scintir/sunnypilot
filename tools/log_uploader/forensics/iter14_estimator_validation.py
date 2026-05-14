"""Now that we have estPowerInstantW (truly raw, no LP, no cap), compare gas-pedal peaks
across drives 18 (iter13) and 85+86 (iter14). User's empirical real ICE threshold ~57 kW.
ICE never engaged any drive -> actual peak motor power < 57 kW each drive."""
from __future__ import annotations
from pathlib import Path
import sys
from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025

def analyze(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  gas_episodes = []
  cur = None
  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception: continue
    last_v = 0.0; last_gas = False; last_cluster = 0.0
    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9
      if typ == "carState":
        cs = msg.carState
        last_v = cs.vEgo
        last_gas = bool(cs.gasPressed)
        last_cluster = cs.vEgoCluster
      elif typ == "carStateSP":
        sp = msg.carStateSP
        v_mph = last_v * MPH
        inst_kw = float(sp.estPowerInstantW) / 1000.0
        ctl_kw = float(sp.estPowerControlW) / 1000.0
        hud_kw = float(sp.estPowerW) / 1000.0
        if last_gas:
          if cur is None: cur = {'start_t': t, 'peak_inst': 0, 'peak_ctl': 0, 'peak_hud': 0, 'max_v': 0}
          cur['peak_inst'] = max(cur['peak_inst'], inst_kw)
          cur['peak_ctl'] = max(cur['peak_ctl'], ctl_kw)
          cur['peak_hud'] = max(cur['peak_hud'], hud_kw)
          cur['max_v'] = max(cur['max_v'], v_mph)
        else:
          if cur is not None:
            cur['end_t'] = t
            cur['duration_s'] = t - cur['start_t']
            gas_episodes.append(cur)
            cur = None
  # top 3 by peak instant
  return sorted(gas_episodes, key=lambda e: -e['peak_inst'])[:5]

for route in sys.argv[1:]:
  rd = Path(route)
  eps = analyze(rd)
  print(f"=== {rd.name} ===")
  print(f"Top 5 gas-pedal episodes by peak estPowerInstantW:")
  for e in eps:
    print(f"  dur={e['duration_s']:5.1f}s  max_vEgo={e['max_v']:.1f}mph  "
          f"peak_INSTANT={e['peak_inst']:.1f}kW  peak_CTL={e['peak_ctl']:.1f}kW  peak_HUD={e['peak_hud']:.1f}kW")
  print()
