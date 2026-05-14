"""Direct query: find frames where evLimiterState==4 AND estPowerControlW >= 40 kW."""
from pathlib import Path
from openpilot.tools.lib.logreader import LogReader
import sys

def find(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  hits = []
  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception: continue
    for msg in lr:
      if msg.which() != "carStateSP": continue
      sp = msg.carStateSP
      state = int(sp.evLimiterState)
      ctl_kw = float(sp.estPowerControlW) / 1000.0
      if state == 4 and ctl_kw >= 40.0:
        t = msg.logMonoTime / 1e9
        hits.append({
          't': t,
          'state': state,
          'state_candidate': int(sp.evLimiterStateCandidateBeforeGuard),
          'estPowerW': float(sp.estPowerW)/1000,
          'estPowerControlW': ctl_kw,
          'estPowerInstantW': float(sp.estPowerInstantW)/1000,
          'guard_yield': str(sp.evLimiterPowerGuardYieldReason),
          'guard_lockout': bool(sp.evLimiterPowerGuardLockoutActive),
          'capped_sustain': int(sp.evLimiterPowerCappedSustainFrames),
          'near_budget_sustain': int(sp.evLimiterPowerNearBudgetSustainFrames),
          'yield_events': int(sp.evLimiterRecoveryYieldEvents),
        })
  return hits

if __name__ == "__main__":
  route = Path(sys.argv[1])
  hits = find(route)
  print(f"=== {route.name}: {len(hits)} frames where state=RECOVERY AND estPowerControlW >= 40 kW ===")
  if not hits:
    print("(none — iter14 guard fully effective)")
    sys.exit(0)
  print(f"First 30:")
  print(f"  t          cand|state  CTL  INST  HUD  yield  lock  cap_s  nb_s  yld_ev")
  for h in hits[:30]:
    print(f"  {h['t']:12.3f}   {h['state_candidate']}|{h['state']}   "
          f"{h['estPowerControlW']:5.1f} {h['estPowerInstantW']:5.1f} {h['estPowerW']:5.1f}  "
          f"{h['guard_yield']:>9}  {'L' if h['guard_lockout'] else '.':>4}  "
          f"{h['capped_sustain']:3d}   {h['near_budget_sustain']:3d}  {h['yield_events']}")
