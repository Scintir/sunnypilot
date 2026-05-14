"""Two-fold investigation per user feedback:
1. Quantify over-estimation: gas-pedal-pressed periods (driver overriding, EV-only motor delivering
   max real power without triggering ICE this drive) -> peak estPowerW is the system's read on real
   high-load behavior. ICE never engaged this drive -> actual peak motor power < 57 kW (real ICE
   threshold). If estPowerW peaks were >57 kW during these events, that proves over-estimation.
2. Quantify overshoot at cap: when SCC enabled AND estPowerW > 40 kW (the user's set cap), how high
   did estPowerW actually go (raw, pre-cap), and for how long?

User context: EvLimiterMotorCapKW=40 (conservative). Real ICE threshold ~57 kW. Never engaged this
drive."""
from __future__ import annotations
import sys, json
from pathlib import Path
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025

def analyze(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))

  # Gas-pedal-pressed episodes (contiguous frames where gasPressed=True, cruise may or may not be on)
  gas_episodes = []
  current_gas = None  # {start, samples}

  # SCC-engaged + over-cap episodes
  scc_overcap_episodes = []
  current_overcap = None

  # Histograms
  gas_pedal_power_kw = []  # all estPowerW during gas-pedal-pressed
  scc_enabled_power_kw = []  # all estPowerW during cruise enabled (any speed)
  scc_enabled_power_raw_kw = []  # raw (pre-cap) value
  no_overlap_power_kw = []  # neither gas pressed nor cruise (pure coast/brake/etc)

  prev_gas = False
  prev_overcap = False

  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception: continue

    last_v_ego = 0.0
    last_gas = False
    last_brake = False
    last_cruise = False
    last_cluster = 0.0

    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9

      if typ == "carState":
        cs = msg.carState
        last_v_ego = cs.vEgo
        last_gas = bool(cs.gasPressed)
        last_brake = bool(cs.brakePressed)
        last_cruise = bool(cs.cruiseState.enabled)
        last_cluster = cs.vEgoCluster

      elif typ == "carStateSP":
        sp = msg.carStateSP
        v_ego_mph = last_v_ego * MPH
        cluster_mph = last_cluster * MPH
        est = float(sp.estPowerW) / 1000.0
        est_raw = float(sp.estPowerRawW) / 1000.0
        capped = bool(sp.estPowerCapped)
        saturated = bool(sp.estPowerSaturated)
        abasis = float(sp.accelDemand)
        grade = float(sp.evLimiterGradeAccel)
        state = int(sp.evLimiterState)

        # Gas-pedal-pressed episode tracking
        if last_gas:
          if current_gas is None:
            current_gas = {'start_t': t, 'samples': [], 'peak_est': 0.0, 'peak_est_raw': 0.0, 'max_v_ego': 0.0}
          gas_pedal_power_kw.append(est)
          current_gas['samples'].append({
            't': t, 'v_ego_mph': v_ego_mph, 'estPower_kW': est, 'estPowerRaw_kW': est_raw,
            'cluster_mph': cluster_mph, 'cruise_enabled': last_cruise, 'aBasis': abasis, 'grade': grade,
            'capped': capped, 'saturated': saturated, 'state': state,
          })
          current_gas['peak_est'] = max(current_gas['peak_est'], est)
          current_gas['peak_est_raw'] = max(current_gas['peak_est_raw'], est_raw)
          current_gas['max_v_ego'] = max(current_gas['max_v_ego'], v_ego_mph)
        else:
          if current_gas is not None:
            current_gas['end_t'] = t
            current_gas['duration_s'] = t - current_gas['start_t']
            current_gas['n_samples'] = len(current_gas['samples'])
            # keep only first/peak/last
            samps = current_gas['samples']
            peak_idx = max(range(len(samps)), key=lambda i: samps[i]['estPower_kW'])
            current_gas['samples'] = [samps[0], samps[peak_idx], samps[-1]] if len(samps) > 0 else []
            gas_episodes.append(current_gas)
            current_gas = None

        # SCC-engaged samples (any cruise enabled, any speed)
        if last_cruise and not last_brake:  # exclude brake events (drops cruise immediately)
          scc_enabled_power_kw.append(est)
          scc_enabled_power_raw_kw.append(est_raw)

        # Pure coast/brake (neither gas nor cruise) - background baseline
        if not last_gas and not last_cruise and not last_brake:
          no_overlap_power_kw.append(est)

        # Over-cap (>40 kW) AND SCC enabled tracking
        if last_cruise and est > 40.0:
          if current_overcap is None:
            current_overcap = {'start_t': t, 'samples': [], 'peak_est': 0.0, 'peak_est_raw': 0.0,
                               'max_v_ego': 0.0, 'state_set': set()}
          current_overcap['samples'].append({
            't': t, 'v_ego_mph': v_ego_mph, 'estPower_kW': est, 'estPowerRaw_kW': est_raw,
            'aBasis': abasis, 'grade': grade, 'capped': capped, 'state': state,
          })
          current_overcap['peak_est'] = max(current_overcap['peak_est'], est)
          current_overcap['peak_est_raw'] = max(current_overcap['peak_est_raw'], est_raw)
          current_overcap['max_v_ego'] = max(current_overcap['max_v_ego'], v_ego_mph)
          current_overcap['state_set'].add(state)
        else:
          if current_overcap is not None:
            current_overcap['end_t'] = t
            current_overcap['duration_s'] = t - current_overcap['start_t']
            current_overcap['n_samples'] = len(current_overcap['samples'])
            current_overcap['states_visited'] = sorted(current_overcap.pop('state_set'))
            samps = current_overcap['samples']
            peak_idx = max(range(len(samps)), key=lambda i: samps[i]['estPower_kW'])
            current_overcap['samples'] = [samps[0], samps[peak_idx], samps[-1]] if len(samps) > 0 else []
            scc_overcap_episodes.append(current_overcap)
            current_overcap = None

  def pct(v, p):
    if not v: return None
    s = sorted(v)
    return s[min(int(p * len(s)), len(s)-1)]

  return {
    'route': route_dir.name,
    'gas_pedal_episodes': len(gas_episodes),
    'gas_pedal_power_kW_p50': pct(gas_pedal_power_kw, 0.50),
    'gas_pedal_power_kW_p90': pct(gas_pedal_power_kw, 0.90),
    'gas_pedal_power_kW_p99': pct(gas_pedal_power_kw, 0.99),
    'gas_pedal_power_kW_max': max(gas_pedal_power_kw) if gas_pedal_power_kw else 0,
    'gas_pedal_top5_episodes_by_peak': sorted(gas_episodes, key=lambda e: -e['peak_est'])[:5],
    'gas_pedal_total_samples': len(gas_pedal_power_kw),

    'scc_enabled_power_kW_p50': pct(scc_enabled_power_kw, 0.50),
    'scc_enabled_power_kW_p90': pct(scc_enabled_power_kw, 0.90),
    'scc_enabled_power_kW_p99': pct(scc_enabled_power_kw, 0.99),
    'scc_enabled_power_kW_max': max(scc_enabled_power_kw) if scc_enabled_power_kw else 0,
    'scc_enabled_power_raw_kW_p99': pct(scc_enabled_power_raw_kw, 0.99),
    'scc_enabled_power_raw_kW_max': max(scc_enabled_power_raw_kw) if scc_enabled_power_raw_kw else 0,
    'scc_enabled_total_samples': len(scc_enabled_power_kw),

    'overcap_episodes_count': len(scc_overcap_episodes),
    'overcap_top5_episodes_by_peak_raw': sorted(scc_overcap_episodes, key=lambda e: -e['peak_est_raw'])[:5],
    'overcap_top5_episodes_by_duration': sorted(scc_overcap_episodes, key=lambda e: -e['duration_s'])[:5],
    'overcap_total_seconds': sum(e['duration_s'] for e in scc_overcap_episodes),
  }

if __name__ == "__main__":
  for arg in sys.argv[1:]:
    res = analyze(Path(arg))
    print(json.dumps(res, indent=2, default=str))
    print()
