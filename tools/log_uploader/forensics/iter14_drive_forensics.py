"""iter14 v2 drive forensics — extends drive 18+19 script with iter14 telemetry.
Reports per-drive: state distribution, block reasons, power-guard yield events,
estPowerInstantW/ControlW/W distributions at speed buckets, RECOVERY-while-capped
runs (the iter13 bug), cruise-off events with prior-1s SET activity."""
from __future__ import annotations
import sys, json
from pathlib import Path
from collections import defaultdict, Counter, deque
from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025
STATE_NAMES = {0:"IDLE", 1:"STANDSTILL_HOLD", 2:"PRELAUNCH_SET", 3:"SOFT_CAP",
               4:"RECOVERY", 5:"OVERRIDE_SET", 6:"OVERRIDE_RES", 7:"BUS_FAULT", 8:"DISABLED"}

def analyze(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  if not segs:
    return {'route': route_dir.name, 'error': 'no segments'}

  state_dist = Counter()
  block_dist = Counter()
  yield_reason_dist = Counter()
  guard_lockout_frames = 0

  # cumulative counters (snapshot at end)
  set_emitted_max = 0
  set_dropped_max = 0
  set_requested_max = 0
  all_btn_max = 0
  ack_max = 0
  standstill_entered_max = 0
  standstill_set_emitted_max = 0
  recovery_yield_events_max = 0
  recovery_lockouts_entered_max = 0
  suspected_cancel_max = 0

  fault_inhibit_seen = False

  ev_mode_assumed_true = 0
  ev_mode_param_read_ok_true = 0
  total_csp = 0

  # power buckets by 5-mph speed (estPowerControlW used for state arbiter — the relevant signal)
  control_power_by_speed = defaultdict(list)
  instant_power_by_speed = defaultdict(list)
  hud_power_by_speed = defaultdict(list)

  # RECOVERY-while-capped runs (the iter13 bug we fixed)
  recovery_capped_runs = []
  curr_run_start = None
  curr_run_frames = 0

  # cruise-off events
  cruise_prev = None
  cruise_off_events = []
  recent_set_emits = deque()  # (frame_t, delta) for prior-1s lookup
  prev_set_emit = 0

  # gas-pedal episodes — for iter14 ground truth on instant power
  gas_episodes_peak_instant = []
  curr_gas_episode = None

  # cluster vs vEgo gap
  engaged_moving_gaps = []
  total_engaged_moving_s = 0.0
  total_engaged_standstill_s = 0.0

  drive_start_t = None
  drive_end_t = None

  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception as e:
      print(f"WARN: cannot open {rlog}: {e}", file=sys.stderr)
      continue

    last_v_ego = 0.0
    last_cluster = 0.0
    last_gas = False
    last_brake = False
    last_cruise = False

    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9
      if drive_start_t is None: drive_start_t = t
      drive_end_t = t

      if typ == "carState":
        cs = msg.carState
        last_v_ego = cs.vEgo
        last_cluster = cs.vEgoCluster
        last_gas = bool(cs.gasPressed)
        last_brake = bool(cs.brakePressed)
        cur_cruise = bool(cs.cruiseState.enabled)
        if cruise_prev is True and cur_cruise is False:
          # cruise off — count prior-1s SET emits
          while recent_set_emits and t - recent_set_emits[0][0] > 1.0:
            recent_set_emits.popleft()
          set_emits_prior_1s = sum(d for (_, d) in recent_set_emits)
          cruise_off_events.append({
            't': t, 'v_ego_mph': cs.vEgo * MPH,
            'brake_pressed': bool(cs.brakePressed),
            'gas_pressed': bool(cs.gasPressed),
            'cruise_available': bool(cs.cruiseState.available),
            'set_emits_prior_1s': set_emits_prior_1s,
          })
        cruise_prev = cur_cruise

      elif typ == "carStateSP":
        sp = msg.carStateSP
        total_csp += 1
        cur_state = int(sp.evLimiterState)
        state_dist[cur_state] += 1
        block_dist[str(sp.evLimiterLastBlockReason)] += 1
        yield_reason_dist[str(sp.evLimiterPowerGuardYieldReason)] += 1
        if sp.evLimiterPowerGuardLockoutActive:
          guard_lockout_frames += 1

        if sp.evLimiterFaultInhibitActive:
          fault_inhibit_seen = True

        # cumulative counter snapshots
        set_emitted_max = max(set_emitted_max, int(sp.evLimiterSetEmitted))
        set_dropped_max = max(set_dropped_max, int(sp.evLimiterSetDropped))
        set_requested_max = max(set_requested_max, int(sp.evLimiterSetRequested))
        all_btn_max = max(all_btn_max, int(sp.evLimiterAllBtnEmitted))
        ack_max = max(ack_max, int(sp.evLimiterSetClusterDecrementAcked))
        standstill_entered_max = max(standstill_entered_max, int(sp.evLimiterStandstillEntered))
        standstill_set_emitted_max = max(standstill_set_emitted_max, int(sp.evLimiterStandstillSetEmitted))
        recovery_yield_events_max = max(recovery_yield_events_max, int(sp.evLimiterRecoveryYieldEvents))
        recovery_lockouts_entered_max = max(recovery_lockouts_entered_max, int(sp.evLimiterRecoveryLockoutsEntered))
        suspected_cancel_max = max(suspected_cancel_max, int(sp.evLimiterSuspectedSccCancelEvents))

        # ev mode plumbing
        if sp.evModeAssumed: ev_mode_assumed_true += 1
        if sp.evModeParamReadOk: ev_mode_param_read_ok_true += 1

        # set emit delta tracking for cruise-off forensics
        cur_set_emit = int(sp.evLimiterSetEmitted)
        if cur_set_emit > prev_set_emit:
          recent_set_emits.append((t, cur_set_emit - prev_set_emit))
        prev_set_emit = cur_set_emit
        while recent_set_emits and t - recent_set_emits[0][0] > 1.0:
          recent_set_emits.popleft()

        # power-by-speed (use control power — the iter14 control signal)
        v_ego_mph = last_v_ego * MPH
        cluster_mph = last_cluster * MPH
        ctl_kw = float(sp.estPowerControlW) / 1000.0
        inst_kw = float(sp.estPowerInstantW) / 1000.0
        hud_kw = float(sp.estPowerW) / 1000.0
        bucket = int(v_ego_mph // 5) * 5
        if v_ego_mph > 5.0:
          control_power_by_speed[bucket].append(ctl_kw)
          instant_power_by_speed[bucket].append(inst_kw)
          hud_power_by_speed[bucket].append(hud_kw)

        # RECOVERY-while-capped detection (iter13 bug)
        is_recovery_capped = (cur_state == 4 and ctl_kw >= 40.0)
        if is_recovery_capped:
          if curr_run_start is None:
            curr_run_start = t
            curr_run_frames = 0
          curr_run_frames += 1
        else:
          if curr_run_start is not None:
            recovery_capped_runs.append({
              'start_t': curr_run_start, 'end_t': t,
              'duration_s': t - curr_run_start, 'frames': curr_run_frames,
            })
            curr_run_start = None
            curr_run_frames = 0

        # gas-pedal episodes — track peak instant
        if last_gas:
          if curr_gas_episode is None:
            curr_gas_episode = {'start_t': t, 'peak_inst_kw': 0.0, 'peak_ctl_kw': 0.0,
                                'peak_hud_kw': 0.0, 'max_v_ego_mph': 0.0}
          curr_gas_episode['peak_inst_kw'] = max(curr_gas_episode['peak_inst_kw'], inst_kw)
          curr_gas_episode['peak_ctl_kw'] = max(curr_gas_episode['peak_ctl_kw'], ctl_kw)
          curr_gas_episode['peak_hud_kw'] = max(curr_gas_episode['peak_hud_kw'], hud_kw)
          curr_gas_episode['max_v_ego_mph'] = max(curr_gas_episode['max_v_ego_mph'], v_ego_mph)
        else:
          if curr_gas_episode is not None:
            curr_gas_episode['end_t'] = t
            curr_gas_episode['duration_s'] = t - curr_gas_episode['start_t']
            gas_episodes_peak_instant.append(curr_gas_episode)
            curr_gas_episode = None

        # cluster vs vEgo gap (engaged, moving)
        if last_cruise and v_ego_mph > 5.0:
          engaged_moving_gaps.append(cluster_mph - v_ego_mph)
          total_engaged_moving_s += 0.01
        elif last_cruise:
          total_engaged_standstill_s += 0.01

  def pct(v, p):
    if not v: return None
    s = sorted(v)
    return s[min(int(p * len(s)), len(s)-1)]

  def speed_bucket_stats(buckets, key):
    out = {}
    for sp_bucket, vals in sorted(buckets.items()):
      if len(vals) < 100: continue
      sv = sorted(vals)
      n = len(sv)
      out[sp_bucket] = {
        'samples': n,
        f'{key}_p50_kw': sv[n//2],
        f'{key}_p90_kw': sv[int(n*0.9)],
        f'{key}_p99_kw': sv[int(n*0.99)],
        f'{key}_max_kw': sv[-1],
      }
    return out

  return {
    'route': route_dir.name,
    'segments': len(segs),
    'duration_s': drive_end_t - drive_start_t if drive_start_t else 0,
    'total_csp_frames': total_csp,
    'state_distribution': {STATE_NAMES.get(k,str(k)): v for k,v in state_dist.most_common()},
    'block_reason_distribution': dict(block_dist.most_common(10)),
    'guard_yield_reason_distribution': dict(yield_reason_dist.most_common()),
    'guard_lockout_active_frames': guard_lockout_frames,

    # iter14 acceptance metrics
    'recovery_yield_events_total': recovery_yield_events_max,
    'recovery_lockouts_entered_total': recovery_lockouts_entered_max,
    'suspected_scc_cancels_total': suspected_cancel_max,
    'fault_inhibit_seen': fault_inhibit_seen,

    # iter13 v4 metrics carried forward
    'set_emitted_total': set_emitted_max,
    'set_dropped_total': set_dropped_max,
    'set_requested_total': set_requested_max,
    'all_btn_emitted_total': all_btn_max,
    'cluster_decrement_acked_total': ack_max,
    'standstill_entered_total': standstill_entered_max,
    'standstill_set_emitted_total': standstill_set_emitted_max,
    'ev_mode_assumed_true_pct': ev_mode_assumed_true/max(total_csp,1)*100,
    'ev_mode_param_read_ok_true_pct': ev_mode_param_read_ok_true/max(total_csp,1)*100,

    # RECOVERY-while-capped runs (iter13 bug fingerprint — should be near 0)
    'recovery_while_capped_runs_count': len(recovery_capped_runs),
    'recovery_while_capped_total_s': sum(r['duration_s'] for r in recovery_capped_runs),
    'recovery_while_capped_max_run_s': max((r['duration_s'] for r in recovery_capped_runs), default=0),
    'recovery_while_capped_top5_longest': sorted(recovery_capped_runs, key=lambda r: -r['duration_s'])[:5],

    # gas-pedal peak estimator readings (ground truth comparison)
    'gas_episodes_count': len(gas_episodes_peak_instant),
    'gas_peak_instant_kw_max': max((e['peak_inst_kw'] for e in gas_episodes_peak_instant), default=0),
    'gas_peak_control_kw_max': max((e['peak_ctl_kw'] for e in gas_episodes_peak_instant), default=0),
    'gas_peak_hud_kw_max': max((e['peak_hud_kw'] for e in gas_episodes_peak_instant), default=0),
    'gas_top3_episodes_by_instant_peak': sorted(gas_episodes_peak_instant, key=lambda e: -e['peak_inst_kw'])[:3],

    # power-by-speed (control signal, the iter14 control input)
    'control_power_by_speed_5mph': speed_bucket_stats(control_power_by_speed, 'ctl'),
    'instant_power_by_speed_5mph': speed_bucket_stats(instant_power_by_speed, 'inst'),
    'hud_power_by_speed_5mph': speed_bucket_stats(hud_power_by_speed, 'hud'),

    # cluster vs vEgo
    'engaged_moving_s': total_engaged_moving_s,
    'engaged_standstill_s': total_engaged_standstill_s,
    'gap_p50_mph': pct(engaged_moving_gaps, 0.50),
    'gap_p90_mph': pct(engaged_moving_gaps, 0.90),
    'gap_p99_mph': pct(engaged_moving_gaps, 0.99),
    'gap_max_mph': max(engaged_moving_gaps, default=0),

    # cruise-off events with prior-1s SET activity
    'cruise_off_events_count': len(cruise_off_events),
    'cruise_off_events_with_set_in_prior_1s': sum(1 for e in cruise_off_events if e['set_emits_prior_1s'] > 0),
    'cruise_off_events_max_set_prior_1s': max((e['set_emits_prior_1s'] for e in cruise_off_events), default=0),
  }

if __name__ == "__main__":
  for arg in sys.argv[1:]:
    print(json.dumps(analyze(Path(arg)), indent=2, default=str))
    print()
