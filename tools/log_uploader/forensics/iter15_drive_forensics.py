"""iter15 drive forensics — extends iter14 script with iter15 telemetry (Sections A-D).

Per-drive it reports everything iter14_drive_forensics did, PLUS:
  Section A (hard-preempt guard): RECOVERY-while-capped run durations, guard-forced
    transitions, strict edge-detected RECOVERY->SOFT_CAP episodes.
  Section B (grade clamp): raw vs published grade power, clamp-fired frame count.
  Section C (standstill windup): narrow-reset events, exit snapshots, exit->first-RES latency.
  Section D (post-RES quiet): quiet-active frames, suppressed softcap decrements, hard overrides.

Run with the off-device shim so the openpilot LogReader imports cleanly on a dev box.
"""
from __future__ import annotations
import sys, json
from pathlib import Path
from collections import defaultdict, Counter, deque

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _offdevice_shim  # noqa: F401  (installs import stubs; must precede logreader)
from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025
# Ground-truth state numbering from ev_limiter.py:98-106 (capnp @7 comment is stale).
STATE_NAMES = {0: "IDLE", 1: "STANDSTILL_HOLD", 2: "PRELAUNCH_SET", 3: "SOFT_CAP",
               4: "RECOVERY", 5: "OVERRIDE_SET", 6: "OVERRIDE_RES", 7: "BUS_FAULT", 8: "DISABLED"}
RECOVERY = 4
CAP_KW = 40.0  # est_power_control_w cap used by the state arbiter (RECOVERY-while-capped fingerprint)


def analyze(route_dir: Path):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--", 1)[1]))
  if not segs:
    return {'route': route_dir.name, 'error': 'no segments'}

  state_dist = Counter()
  block_dist = Counter()
  yield_reason_dist = Counter()
  guard_lockout_frames = 0

  # cumulative counters (snapshot at end via max — they only ever grow)
  cum = defaultdict(int)
  CUM_FIELDS = [
    'evLimiterSetEmitted', 'evLimiterSetDropped', 'evLimiterSetRequested', 'evLimiterAllBtnEmitted',
    'evLimiterSetClusterDecrementAcked', 'evLimiterStandstillEntered', 'evLimiterStandstillSetEmitted',
    'evLimiterRecoveryYieldEvents', 'evLimiterRecoveryLockoutsEntered', 'evLimiterSuspectedSccCancelEvents',
    # iter15 Section A
    'evLimiterGuardForcedTransitionEvents', 'evLimiterRecoveryYieldEpisodes',
    # iter15 Section B
    'evLimiterGradePowerCappedFrames',
    # iter15 Section C
    'evLimiterLongStandstillResets', 'evLimiterLongStandstillPrelaunchBackoffCleared',
    'evLimiterLongStandstillSoftcapReasonCleared', 'evLimiterStandstillExitedByAchieved',
    'evLimiterStandstillExitedByNoAckBackoff',
    # iter15 Section D
    'evLimiterSoftcapDecrementSuppressedFrames', 'evLimiterSoftcapDecrementSuppressedEvents',
    'evLimiterPostResHardOverrideEvents',
  ]

  fault_inhibit_seen = False
  ev_mode_assumed_true = 0
  ev_mode_param_read_ok_true = 0
  total_csp = 0

  control_power_by_speed = defaultdict(list)
  instant_power_by_speed = defaultdict(list)

  # Section A — RECOVERY-while-capped runs (the iter14 hard-gate FAILURE; iter15 hard-preempt should kill these)
  recovery_capped_runs = []
  curr_run_start = None
  curr_run_frames = 0
  guard_forced_frames = 0  # frames where evLimiterGuardForcedTransition True

  # Section B — grade clamp
  grade_raw_kw_samples = []      # evLimiterGradePowerRawW (pre-clamp), kW
  grade_raw_over_cap_peak = 0.0

  # Section C — standstill windup
  standstill_exit_times_s = []
  standstill_exit_to_res_latency = []
  standstill_exit_snapshots = []

  # Section D — post-RES quiet
  post_res_quiet_frames = 0

  # cruise-off events
  cruise_prev = None
  cruise_off_events = []
  recent_set_emits = deque()
  prev_set_emit = 0

  # cluster vs vEgo gap
  engaged_moving_gaps = []
  total_engaged_moving_s = 0.0
  total_engaged_standstill_s = 0.0

  drive_start_t = None
  drive_end_t = None
  prev_exit_time_val = None
  prev_latency_val = None

  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists():
      continue
    try:
      lr = LogReader(str(rlog))
    except Exception as e:
      print(f"WARN: cannot open {rlog}: {e}", file=sys.stderr)
      continue

    last_v_ego = 0.0
    last_cluster = 0.0

    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9
      if drive_start_t is None:
        drive_start_t = t
      drive_end_t = t

      if typ == "carState":
        cs = msg.carState
        last_v_ego = cs.vEgo
        last_cluster = cs.vEgoCluster
        cur_cruise = bool(cs.cruiseState.enabled)
        if cruise_prev is True and cur_cruise is False:
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

        for f in CUM_FIELDS:
          cum[f] = max(cum[f], int(getattr(sp, f)))

        if sp.evModeAssumed:
          ev_mode_assumed_true += 1
        if sp.evModeParamReadOk:
          ev_mode_param_read_ok_true += 1

        cur_set_emit = int(sp.evLimiterSetEmitted)
        if cur_set_emit > prev_set_emit:
          recent_set_emits.append((t, cur_set_emit - prev_set_emit))
        prev_set_emit = cur_set_emit
        while recent_set_emits and t - recent_set_emits[0][0] > 1.0:
          recent_set_emits.popleft()

        v_ego_mph = last_v_ego * MPH
        cluster_mph = last_cluster * MPH
        ctl_kw = float(sp.estPowerControlW) / 1000.0
        inst_kw = float(sp.estPowerInstantW) / 1000.0
        bucket = int(v_ego_mph // 5) * 5
        if v_ego_mph > 5.0:
          control_power_by_speed[bucket].append(ctl_kw)
          instant_power_by_speed[bucket].append(inst_kw)

        # Section A — RECOVERY-while-capped runs + guard-forced frames
        if bool(sp.evLimiterGuardForcedTransition):
          guard_forced_frames += 1
        is_recovery_capped = (cur_state == RECOVERY and ctl_kw >= CAP_KW)
        if is_recovery_capped:
          if curr_run_start is None:
            curr_run_start = t
            curr_run_frames = 0
          curr_run_frames += 1
        else:
          if curr_run_start is not None:
            recovery_capped_runs.append({'start_t': curr_run_start, 'end_t': t,
                                         'duration_s': t - curr_run_start, 'frames': curr_run_frames})
            curr_run_start = None
            curr_run_frames = 0

        # Section B — grade clamp
        grade_raw_kw = float(sp.evLimiterGradePowerRawW) / 1000.0
        grade_raw_kw_samples.append(grade_raw_kw)
        grade_raw_over_cap_peak = max(grade_raw_over_cap_peak, grade_raw_kw)

        # Section C — standstill exit telemetry (latch on change of the cumulative/sampled fields)
        exit_time_val = float(sp.evLimiterStandstillExitTimeS)
        if prev_exit_time_val is not None and exit_time_val != prev_exit_time_val and exit_time_val > 0:
          standstill_exit_times_s.append(exit_time_val)
          snap = str(sp.evLimiterStandstillExitStateSnapshot)
          if snap and snap != "":
            standstill_exit_snapshots.append(snap[:200])
        prev_exit_time_val = exit_time_val
        lat_val = int(sp.evLimiterStandstillExitToFirstResLatencyFrames)
        if prev_latency_val is not None and lat_val != prev_latency_val and lat_val > 0:
          standstill_exit_to_res_latency.append(lat_val)
        prev_latency_val = lat_val

        # Section D — post-RES quiet
        if bool(sp.evLimiterPostResQuietActive):
          post_res_quiet_frames += 1

        # cluster vs vEgo gap (engaged, moving)
        if cruise_prev and v_ego_mph > 5.0:
          engaged_moving_gaps.append(cluster_mph - v_ego_mph)
          total_engaged_moving_s += 0.01
        elif cruise_prev:
          total_engaged_standstill_s += 0.01

  def pct(v, p):
    if not v:
      return None
    s = sorted(v)
    return s[min(int(p * len(s)), len(s) - 1)]

  def speed_bucket_stats(buckets, key):
    out = {}
    for sp_bucket, vals in sorted(buckets.items()):
      if len(vals) < 100:
        continue
      sv = sorted(vals)
      n = len(sv)
      out[sp_bucket] = {'samples': n, f'{key}_p50_kw': round(sv[n // 2], 1),
                        f'{key}_p90_kw': round(sv[int(n * 0.9)], 1),
                        f'{key}_p99_kw': round(sv[int(n * 0.99)], 1),
                        f'{key}_max_kw': round(sv[-1], 1)}
    return out

  runs_over_05 = [r for r in recovery_capped_runs if r['duration_s'] > 0.5]

  return {
    'route': route_dir.name,
    'segments': len(segs),
    'duration_min': round((drive_end_t - drive_start_t) / 60.0, 1) if drive_start_t else 0,
    'total_csp_frames': total_csp,
    'state_distribution': {STATE_NAMES.get(k, str(k)): v for k, v in state_dist.most_common()},
    'block_reason_distribution': dict(block_dist.most_common(8)),
    'guard_yield_reason_distribution': dict(yield_reason_dist.most_common()),

    # ---- carryover safety acceptance ----
    'suspected_scc_cancels_total': cum['evLimiterSuspectedSccCancelEvents'],
    'fault_inhibit_seen': fault_inhibit_seen,
    'ev_mode_assumed_true_pct': round(ev_mode_assumed_true / max(total_csp, 1) * 100, 1),
    'ev_mode_param_read_ok_true_pct': round(ev_mode_param_read_ok_true / max(total_csp, 1) * 100, 1),
    'cruise_off_events_count': len(cruise_off_events),
    'cruise_off_events_with_set_in_prior_1s': sum(1 for e in cruise_off_events if e['set_emits_prior_1s'] > 0),
    'cruise_off_events_max_set_prior_1s': max((e['set_emits_prior_1s'] for e in cruise_off_events), default=0),

    # ---- Section A: hard-preempt guard ----
    'A_recovery_while_capped_runs_count': len(recovery_capped_runs),
    'A_recovery_while_capped_runs_over_0_5s': len(runs_over_05),
    'A_recovery_while_capped_max_run_s': round(max((r['duration_s'] for r in recovery_capped_runs), default=0), 3),
    'A_recovery_while_capped_total_s': round(sum(r['duration_s'] for r in recovery_capped_runs), 2),
    'A_recovery_while_capped_top5': sorted(recovery_capped_runs, key=lambda r: -r['duration_s'])[:5],
    'A_guard_forced_transition_frames': guard_forced_frames,
    'A_guard_forced_transition_events_total': cum['evLimiterGuardForcedTransitionEvents'],
    'A_recovery_yield_episodes_total': cum['evLimiterRecoveryYieldEpisodes'],
    'A_recovery_yield_events_total': cum['evLimiterRecoveryYieldEvents'],
    'A_recovery_lockouts_entered_total': cum['evLimiterRecoveryLockoutsEntered'],
    'A_guard_lockout_active_frames': guard_lockout_frames,

    # ---- Section B: grade clamp ----
    'B_grade_power_capped_frames_total': cum['evLimiterGradePowerCappedFrames'],
    'B_grade_raw_kw_p50': round(pct(grade_raw_kw_samples, 0.50) or 0, 2),
    'B_grade_raw_kw_p90': round(pct(grade_raw_kw_samples, 0.90) or 0, 2),
    'B_grade_raw_kw_p99': round(pct(grade_raw_kw_samples, 0.99) or 0, 2),
    'B_grade_raw_kw_peak': round(grade_raw_over_cap_peak, 2),

    # ---- Section C: standstill windup ----
    'C_standstill_entered_total': cum['evLimiterStandstillEntered'],
    'C_standstill_exited_by_achieved': cum['evLimiterStandstillExitedByAchieved'],
    'C_standstill_exited_by_noack_backoff': cum['evLimiterStandstillExitedByNoAckBackoff'],
    'C_long_standstill_resets_total': cum['evLimiterLongStandstillResets'],
    'C_long_standstill_prelaunch_backoff_cleared': cum['evLimiterLongStandstillPrelaunchBackoffCleared'],
    'C_long_standstill_softcap_reason_cleared': cum['evLimiterLongStandstillSoftcapReasonCleared'],
    'C_standstill_exit_durations_count': len(standstill_exit_times_s),
    'C_standstill_exit_dur_p50_s': round(pct(standstill_exit_times_s, 0.5) or 0, 1),
    'C_standstill_exit_dur_max_s': round(max(standstill_exit_times_s, default=0), 1),
    'C_exit_to_first_res_latency_p50_frames': pct(standstill_exit_to_res_latency, 0.5),
    'C_exit_to_first_res_latency_max_frames': max(standstill_exit_to_res_latency, default=0),
    'C_exit_snapshots_sample': standstill_exit_snapshots[:3],

    # ---- Section D: post-RES quiet ----
    'D_post_res_quiet_frames': post_res_quiet_frames,
    'D_softcap_decrement_suppressed_frames_total': cum['evLimiterSoftcapDecrementSuppressedFrames'],
    'D_softcap_decrement_suppressed_events_total': cum['evLimiterSoftcapDecrementSuppressedEvents'],
    'D_post_res_hard_override_events_total': cum['evLimiterPostResHardOverrideEvents'],

    # ---- button accounting (carryover) ----
    'set_requested_total': cum['evLimiterSetRequested'],
    'set_emitted_total': cum['evLimiterSetEmitted'],
    'set_dropped_total': cum['evLimiterSetDropped'],
    'all_btn_emitted_total': cum['evLimiterAllBtnEmitted'],
    'cluster_decrement_acked_total': cum['evLimiterSetClusterDecrementAcked'],
    'standstill_set_emitted_total': cum['evLimiterStandstillSetEmitted'],

    # ---- power-by-speed ----
    'control_power_by_speed_5mph': speed_bucket_stats(control_power_by_speed, 'ctl'),
    'instant_power_by_speed_5mph': speed_bucket_stats(instant_power_by_speed, 'inst'),

    # ---- cluster vs vEgo ----
    'engaged_moving_s': round(total_engaged_moving_s, 1),
    'engaged_standstill_s': round(total_engaged_standstill_s, 1),
    'gap_p50_mph': round(pct(engaged_moving_gaps, 0.50) or 0, 2),
    'gap_p90_mph': round(pct(engaged_moving_gaps, 0.90) or 0, 2),
    'gap_p99_mph': round(pct(engaged_moving_gaps, 0.99) or 0, 2),
    'gap_max_mph': round(max(engaged_moving_gaps, default=0), 2),
  }


if __name__ == "__main__":
  out_path = None
  args = sys.argv[1:]
  if args and args[0] == "-o":
    out_path = args[1]
    args = args[2:]
  results = [analyze(Path(a)) for a in args]
  blob = json.dumps(results if len(results) > 1 else results[0], indent=2, default=str)
  if out_path:
    Path(out_path).write_text(blob)
    print(f"wrote {out_path}")
  else:
    print(blob)
