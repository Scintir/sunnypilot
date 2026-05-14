#!/usr/bin/env python3
"""iter13 v4 drive forensics — walks all rlog.zst segments of a route and
produces a summary of evLimiter telemetry plus cluster-vs-vEgo metrics."""
from __future__ import annotations
import sys, os, json
from pathlib import Path
from collections import defaultdict
from openpilot.tools.lib.logreader import LogReader

MPH_PER_MS = 2.2369362920544025

def analyze_route(route_dir: str):
  segs = sorted([p for p in Path(route_dir).parent.glob(f"{Path(route_dir).name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  print(f"Analyzing {len(segs)} segments from {route_dir}", file=sys.stderr)

  total_can = 0
  ev_states = defaultdict(int)
  ev_block_reasons = defaultdict(int)
  fault_inhibit_seen = False
  fault_inhibit_reasons = defaultdict(int)
  suspected_cancel_max = 0
  set_emitted_max = 0
  set_dropped_max = 0
  set_requested_max = 0
  all_btn_max = 0
  cluster_decrement_acked_max = 0
  standstill_entered_max = 0
  standstill_set_emitted_max = 0
  ev_mode_assumed_true_frames = 0
  ev_mode_assumed_false_frames = 0
  ev_mode_param_read_ok_true_frames = 0
  ev_mode_param_read_ok_false_frames = 0
  total_csp_frames = 0

  # For p50/p90 gap during engaged-moving
  engaged_moving_gaps = []
  cluster_speeds_engaged_moving = []
  vego_speeds_engaged_moving = []

  # SCC cancel detection: cruise enabled true→false transitions
  cruise_enabled_prev = None
  cruise_off_events = []

  # Timeline of state changes
  state_change_timeline = []
  prev_state = None

  # Active periods
  total_time_engaged_s = 0.0
  total_time_standstill_engaged_s = 0.0
  total_time_moving_engaged_s = 0.0

  # Per-segment timestamps
  drive_start_t = None
  drive_end_t = None

  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists():
      continue
    try:
      lr = LogReader(str(rlog))
    except Exception as e:
      print(f"WARN: cannot open {rlog}: {e}", file=sys.stderr)
      continue

    prev_t = None
    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9
      if drive_start_t is None: drive_start_t = t
      drive_end_t = t

      if typ == "carState":
        cs = msg.carState
        cur_cruise = bool(cs.cruiseState.enabled)
        if cruise_enabled_prev is True and cur_cruise is False:
          # cruise off transition
          cruise_off_events.append({
            't': t,
            'brake_pressed': bool(cs.brakePressed),
            'gas_pressed': bool(cs.gasPressed),
            'gear_drive': str(cs.gearShifter) == 'GearShifter.drive',
            'door_open': bool(getattr(cs, 'doorOpen', False)),
            'seatbelt_unlatched': bool(getattr(cs, 'seatbeltUnlatched', False)),
            'cruise_available': bool(cs.cruiseState.available),
            'v_ego_mph': cs.vEgo * MPH_PER_MS,
          })
        cruise_enabled_prev = cur_cruise

      elif typ == "carStateSP":
        sp = msg.carStateSP
        total_csp_frames += 1
        ev_states[int(sp.evLimiterState)] += 1
        ev_block_reasons[str(sp.evLimiterLastBlockReason)] += 1
        if sp.evLimiterFaultInhibitActive:
          fault_inhibit_seen = True
          fault_inhibit_reasons[str(sp.evLimiterFaultInhibitReason)] += 1

        suspected_cancel_max = max(suspected_cancel_max, int(sp.evLimiterSuspectedSccCancelEvents))
        set_emitted_max = max(set_emitted_max, int(sp.evLimiterSetEmitted))
        set_dropped_max = max(set_dropped_max, int(sp.evLimiterSetDropped))
        set_requested_max = max(set_requested_max, int(sp.evLimiterSetRequested))
        all_btn_max = max(all_btn_max, int(sp.evLimiterAllBtnEmitted))
        cluster_decrement_acked_max = max(cluster_decrement_acked_max, int(sp.evLimiterSetClusterDecrementAcked))
        standstill_entered_max = max(standstill_entered_max, int(sp.evLimiterStandstillEntered))
        standstill_set_emitted_max = max(standstill_set_emitted_max, int(sp.evLimiterStandstillSetEmitted))

        if sp.evModeAssumed: ev_mode_assumed_true_frames += 1
        else:                ev_mode_assumed_false_frames += 1
        if sp.evModeParamReadOk: ev_mode_param_read_ok_true_frames += 1
        else:                    ev_mode_param_read_ok_false_frames += 1

        cur_state = int(sp.evLimiterState)
        if cur_state != prev_state:
          state_change_timeline.append((t, prev_state, cur_state))
          prev_state = cur_state

      elif typ == "controlsState":
        # gather cluster vs vEgo
        pass

    if prev_t is None:
      prev_t = t

  # Second pass for engaged-moving gap (we need cluster speed)
  # Actually carState.vEgoCluster + carState.vEgo gives us this
  # Re-walk
  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try:
      lr = LogReader(str(rlog))
    except Exception:
      continue
    cruise_now = False
    for msg in lr:
      typ = msg.which()
      if typ == "carState":
        cs = msg.carState
        cruise_now = bool(cs.cruiseState.enabled)
        cluster_mph = cs.vEgoCluster * MPH_PER_MS
        vego_mph = cs.vEgo * MPH_PER_MS
        if cruise_now and vego_mph > 5.0:
          gap = cluster_mph - vego_mph
          engaged_moving_gaps.append(gap)
          cluster_speeds_engaged_moving.append(cluster_mph)
          vego_speeds_engaged_moving.append(vego_mph)
          total_time_moving_engaged_s += 0.01
        elif cruise_now:
          total_time_standstill_engaged_s += 0.01
        if cruise_now:
          total_time_engaged_s += 0.01

  # Compute percentiles
  gaps = sorted(engaged_moving_gaps)
  def pct(v, p):
    if not v: return None
    i = int(p * len(v))
    if i >= len(v): i = len(v)-1
    return v[i]

  return {
    "route": Path(route_dir).name,
    "segment_count": len(segs),
    "drive_duration_s": drive_end_t - drive_start_t if drive_start_t else 0,
    "total_csp_frames": total_csp_frames,
    "ev_state_distribution": dict(ev_states),
    "block_reason_distribution": {k: v for k, v in sorted(ev_block_reasons.items(), key=lambda x: -x[1])},
    "fault_inhibit_seen": fault_inhibit_seen,
    "fault_inhibit_reasons": dict(fault_inhibit_reasons),
    "suspected_cancel_max": suspected_cancel_max,
    "set_emitted_total": set_emitted_max,    # cumulative counter, peak == final
    "set_dropped_total": set_dropped_max,
    "set_requested_total": set_requested_max,
    "all_btn_emitted_total": all_btn_max,
    "cluster_decrement_acked_total": cluster_decrement_acked_max,
    "standstill_entered_total": standstill_entered_max,
    "standstill_set_emitted_total": standstill_set_emitted_max,
    "ev_mode_assumed_true_pct": ev_mode_assumed_true_frames / max(total_csp_frames, 1) * 100,
    "ev_mode_param_read_ok_true_pct": ev_mode_param_read_ok_true_frames / max(total_csp_frames, 1) * 100,
    "cruise_off_events": len(cruise_off_events),
    "cruise_off_events_detail": cruise_off_events[:10],  # first 10
    "state_changes": len(state_change_timeline),
    "state_change_first_5": state_change_timeline[:5],
    "state_change_last_5": state_change_timeline[-5:],
    "engaged_moving_total_s": total_time_moving_engaged_s,
    "engaged_standstill_total_s": total_time_standstill_engaged_s,
    "engaged_total_s": total_time_engaged_s,
    "cluster_minus_vego_p50_mph": pct(gaps, 0.50),
    "cluster_minus_vego_p90_mph": pct(gaps, 0.90),
    "cluster_minus_vego_p99_mph": pct(gaps, 0.99),
    "cluster_minus_vego_max_mph": gaps[-1] if gaps else None,
    "engaged_moving_samples": len(gaps),
  }

if __name__ == "__main__":
  for arg in sys.argv[1:]:
    result = analyze_route(arg)
    print(json.dumps(result, indent=2, default=str))
    print()
