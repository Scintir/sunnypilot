"""Investigate cruise-off events: was iter13 emitting SETs in the prior 1s?
If yes, possible limiter-induced cancel that circuit breaker missed.
If no, driver-induced (MAIN button) or genuine SCC fault."""
from __future__ import annotations
import sys, json
from collections import deque
from pathlib import Path
from openpilot.tools.lib.logreader import LogReader

MPH = 2.2369362920544025

def investigate(route_dir: Path, suspect_times: list[float], window_s: float = 1.0):
  segs = sorted([p for p in route_dir.parent.glob(f"{route_dir.name}*") if p.is_dir()],
                key=lambda p: int(p.name.rsplit("--",1)[1]))
  set_emit_hist: deque[tuple[float, int]] = deque()  # (t, set_emitted_cumulative)
  all_btn_hist: deque[tuple[float, int]] = deque()
  results = []

  prev_set_emit = 0
  prev_all_btn = 0
  cruise_prev = None
  main_btn_recent = deque()  # times of MAIN button presses

  for seg in segs:
    rlog = seg / "rlog.zst"
    if not rlog.exists(): continue
    try: lr = LogReader(str(rlog))
    except Exception: continue
    for msg in lr:
      typ = msg.which()
      t = msg.logMonoTime / 1e9

      if typ == "carStateSP":
        sp = msg.carStateSP
        cur_emit = int(sp.evLimiterSetEmitted)
        cur_btn = int(sp.evLimiterAllBtnEmitted)
        if cur_emit != prev_set_emit:
          set_emit_hist.append((t, cur_emit, cur_emit - prev_set_emit))
        if cur_btn != prev_all_btn:
          all_btn_hist.append((t, cur_btn, cur_btn - prev_all_btn))
        prev_set_emit = cur_emit
        prev_all_btn = cur_btn

      elif typ == "carState":
        cs = msg.carState
        cur = bool(cs.cruiseState.enabled)
        # MAIN button presses can be detected via cs.cruiseState.available transitions or via
        # cs.gearShifter changes; but cs has cs.cruiseButtons history? actually let's read via CAN
        if cruise_prev is True and cur is False:
          # check window: how many SET emissions in prior window_s?
          recent_sets = [(tt, c, d) for (tt, c, d) in set_emit_hist if t - window_s <= tt < t]
          recent_btns = [(tt, c, d) for (tt, c, d) in all_btn_hist if t - window_s <= tt < t]
          for st in suspect_times:
            if abs(t - st) < 0.5:  # within 500ms of suspect
              results.append({
                "cruise_off_t": t,
                "suspect_t": st,
                "v_ego_mph": cs.vEgo * MPH,
                "brake_pressed": bool(cs.brakePressed),
                "gas_pressed": bool(cs.gasPressed),
                "cruise_available": bool(cs.cruiseState.available),
                "set_emissions_prior_1s": sum(d for (_,_,d) in recent_sets),
                "set_emissions_total_pre_event": prev_set_emit,
                "all_btn_emissions_prior_1s": sum(d for (_,_,d) in recent_btns),
                "all_btn_emissions_total_pre_event": prev_all_btn,
                "recent_set_timeline": [(tt - t, d) for (tt, _, d) in recent_sets],
              })
        cruise_prev = cur
  return results

if __name__ == "__main__":
  drives = [
    ("/home/alex.smith/git/sunnypilot/can_data/00000078--26c897cf37", [974.88, 1661.64, 1794.14, 1958.62, 2261.01, 2473.45]),
    ("/home/alex.smith/git/sunnypilot/can_data/00000079--7364e21a2b", [52592.12, 52674.78, 52753.64, 52901.96, 53140.41, 53738.86, 53753.99, 53755.73, 53756.61, 54273.27]),
  ]
  for rd, suspects in drives:
    print(f"=== {Path(rd).name} ===")
    out = investigate(Path(rd), suspects)
    for r in out:
      print(json.dumps(r, indent=2, default=str))
    print()
