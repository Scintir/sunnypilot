"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

import sys

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.ev_limiter import get_shared_state as _ev_limiter_shared_state
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Module-global: only warn once per process if the expected EV-limiter
# signals (TCS13 aBasis, CLU13 DTE) are absent on a HYBRID car.
_EV_SIGNALS_MISSING_WARNED = False

# Approximate curb mass of a 2022 Santa Fe PHEV (kg). Used to turn
# aBasis (m/s^2) + grade contribution + vEgo (m/s) into an estimated
# propulsion-power demand (W) that's used as the SOFT_CAP trigger:
#   P ~= mass * max(0, aBasis + max(0, grade_accel)) * vEgo
# Drag is intentionally NOT modelled — what we want is *marginal* demand
# above flat-cruise baseline, since flat cruise rarely engages ICE.
VEHICLE_MASS_KG = 1950.0

# Grade derivation from CAN-only signals (no openpilot service dependency).
# Body-frame longitudinal accel (ESP12.LONG_ACCEL) ≈ inertial accel + g·sin(pitch),
# while wheel-speed-derived aEgo is purely inertial in the ground frame, so:
#   g·sin(pitch) ≈ LONG_ACCEL - aEgo
# Sign verified empirically in drive #4 (corr +0.65 across 17.6k samples;
# bin analysis: ratio ≈ 0.94 of geometric expectation across pitch range).
GRADE_FILTER_TAU_S = 1.0                  # ~1 s LP for grade — slow grade vs noisy axle accel
GRADE_FILTER_DT_S = 0.01                  # carstate_ext.update() runs at 100 Hz (card.py Ratekeeper)
GRADE_FILTER_ALPHA = GRADE_FILTER_DT_S / (GRADE_FILTER_TAU_S + GRADE_FILTER_DT_S)
GRADE_ACCEL_RAW_CLIP_MS2 = 1.5            # clip raw input before filtering
GRADE_ACCEL_FILTERED_CLIP_MS2 = 1.0       # clip filtered output (≈10% grade ceiling)


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0
    self.grade_accel_filtered = 0.0  # m/s^2, signed; positive = uphill

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv

    self._update_ev_limiter_signals(ret, ret_sp, cp)

  def _update_ev_limiter_signals(self, ret: structs.CarState, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Populate EV-limiter signals.

    Trigger input is an estimated propulsion power computed from TCS13.aBasis
    (aggregated longitudinal-accel demand — includes driver + stock SCC +
    control overlay) plus a grade-correction term derived from
    ESP12.LONG_ACCEL minus aEgo, times vEgo times an approximate vehicle
    mass. DTE comes from the cluster (CLU13.CF_Clu_DTE) as a "battery has
    juice" proxy.

    Gated on the HYBRID flag since the limiter itself is HYBRID-only.
    """
    if not (self.CP.flags & HyundaiFlags.HYBRID):
      return

    try:
      abasis = float(cp.vl["TCS13"]["aBasis"])
      v_ego = float(ret.vEgo)

      # Grade derivation (sign verified, drive #4 logs):
      #   body-frame LONG_ACCEL ≈ inertial accel + g·sin(pitch)
      #   ground-frame aEgo     ≈ inertial accel
      # So grade_accel ≈ LONG_ACCEL - aEgo, positive on uphill.
      # ESP12 is missing on some Hyundai PT buses (lazy `cp.vl[...]` access
      # raises AssertionError when the DBC doesn't define the message). On
      # miss we hold the last filter value rather than resetting — for a
      # transient miss after a valid history that stays conservative; on a
      # car that never has ESP12 at all the filter starts and stays at 0.
      try:
        long_accel = float(cp.vl["ESP12"]["LONG_ACCEL"])
        a_ego = float(ret.aEgo)
        grade_accel_raw = long_accel - a_ego
        if grade_accel_raw > GRADE_ACCEL_RAW_CLIP_MS2:
          grade_accel_raw = GRADE_ACCEL_RAW_CLIP_MS2
        elif grade_accel_raw < -GRADE_ACCEL_RAW_CLIP_MS2:
          grade_accel_raw = -GRADE_ACCEL_RAW_CLIP_MS2
        # First-order LP at ~1 s τ to smooth axle/IMU noise; sign retained
        # so consumers can see downhill grades for HUD/debug.
        self.grade_accel_filtered += GRADE_FILTER_ALPHA * (grade_accel_raw - self.grade_accel_filtered)
      except (KeyError, AssertionError):
        pass

      grade_f = self.grade_accel_filtered
      if grade_f > GRADE_ACCEL_FILTERED_CLIP_MS2:
        grade_f = GRADE_ACCEL_FILTERED_CLIP_MS2
      elif grade_f < -GRADE_ACCEL_FILTERED_CLIP_MS2:
        grade_f = -GRADE_ACCEL_FILTERED_CLIP_MS2
      # Only the uphill component contributes to ICE-engagement risk.
      # max(0, …) is applied AFTER filtering — applying it before would
      # let positive-side noise bias the filter on flat ground.
      uphill_grade = max(0.0, grade_f)

      effective_accel = max(0.0, abasis + uphill_grade)
      power_w = VEHICLE_MASS_KG * effective_accel * v_ego

      ret_sp.accelDemand = abasis
      ret_sp.estPowerW = power_w
      # Publish the clipped value — that's what consumers should see, and it
      # matches the schema comment about a ±1.0 m/s² ceiling.
      ret_sp.evLimiterGradeAccel = float(grade_f)
      self.accel_demand = abasis
      self.est_power_w = power_w
    except KeyError as e:
      global _EV_SIGNALS_MISSING_WARNED
      if not _EV_SIGNALS_MISSING_WARNED:
        print(f"[ev_limiter] TCS13 aBasis missing from parser: {e}", file=sys.stderr)
        _EV_SIGNALS_MISSING_WARNED = True

    try:
      dte_raw = int(cp.vl["CLU13"]["CF_Clu_DTE"])
      ret_sp.dteRaw = dte_raw
      self.dte_raw = dte_raw
    except KeyError:
      pass

    pub = _ev_limiter_shared_state()
    ret_sp.evLimiterActive = bool(pub["active"])
    ret_sp.evLimiterSetSpeedOffset = float(pub["set_speed_offset"])
    ret_sp.evLimiterUserTargetSpeed = float(pub.get("user_target", 0.0))
    ret_sp.evLimiterState = int(pub.get("state", 7))

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor
