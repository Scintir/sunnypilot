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

# Approximate curb mass of a 2022 Santa Fe PHEV (kg). Used in the EV power
# estimator (carstate_ext._update_ev_limiter_signals).
VEHICLE_MASS_KG = 1950.0
GRAVITY_MS2 = 9.81

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

# Steady-state road load (rolling resistance + aero drag). Conservative
# defaults for a midsize SUV; can be calibrated later from logs. Drive #6
# 7:40 ICE event exposed an estimator blind spot: at 73 mph holding speed
# against grade, aBasis was -0.4 (commanded decel) but motor was doing
# real work (drag + grade hold) ~50 kW. Iter6's marginal-only formula
# read 0 kW. Iter7 baseline + grade-positive-clamp catches it.
ROLLING_RESISTANCE_COEFF = 0.011          # Crr (dimensionless)
AERO_DRAG_COEFF = 0.75                    # CdA (m²); Santa Fe is boxier than typical sedan
AIR_DENSITY_KG_M3 = 1.225                 # sea level @ 15°C

# Asymmetric LP filter on the published estPowerW. User feedback (drive #6):
# the IMU-derived grade signal is noisy → estPowerW HUD reads erratically.
# Fast rise (capture spikes), slow fall (smooth display). Limiter publishes
# `max(raw, filtered)` so the protective gate can't be lagged by the filter.
POWER_TAU_RISE_S = 0.15                   # 150 ms tau — spikes captured almost immediately
POWER_TAU_FALL_S = 2.0                    # 2 s tau — slow decay, stable HUD
DT_CLAMP_MIN_S = 0.001                    # safety: never let dt blow up alpha
DT_CLAMP_MAX_S = 0.1                      # 100 ms (10x nominal)


def road_load_power_w(v_ego_ms: float) -> float:
  """Steady-state road load: rolling resistance (linear in v) + aero drag
  (cubic in v). With current Crr=0.011 and CdA=0.75 constants:
    33 m/s (74 mph) ≈ 24 kW
    27 m/s (60 mph) ≈ 14 kW
    22 m/s (50 mph) ≈ 10 kW
  Returns watts; never negative.
  """
  if v_ego_ms <= 0.0:
    return 0.0
  p_roll = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS_KG * GRAVITY_MS2 * v_ego_ms
  p_aero = 0.5 * AIR_DENSITY_KG_M3 * AERO_DRAG_COEFF * v_ego_ms ** 3
  return p_roll + p_aero


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0
    self.grade_accel_filtered = 0.0  # m/s^2, signed; positive = uphill
    # ESP12 silent-zero detector — drive #5 had iter5's grade fix publishing
    # gradeAccel=0 for 173k samples because lazy CANParser registration
    # failed silently. Now that ESP12 is explicitly subscribed, log once
    # if it still stays at zero for the first 5 s of operation.
    self._esp12_seen_nonzero = False
    self._esp12_zero_warning_logged = False
    self._esp12_first_call_frame = -1
    self._esp12_call_count = 0
    # Asymmetric LP filter for published estPowerW (iter7).
    self._power_filtered_w = 0.0
    self._power_filter_initialized = False

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
        # Silent-zero detector: log once if ESP12 stays at exactly 0 for the
        # first ~5 s of carstate calls. iter5 had this happen unnoticed for
        # an entire 40 min drive.
        self._esp12_call_count += 1
        if long_accel != 0.0:
          self._esp12_seen_nonzero = True
        elif (
          not self._esp12_zero_warning_logged
          and not self._esp12_seen_nonzero
          and self._esp12_call_count >= 500   # 5 s at 100 Hz
        ):
          print("[ev_limiter] WARNING: ESP12.LONG_ACCEL stuck at 0.0 for 5 s — "
                "grade-aware power is degraded to flat-only", file=sys.stderr)
          self._esp12_zero_warning_logged = True

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
      # Negative SCC accel command (commanded decel) does NOT cancel positive
      # grade contribution: drive #6's 7:40 ICE event had aBasis=-0.4 with
      # grade=+0.4, and iter6's `max(0, abasis+grade)` read 0 even though the
      # motor was doing real work to hold 73 mph against grade. Iter7 clamps
      # both positive separately so grade always counts and decel never cancels.
      uphill_grade = max(0.0, grade_f)
      abasis_pos = max(0.0, abasis)

      # Power = mass × v × (commanded-accel + grade-pull) + steady-state road load.
      # Road load (rolling resistance + aero drag) is the missing baseline iter5/6
      # ignored — at 73 mph it's ~23 kW alone, dominant enough that without it
      # the limiter's threshold is comparing apples to oranges.
      p_accel_grade_w = VEHICLE_MASS_KG * v_ego * (abasis_pos + uphill_grade)
      p_road_w = road_load_power_w(v_ego)
      raw_power_w = max(0.0, p_accel_grade_w + p_road_w)

      # Asymmetric LP for HUD smoothness + control responsiveness.
      # Use max(raw, filtered) for the published value so the protective gate
      # always sees the higher of the two — fast spike capture, slow HUD decay.
      if not self._power_filter_initialized:
        self._power_filtered_w = raw_power_w
        self._power_filter_initialized = True
      else:
        # dt-aware alpha; clamp dt so a timing hiccup can't blow up the filter.
        dt = GRADE_FILTER_DT_S
        if dt < DT_CLAMP_MIN_S:
          dt = DT_CLAMP_MIN_S
        elif dt > DT_CLAMP_MAX_S:
          dt = DT_CLAMP_MAX_S
        if raw_power_w > self._power_filtered_w:
          alpha = dt / (POWER_TAU_RISE_S + dt)
        else:
          alpha = dt / (POWER_TAU_FALL_S + dt)
        self._power_filtered_w += alpha * (raw_power_w - self._power_filtered_w)
      power_w_published = max(raw_power_w, self._power_filtered_w)

      ret_sp.accelDemand = abasis
      ret_sp.estPowerW = power_w_published
      # Publish the clipped grade value — useful for HUD/debug.
      # NOTE drive #6 forensic: this field has been observed to publish 0
      # in practice while estPowerW above publishes correctly. The sequential
      # writes look identical, root cause unknown. Not blocking iter7 since
      # consumers (HUD, limiter) read estPowerW; revisit when reproducible.
      ret_sp.evLimiterGradeAccel = float(grade_f)
      self.accel_demand = abasis
      self.est_power_w = power_w_published
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
