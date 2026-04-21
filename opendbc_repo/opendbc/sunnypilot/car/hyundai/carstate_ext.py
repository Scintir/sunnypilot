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
from opendbc.sunnypilot.car.hyundai.scintir_ev_limiter import get_shared_state as _scintir_shared_state
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


# Module-global: only warn once per process if the expected Scintir v2
# signals (TCS13 aBasis, CLU13 DTE) are absent on a HYBRID car.
_SCINTIR_MISSING_WARNED = False

# Approximate curb mass of a 2022 Santa Fe PHEV (kg). Used to turn
# aBasis (m/s^2) + vEgo (m/s) into an estimated propulsion power (W):
#   P ~= mass * max(0, aBasis) * vEgo
# Off by O(10%) because it ignores grade and drag losses, but that's more
# than precise enough for an "ICE-about-to-engage" threshold that the user
# will tune by hand anyway.
SCINTIR_VEHICLE_MASS_KG = 1950.0


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0

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

    self._update_scintir_ev_signals(ret, ret_sp, cp)

  def _update_scintir_ev_signals(self, ret: structs.CarState, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Populate Scintir signals used by the EV power limiter.

    Trigger input is an estimated propulsion power computed from TCS13.aBasis
    (aggregated longitudinal-accel demand — includes driver + stock SCC +
    control overlay) times vEgo times an approximate vehicle mass. DTE comes
    from the cluster (CLU13.CF_Clu_DTE) as a "battery has juice" proxy.

    Gated on the HYBRID flag since the limiter itself is HYBRID-only.
    """
    if not (self.CP.flags & HyundaiFlags.HYBRID):
      return

    try:
      abasis = float(cp.vl["TCS13"]["aBasis"])
      v_ego = float(ret.vEgo)
      power_w = SCINTIR_VEHICLE_MASS_KG * max(0.0, abasis) * v_ego
      ret_sp.scintirAccelDemand = abasis
      ret_sp.scintirEstPowerW = power_w
      self.scintir_accel_demand = abasis
      self.scintir_est_power_w = power_w
    except KeyError as e:
      global _SCINTIR_MISSING_WARNED
      if not _SCINTIR_MISSING_WARNED:
        print(f"[scintir] TCS13 aBasis missing from parser: {e}", file=sys.stderr)
        _SCINTIR_MISSING_WARNED = True

    try:
      dte_raw = int(cp.vl["CLU13"]["CF_Clu_DTE"])
      ret_sp.scintirDteRaw = dte_raw
      self.scintir_dte_raw = dte_raw
    except KeyError:
      pass

    pub = _scintir_shared_state()
    ret_sp.scintirEvLimiterActive = bool(pub["active"])
    ret_sp.scintirEvLimiterSetSpeedOffset = float(pub["set_speed_offset"])

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor
