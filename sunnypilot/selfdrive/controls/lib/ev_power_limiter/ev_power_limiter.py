"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
import cereal.messaging as messaging
from cereal import custom
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY as GRAVITY
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.ev_power_limiter.debug_logger import DebugLogger

EvPowerLimiterState = custom.LongitudinalPlanSP.EvPowerLimiter.EvPowerLimiterState

# Physics constants
AIR_DENSITY = 1.225  # kg/m^3 at sea level, 15C

# Default vehicle aerodynamic constants
DEFAULT_CD = 0.33  # drag coefficient (typical midsize SUV)
DEFAULT_FRONTAL_AREA = 2.7  # m^2 (typical midsize SUV)
DEFAULT_CRR = 0.012  # rolling resistance coefficient (typical SUV tires)

# Power limit defaults
DEFAULT_EV_POWER_LIMIT_KW = 50  # kW, conservative for ~67kW motor
MIN_EV_POWER_LIMIT_KW = 10  # kW, minimum configurable

# Speed thresholds
MIN_V_FOR_POWER_CALC = 0.5  # m/s, below this power limiting is disabled

# Max accel floor: if even maintaining speed exceeds EV power,
# clamp to this to allow graceful speed loss without aggressive braking
MAX_ACCEL_FLOOR = -0.5  # m/s^2

# Velocity target horizon (seconds ahead to project v_target from max_accel)
V_TARGET_HORIZON = 4.0


class EvPowerLimiter:
  """Limits longitudinal acceleration to keep estimated motor power below a user-configurable kW cap.

  Designed for EVs and PHEVs to stay within EV-only power range.
  The physics (P = F·v) are powertrain-agnostic, but the aerodynamic constants
  are defaults for a midsize SUV — accuracy improves with per-car values.
  Enabling on an ICE vehicle is harmless but provides no meaningful benefit.
  """

  def __init__(self, CP, CP_SP):
    self.params = Params()
    self.CP = CP
    self.frame = 0

    # Vehicle mass from CarParams (curb weight + STD_CARGO_KG, already set)
    self.mass = CP.mass

    # Aerodynamic constants (could be extended to per-car via CarParamsSP)
    self.cd = DEFAULT_CD
    self.frontal_area = DEFAULT_FRONTAL_AREA
    self.crr = DEFAULT_CRR

    # Debug logger (shared with MPC SP)
    self.debug = DebugLogger()

    # User-configurable params
    self.enabled = False
    self.power_limit_kw = DEFAULT_EV_POWER_LIMIT_KW
    self._read_params()

    # State
    self.state = EvPowerLimiterState.disabled
    self.is_enabled = False
    self.is_active = False

    # Outputs
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.0
    self.estimated_power_kw = 0.0
    self.max_accel = 2.0

  def _read_params(self):
    self.enabled = self.params.get_bool("EvPowerLimiter")
    self.debug.enabled = self.params.get_bool("EvPowerLimiterDebug")
    try:
      val = self.params.get("EvPowerLimitKw")
      if val is not None:
        self.power_limit_kw = max(MIN_EV_POWER_LIMIT_KW, int(val))
    except (ValueError, TypeError):
      self.power_limit_kw = DEFAULT_EV_POWER_LIMIT_KW

  @property
  def power_limit_w(self) -> float:
    return self.power_limit_kw * 1000.0

  def _compute_road_load_force(self, v: float, pitch: float) -> float:
    """Compute resistive forces (gravity + aero + rolling) in Newtons."""
    f_gravity = self.mass * GRAVITY * np.sin(pitch)
    f_aero = 0.5 * AIR_DENSITY * self.cd * self.frontal_area * v ** 2
    f_rolling = self.crr * self.mass * GRAVITY * np.cos(pitch)
    return f_gravity + f_aero + f_rolling

  def _compute_max_accel_from_resist(self, v: float, f_resist: float) -> float:
    """Back-solve for max acceleration keeping power <= limit, given pre-computed road load."""
    if v < MIN_V_FOR_POWER_CALC:
      return 2.0  # no limiting at very low speeds

    f_max = self.power_limit_w / v
    a_max = (f_max - f_resist) / self.mass
    return max(a_max, MAX_ACCEL_FLOOR)

  def update(self, sm: messaging.SubMaster, long_enabled: bool, v_ego: float, a_ego: float):
    self.frame += 1

    # Periodic param refresh
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self._read_params()

    # Read pitch from IMU
    if len(sm['carControl'].orientationNED) == 3:
      pitch = sm['carControl'].orientationNED[1]
    else:
      pitch = 0.0

    # Compute road load once, reuse for both power estimate and max accel
    f_resist = self._compute_road_load_force(v_ego, pitch)
    self.estimated_power_kw = (self.mass * a_ego + f_resist) * v_ego / 1000.0
    self.max_accel = self._compute_max_accel_from_resist(v_ego, f_resist)

    # State machine
    if not self.enabled or not long_enabled:
      self.state = EvPowerLimiterState.disabled
    elif v_ego < MIN_V_FOR_POWER_CALC:
      self.state = EvPowerLimiterState.inactive
    elif self.estimated_power_kw > self.power_limit_kw * 0.8:
      self.state = EvPowerLimiterState.active
    else:
      self.state = EvPowerLimiterState.inactive

    self.is_enabled = self.state != EvPowerLimiterState.disabled
    self.is_active = self.state == EvPowerLimiterState.active

    # Compute outputs (informational only — actual limiting is via accel_clip in the planner)
    if self.is_enabled and v_ego >= MIN_V_FOR_POWER_CALC:
      self.output_v_target = max(v_ego + self.max_accel * V_TARGET_HORIZON, MIN_V_FOR_POWER_CALC)
      self.output_a_target = self.max_accel
    else:
      self.output_v_target = V_CRUISE_UNSET
      self.output_a_target = 0.0

    self.debug.log_power_limiter(v_ego, a_ego, pitch, self.estimated_power_kw,
                                 self.max_accel, self.power_limit_kw, self.state, None)
