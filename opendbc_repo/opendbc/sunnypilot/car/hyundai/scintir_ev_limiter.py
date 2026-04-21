"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Scintir EV power limiter — classic-CAN Hyundai HYBRID, stock-long only.

Decides whether to press CLU11 SET_DECEL or RES_ACCEL to bias the stock
SCC set speed down when the vehicle's propulsion-power demand looks about
to exceed the EV drivetrain's envelope (forcing ICE engagement), or back
up toward the driver's last observed target when demand has eased.

Trigger signal: estimated propulsion power
    P_est = mass * max(0, aBasis) * vEgo
where aBasis (TCS13) is the AGGREGATED longitudinal-acceleration demand
— it naturally sums driver pedal, stock SCC torque request, and any ESP
overlay — so the limiter fires whenever the VEHICLE is asking for power,
not just when the driver is pressing. Verified in 45 drives of real data
that aBasis > 0 during SCC-commanded acceleration with no driver pedal
(raw pedal signal is 0 in that case).

Engagement gates (all must hold for active TX):
  * sunnypilot is engaged (CC.enabled — panda controls_allowed == True);
  * no driver pedal / own-cruise-button press for DRIVER_OVERRIDE_BACKOFF_FRAMES;
  * cluster DTE above the configured floor (proxy for "battery has juice");
  * ego speed above the configured minimum.

Driver always overrides via brake or accelerator pedal (stock SCC
disengages on either); the limiter additionally backs off on any observed
cruise-button press.

Historical note: v1 of this limiter used BAT11 battery current + P_STS
HCU status + SOC floor. Off-device analysis of 45 real routes showed
those messages are not on any panda-logged bus on the Santa Fe PHEV, so
v2 pivoted to aBasis + DTE, which are both on bus 0.
"""
from opendbc.car.hyundai.values import Buttons, HyundaiFlags


try:
  from openpilot.common.params import Params as _Params
  _PARAMS_AVAILABLE = True
except Exception:  # opendbc may run outside openpilot (tests, standalone)
  _Params = None
  _PARAMS_AVAILABLE = False


PRESS_COOLDOWN_FRAMES = 20             # 200 ms at 100 Hz between SET_DECEL presses (or fast-recovery RES_ACCEL)
PRESS_BURST_COPIES = 5                  # duplicate presses sent per commanded frame so SCC reliably sees it
POWER_HYSTERESIS_W = 5000.0             # 5 kW — threshold band for "fast-recover" RES_ACCEL branch
DRIVER_OVERRIDE_BACKOFF_FRAMES = 300    # 3 s back off after driver pedal/button interaction
STUCK_LOOP_MAX_FRAMES = 6000            # 60 s continuous active -> safety reset to IDLE

# Slow-recovery branch: when the limiter has pulled set speed below the user
# target but power demand has settled into the hysteresis band (neither "over"
# nor "well under"), walk the set speed back up at a much slower cadence than
# the SET_DECEL rate so we don't bounce around the threshold.
SLOW_RECOVERY_COOLDOWN_FRAMES = 300     # 3 s between RES_ACCEL presses while slow-recovering
SLOW_RECOVERY_DWELL_FRAMES = 500        # 5 s of "not over threshold" before slow recovery begins

# Hard cap on how far below the user's set-speed target the limiter is allowed
# to pull: at most this many mph. Once reduction reaches the cap, SET_DECEL is
# inhibited even if demand remains above the power threshold — avoids the
# runaway "set speed keeps falling" behaviour the user flagged.
MAX_REDUCTION_MPH = 5.0
MPH_TO_MS = 0.44704
KPH_TO_MS = 1.0 / 3.6
MAX_REDUCTION_MS = MAX_REDUCTION_MPH * MPH_TO_MS   # cruiseState.speed is m/s, independent of is_metric


# Module-level singleton so the CarController-side limiter can share state
# with the CarState-side publisher (CarStateExt) without passing references.
# One CarController per process -> one limiter -> one reader. CarState consumes
# these values on the *next* frame, so there is always a one-frame lag between
# the CLU11 TX and the UI indicator; that is well below human perception.
_SHARED_STATE: dict = {
  "active": False,
  "set_speed_offset": 0.0,
}


def get_shared_state() -> dict:
  """Return the latest limiter state for UI-side consumers (read-only)."""
  return _SHARED_STATE


class ScintirEVLimiter:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    # Only meaningful on classic-CAN Hyundai HYBRID with stock longitudinal.
    # On any other config self.update() is a no-op.
    self.supported = (
      not bool(CP.flags & HyundaiFlags.CANFD)
      and bool(CP.flags & HyundaiFlags.HYBRID)
      and not CP.openpilotLongitudinalControl
    )

    self._params = _Params() if _PARAMS_AVAILABLE else None

    self.last_press_frame = -10000
    self.driver_interacted_frame = -10000
    self.active_frames = 0
    self.frames_below_threshold = 0   # counts how long est_power has been <= threshold (for slow recovery)
    self.active = False
    self.user_target_speed = 0.0

  # ----- Params helpers ---------------------------------------------------

  def _read_bool(self, key: str, default: bool) -> bool:
    if self._params is None:
      return default
    try:
      return bool(self._params.get_bool(key))
    except Exception:
      return default

  def _read_int(self, key: str, default: int) -> int:
    if self._params is None:
      return default
    try:
      raw = self._params.get(key)
      if raw is None:
        return default
      return int(raw)
    except (ValueError, TypeError):
      return default

  # ----- Main update ------------------------------------------------------

  def update(self, CC, CS, frame: int) -> tuple[int, bool]:
    """Advance the limiter and return (button, active).

    button -- Buttons.NONE / Buttons.RES_ACCEL / Buttons.SET_DECEL. NONE means
              do not TX CLU11 this frame.
    active -- True while the limiter is commanding a set-speed offset
              (including frames in cooldown between presses).
    """
    if not self.supported:
      self._hard_reset()
      return Buttons.NONE, False

    if not self._read_bool("ScintirEVLimiterEnabled", False):
      self._hard_reset()
      return Buttons.NONE, False

    # Tunables (bounded at the UI level; read every frame so changes take
    # effect without restart)
    power_threshold_kw = float(self._read_int("ScintirEVLimiterPowerThresholdKW", 30))
    power_threshold_w = power_threshold_kw * 1000.0
    dte_floor = float(self._read_int("ScintirEVLimiterDTEFloor", 5))
    min_speed_setting = float(self._read_int("ScintirEVLimiterMinSpeed", 15))

    # Inputs from CarState / CarControl (v2: aBasis-derived power + DTE)
    cc_enabled = bool(CC.enabled)
    est_power_w = float(getattr(CS, "scintir_est_power_w", 0.0))
    dte_raw = float(getattr(CS, "scintir_dte_raw", 0.0))
    observed_set_speed = float(CS.out.cruiseState.speed)
    v_ego = float(CS.out.vEgo)
    is_metric = bool(getattr(CS, "is_metric", False))
    gas_pressed = bool(CS.out.gasPressed)
    brake_pressed = bool(CS.out.brakePressed)

    # Keep user_target_speed tracked and _SHARED_STATE fresh even on early exits
    # so the UI indicator and any downstream consumer don't get stale values
    # (gpt-5.4 review: stale shared UI state on gate drop).
    def _publish_and_return(button: int, active: bool) -> tuple[int, bool]:
      _SHARED_STATE["active"] = bool(active)
      _SHARED_STATE["set_speed_offset"] = max(0.0, self.user_target_speed - observed_set_speed)
      return button, active

    # Detect driver's own cruise-button press on CLU11 (RES/SET/CANCEL).
    own_button_pressed = any(b in (Buttons.RES_ACCEL, Buttons.SET_DECEL, Buttons.CANCEL)
                             for b in getattr(CS, "cruise_buttons", ()))

    # Track user-target while we're not actively commanding
    if not self.active:
      self.user_target_speed = observed_set_speed

    # Driver override: pedal or explicit cruise button -> back off
    if gas_pressed or brake_pressed or own_button_pressed:
      self.driver_interacted_frame = frame
      # Only treat an own-button press as a target restate when the limiter
      # is NOT currently commanding. Otherwise the driver is just fighting our
      # offset and taking their press as "the new target" erases the real one.
      if own_button_pressed and not self.active:
        self.user_target_speed = observed_set_speed
    driver_active = (frame - self.driver_interacted_frame) < DRIVER_OVERRIDE_BACKOFF_FRAMES

    # Speed gate: setting is in the user's display units, convert to m/s
    min_speed_ms = min_speed_setting * (KPH_TO_MS if is_metric else MPH_TO_MS)

    gates_ok = (
      cc_enabled
      and not driver_active
      and dte_raw > dte_floor
      and v_ego >= min_speed_ms
    )

    if not gates_ok:
      self.active = False
      self.active_frames = 0
      self.frames_below_threshold = 0
      return _publish_and_return(Buttons.NONE, False)

    over = est_power_w > power_threshold_w
    under = est_power_w < (power_threshold_w - POWER_HYSTERESIS_W)

    # Track time since last "over threshold" event for the slow-recovery gate
    if over:
      self.frames_below_threshold = 0
    else:
      self.frames_below_threshold += 1

    # How much we've already pulled set speed down from the user's target (m/s)
    reduction = max(0.0, self.user_target_speed - observed_set_speed)
    below_target = observed_set_speed < self.user_target_speed

    can_press_fast = (frame - self.last_press_frame) >= PRESS_COOLDOWN_FRAMES
    can_press_slow = (frame - self.last_press_frame) >= SLOW_RECOVERY_COOLDOWN_FRAMES

    button = Buttons.NONE
    self.active = False

    if over and reduction < MAX_REDUCTION_MS:
      # Demand over threshold AND we still have room within the 5 mph cap → pull down
      self.active = True
      if can_press_fast:
        button = Buttons.SET_DECEL
        self.last_press_frame = frame
    elif under and below_target:
      # Power is well under threshold and set speed still below user target → fast recover
      self.active = True
      if can_press_fast:
        button = Buttons.RES_ACCEL
        self.last_press_frame = frame
    elif below_target and self.frames_below_threshold >= SLOW_RECOVERY_DWELL_FRAMES:
      # We're in the hysteresis band, set speed still below target, and power has
      # been settled for long enough → creep the set speed back up toward user target
      self.active = True
      if can_press_slow:
        button = Buttons.RES_ACCEL
        self.last_press_frame = frame

    # Stuck-loop safety: hard reset if we've been "active" way too long
    if self.active:
      self.active_frames += 1
      if self.active_frames > STUCK_LOOP_MAX_FRAMES:
        self._hard_reset()
        return Buttons.NONE, False
    else:
      self.active_frames = 0

    return _publish_and_return(button, self.active)

  def _hard_reset(self) -> None:
    self.active = False
    self.active_frames = 0
    self.frames_below_threshold = 0
    _SHARED_STATE["active"] = False
    _SHARED_STATE["set_speed_offset"] = 0.0
