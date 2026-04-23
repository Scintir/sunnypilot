"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

EV Power Limiter — classic-CAN Hyundai HYBRID, stock-long only.

Prevents the ICE from kicking on during ACC by biasing the stock SCC set
speed via CLU11 button injection. Rewritten 2026-04-22 as a state machine
after the first flat-controller iteration regressed badly (pressed SET
while stopped, fought driver wheel input, saturated the bus).

States (published as evLimiterState uint8):
  0 IDLE                — gates pass, no action required
  1 STANDSTILL_HOLD     — vehicle at/near stop; emit absolutely nothing
  2 SOFT_CAP_ACTIVE     — ICE-imminent trigger held long enough; press SET_DECEL
  3 RECOVERY_ACTIVE     — observed set below user target and safe to raise
  4 DRIVER_OVERRIDE_SET — driver pressed wheel SET recently; we don't RES
  5 DRIVER_OVERRIDE_RES — driver pressed wheel RES recently; we don't SET
  6 BUS_FAULT_HOLD      — reserved for future use (bus error back-off)
  7 DISABLED            — not supported / not enabled / CC off

Key design points (per gpt-5.4 iteration-2 review):
  - Constant 20 mph gap ceiling combined with `vEgo > 15 mph` entry gate
    on SOFT_CAP. Below 15 mph the cap simply does not engage, which fixes
    the "stopped-and-crawling" regression.
  - SOFT_CAP requires load persistence (≥ 200 ms) and exits with
    hysteresis (power < 0.5 × threshold for ≥ 500 ms). No single-frame
    triggers.
  - Driver wheel input priority is directional: driver SET suppresses our
    RES for 2 s; driver RES suppresses our SET for 1 s. Windows extend
    while the driver holds the button.
  - Echo filter: 80 ms, first-matching-event-only. Driver press-and-hold
    after that first pulse gets through.
  - Burst count is 2 copies per commanded frame; global rate limit of 6
    logical presses/sec over any rolling 1-second window. No catchup mode.
  - user_target is seeded from observed_set_speed on every engage rising
    edge. Short cc-off intervals do NOT zero it; the next engage edge
    re-seeds it fresh.
"""
from collections import deque

from opendbc.car import structs
from opendbc.car.hyundai.values import Buttons, HyundaiFlags


try:
  from openpilot.common.params import Params as _Params
  _PARAMS_AVAILABLE = True
except Exception:  # opendbc may run outside openpilot (tests, standalone)
  _Params = None
  _PARAMS_AVAILABLE = False


ButtonType = structs.CarState.ButtonEvent.Type

TX_BUTTON_TO_EVENT_TYPE = {
  Buttons.RES_ACCEL: ButtonType.accelCruise,
  Buttons.SET_DECEL: ButtonType.decelCruise,
  Buttons.CANCEL:    ButtonType.cancel,
}

# State enum (stays in sync with evLimiterState @7 in cereal/custom.capnp)
STATE_IDLE                = 0
STATE_STANDSTILL_HOLD     = 1
STATE_SOFT_CAP_ACTIVE     = 2
STATE_RECOVERY_ACTIVE     = 3
STATE_DRIVER_OVERRIDE_SET = 4
STATE_DRIVER_OVERRIDE_RES = 5
STATE_BUS_FAULT_HOLD      = 6
STATE_DISABLED            = 7

# Frame rate — carcontroller runs at 100 Hz.
FRAMES_PER_SEC = 100

# Burst / cadence
BURST_COPIES = 2                         # copies per commanded frame
SET_COOLDOWN_FRAMES = 30                 # 300 ms between commanded SET frames
RES_COOLDOWN_FRAMES = 40                 # 400 ms between commanded RES frames
GLOBAL_RATE_LIMIT_PRESSES_PER_SEC = 6    # logical presses (incl. bursts) per rolling second
ECHO_FILTER_FRAMES = 8                   # 80 ms, first matching event only

# Driver-priority window lengths
DRIVER_OVERRIDE_SET_FRAMES = 200         # 2 s after last driver SET
DRIVER_OVERRIDE_RES_FRAMES = 100         # 1 s after last driver RES

# Standstill entry/exit
STANDSTILL_V_EGO_MS = 2 * 0.44704        # 2 mph
STANDSTILL_EXIT_V_EGO_MS = 3 * 0.44704   # 3 mph
BRAKE_LOW_SPEED_V_EGO_MS = 5 * 0.44704   # 5 mph (brake gates standstill only under this)
STANDSTILL_CONFIRM_FRAMES = 20           # 200 ms persistence on entry
RECOVERY_AFTER_STANDSTILL_FRAMES = 30    # 300 ms clean after standstill before RES allowed

# SOFT_CAP entry/exit
SOFT_CAP_V_EGO_FLOOR_MS = 15 * 0.44704   # 15 mph
SOFT_CAP_V_EGO_EXIT_MS = 12 * 0.44704    # 12 mph exit hysteresis
SOFT_CAP_POWER_ENTER_FRAC = 0.75         # of EVLimiterPowerThresholdKW
SOFT_CAP_POWER_EXIT_FRAC = 0.50
SOFT_CAP_ABASIS_ENTER = 0.2              # m/s^2
SOFT_CAP_ENTER_FRAMES = 20               # 200 ms persistence
SOFT_CAP_EXIT_FRAMES = 50                # 500 ms clean exit

# Gap (flat per gpt-5.4)
CONSTANT_MAX_GAP_MPH = 20.0

# Recovery
RECOVERY_DEADBAND_MS = 1.0 * 0.44704     # 1 mph deadband around user_target
RECOVERY_V_EGO_FLOOR_MS = 3 * 0.44704    # must be moving

# user_target clamp range
MPH_TO_MS = 0.44704
USER_TARGET_MIN_MS = 0.0
USER_TARGET_MAX_MS = 95.0 * MPH_TO_MS


# Module-level singleton so the CarState-side publisher (carstate_ext) can
# read state without passing references through CarController plumbing.
_SHARED_STATE: dict = {
  "active": False,
  "set_speed_offset": 0.0,   # m/s, max(0, user_target - observed)
  "user_target": 0.0,        # m/s
  "state": STATE_DISABLED,
}


def get_shared_state() -> dict:
  return _SHARED_STATE


class ScintirEVLimiter:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.supported = (
      not bool(CP.flags & HyundaiFlags.CANFD)
      and bool(CP.flags & HyundaiFlags.HYBRID)
      and not CP.openpilotLongitudinalControl
    )

    self._params = _Params() if _PARAMS_AVAILABLE else None

    # user_target lifecycle — seeded on engage rising edge, never auto-zeroed
    self.user_target_speed = 0.0
    self.was_cc_enabled = False

    # TX bookkeeping
    self.last_set_frame = -10000
    self.last_res_frame = -10000
    self.press_history = deque()  # of (frame, copies) — for global rate limit

    # Echo filter (single-slot, first-match consumption)
    self._pending_echo_button = Buttons.NONE
    self._pending_echo_frame = -10000

    # Driver override windows — track LAST driver press of each direction
    self._driver_set_last_frame = -10000
    self._driver_res_last_frame = -10000

    # SOFT_CAP persistence
    self._soft_cap_trigger_frames = 0
    self._soft_cap_clean_frames = 0
    self._soft_cap_on = False

    # STANDSTILL persistence
    self._standstill_trigger_frames = 0
    self._standstill_on = False
    self._left_standstill_at_frame = -10000

    # Published burst count (carcontroller reads this each frame)
    self.current_burst_count = BURST_COPIES

    # Debug state (for logging)
    self.state = STATE_DISABLED

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

  # ----- Helpers ----------------------------------------------------------

  def _consume_global_rate_limit(self, frame: int, copies: int) -> bool:
    """Return True and record the TX if adding `copies` presses in the last
    1 s stays at or under GLOBAL_RATE_LIMIT_PRESSES_PER_SEC. Otherwise drop."""
    cutoff = frame - FRAMES_PER_SEC
    while self.press_history and self.press_history[0][0] < cutoff:
      self.press_history.popleft()
    total = sum(c for _, c in self.press_history)
    if total + copies > GLOBAL_RATE_LIMIT_PRESSES_PER_SEC:
      return False
    self.press_history.append((frame, copies))
    return True

  def _process_button_events(self, CS, frame: int) -> None:
    """Feed driver wheel input into user_target + driver-override windows.
    First matching event within ECHO_FILTER_FRAMES of our TX is swallowed
    as our own echo; subsequent events pass through."""
    echo_window_open = (
      self._pending_echo_button != Buttons.NONE
      and (frame - self._pending_echo_frame) < ECHO_FILTER_FRAMES
    )
    for event in CS.out.buttonEvents:
      if not event.pressed:
        continue
      # Try to consume the echo on the first matching event
      if echo_window_open:
        our_type = TX_BUTTON_TO_EVENT_TYPE.get(self._pending_echo_button)
        if our_type is not None and event.type == our_type:
          # swallow + mark consumed
          self._pending_echo_button = Buttons.NONE
          echo_window_open = False
          continue
      # Real driver press
      if event.type == ButtonType.decelCruise:
        self.user_target_speed -= MPH_TO_MS
        self._driver_set_last_frame = frame
      elif event.type == ButtonType.accelCruise:
        self.user_target_speed += MPH_TO_MS
        self._driver_res_last_frame = frame
    self.user_target_speed = max(USER_TARGET_MIN_MS, min(USER_TARGET_MAX_MS, self.user_target_speed))

  def _in_driver_override_set(self, frame: int) -> bool:
    return (frame - self._driver_set_last_frame) < DRIVER_OVERRIDE_SET_FRAMES

  def _in_driver_override_res(self, frame: int) -> bool:
    return (frame - self._driver_res_last_frame) < DRIVER_OVERRIDE_RES_FRAMES

  def _update_standstill(self, v_ego, brake_pressed, standstill_flag) -> bool:
    trigger = (
      v_ego < STANDSTILL_V_EGO_MS
      or standstill_flag
      or (brake_pressed and v_ego < BRAKE_LOW_SPEED_V_EGO_MS)
    )
    exit_ok = (v_ego >= STANDSTILL_EXIT_V_EGO_MS and not brake_pressed and not standstill_flag)
    if trigger:
      self._standstill_trigger_frames += 1
      if self._standstill_trigger_frames >= STANDSTILL_CONFIRM_FRAMES:
        self._standstill_on = True
    else:
      self._standstill_trigger_frames = 0
    if self._standstill_on and exit_ok:
      self._standstill_on = False
    return self._standstill_on

  def _update_soft_cap(self, v_ego, brake_pressed, est_power_w, abasis, power_threshold_w) -> bool:
    if brake_pressed:
      self._soft_cap_on = False
      self._soft_cap_trigger_frames = 0
      return False
    enter_cond = (
      v_ego > SOFT_CAP_V_EGO_FLOOR_MS
      and est_power_w > SOFT_CAP_POWER_ENTER_FRAC * power_threshold_w
      and abasis > SOFT_CAP_ABASIS_ENTER
    )
    exit_cond = (
      v_ego < SOFT_CAP_V_EGO_EXIT_MS
      or est_power_w < SOFT_CAP_POWER_EXIT_FRAC * power_threshold_w
    )
    if enter_cond:
      self._soft_cap_trigger_frames += 1
      self._soft_cap_clean_frames = 0
      if self._soft_cap_trigger_frames >= SOFT_CAP_ENTER_FRAMES:
        self._soft_cap_on = True
    elif exit_cond:
      self._soft_cap_clean_frames += 1
      self._soft_cap_trigger_frames = 0
      if self._soft_cap_clean_frames >= SOFT_CAP_EXIT_FRAMES:
        self._soft_cap_on = False
    return self._soft_cap_on

  def _record_tx(self, frame: int, button: int) -> None:
    if button == Buttons.SET_DECEL:
      self.last_set_frame = frame
    elif button == Buttons.RES_ACCEL:
      self.last_res_frame = frame
    self._pending_echo_button = button
    self._pending_echo_frame = frame

  def _reset_tx_cadence(self) -> None:
    """Called on entry to STANDSTILL_HOLD / driver override — drops pending
    cadence so we don't snap a stale cooldown the instant we exit."""
    self.last_set_frame = -10000
    self.last_res_frame = -10000

  # ----- Main update ------------------------------------------------------

  def update(self, CC, CS, frame: int) -> tuple[int, bool]:
    """Advance the limiter one tick. Return (button, active)."""

    if not self.supported:
      return self._publish(Buttons.NONE, STATE_DISABLED, 0.0)

    if not self._read_bool("EVLimiterEnabled", False):
      self.user_target_speed = 0.0
      return self._publish(Buttons.NONE, STATE_DISABLED, 0.0)

    # Tunables (read every frame so params changes take effect live)
    power_threshold_w = float(self._read_int("EVLimiterPowerThresholdKW", 40)) * 1000.0
    dte_floor = float(self._read_int("EVLimiterDTEFloor", 5))

    # Inputs
    cc_enabled = bool(CC.enabled)
    v_ego = float(CS.out.vEgo)
    observed_set_speed = float(CS.out.cruiseState.speed)
    brake_pressed = bool(CS.out.brakePressed)
    gas_pressed = bool(CS.out.gasPressed)
    standstill_flag = bool(getattr(CS.out.cruiseState, "standstill", False))
    est_power_w = float(getattr(CS, "est_power_w", 0.0))
    abasis = float(getattr(CS, "accel_demand", 0.0))
    dte_raw = float(getattr(CS, "dte_raw", 0.0))

    # engage rising edge -> seed user_target from observed
    if cc_enabled and not self.was_cc_enabled:
      self.user_target_speed = observed_set_speed
    self.was_cc_enabled = cc_enabled

    # Driver wheel input — always processed (background user_target tracking
    # + override windows), even when we're going to HOLD this tick.
    self._process_button_events(CS, frame)

    if not cc_enabled:
      # Control internals reset after 200 ms cc-off, but user_target persists.
      self._reset_tx_cadence()
      self._soft_cap_on = False
      self._soft_cap_trigger_frames = 0
      self._standstill_trigger_frames = 0
      return self._publish(Buttons.NONE, STATE_DISABLED, observed_set_speed)

    if dte_raw <= dte_floor:
      return self._publish(Buttons.NONE, STATE_DISABLED, observed_set_speed)

    # Standstill gate takes precedence over everything (fixes the 250-press/s
    # stoplight spam). Emit absolutely nothing while held.
    standstill = self._update_standstill(v_ego, brake_pressed, standstill_flag)
    if standstill:
      self._reset_tx_cadence()
      self._left_standstill_at_frame = frame  # keep re-stamping so we can require clean time after exit
      return self._publish(Buttons.NONE, STATE_STANDSTILL_HOLD, observed_set_speed)

    # Driver override windows — if active, emit nothing in the blocked direction.
    in_override_set = self._in_driver_override_set(frame)
    in_override_res = self._in_driver_override_res(frame)

    # Soft cap state (independent of override — we might still SET if cap trips and no RES override)
    soft_cap = self._update_soft_cap(v_ego, brake_pressed, est_power_w, abasis, power_threshold_w)

    # Hard user-target cap (always enforced regardless of soft_cap):
    # if the SCC somehow ended up above the driver's target, pull it down.
    over_user_target = observed_set_speed > self.user_target_speed + 0.5 * MPH_TO_MS

    # Compute soft ceiling (only meaningful when soft_cap fires)
    soft_ceiling_ms = v_ego + CONSTANT_MAX_GAP_MPH * MPH_TO_MS
    over_soft_ceiling = observed_set_speed > soft_ceiling_ms + 0.5 * MPH_TO_MS

    # Candidate action picking (SET wins over RES; hard cap wins over soft cap)
    button = Buttons.NONE
    state = STATE_IDLE

    want_set = over_user_target or (soft_cap and over_soft_ceiling)
    if want_set:
      # Suppressed only by driver RES override (and even then, we still allow
      # a hard user-target-exceeded SET so we never let observed exceed target).
      suppressed = in_override_res and not over_user_target
      if not suppressed:
        if (frame - self.last_set_frame) >= SET_COOLDOWN_FRAMES:
          if self._consume_global_rate_limit(frame, BURST_COPIES):
            button = Buttons.SET_DECEL
            state = STATE_SOFT_CAP_ACTIVE if soft_cap and not over_user_target else STATE_SOFT_CAP_ACTIVE
      # If suppressed, still report override state for logging
      if suppressed:
        state = STATE_DRIVER_OVERRIDE_RES
    else:
      # Recovery: observed below user_target by > 1 mph, not in soft cap,
      # not under driver SET override, no gas pedal, above 3 mph, and at least
      # 300 ms out of standstill.
      below_target = observed_set_speed < self.user_target_speed - RECOVERY_DEADBAND_MS
      standstill_clear = (frame - self._left_standstill_at_frame) >= RECOVERY_AFTER_STANDSTILL_FRAMES
      may_recover = (
        below_target
        and not soft_cap
        and not in_override_set
        and not gas_pressed
        and v_ego > RECOVERY_V_EGO_FLOOR_MS
        and standstill_clear
      )
      if may_recover:
        if (frame - self.last_res_frame) >= RES_COOLDOWN_FRAMES:
          if self._consume_global_rate_limit(frame, BURST_COPIES):
            button = Buttons.RES_ACCEL
            state = STATE_RECOVERY_ACTIVE
      else:
        if in_override_set:
          state = STATE_DRIVER_OVERRIDE_SET
        elif in_override_res:
          state = STATE_DRIVER_OVERRIDE_RES
        elif soft_cap:
          state = STATE_SOFT_CAP_ACTIVE
        else:
          state = STATE_IDLE

    if button != Buttons.NONE:
      self._record_tx(frame, button)

    return self._publish(button, state, observed_set_speed)

  # ----- Publish ----------------------------------------------------------

  def _publish(self, button: int, state: int, observed_set_speed: float) -> tuple[int, bool]:
    self.state = state
    active = state in (STATE_SOFT_CAP_ACTIVE, STATE_RECOVERY_ACTIVE)
    self.current_burst_count = BURST_COPIES
    _SHARED_STATE["active"] = bool(active)
    _SHARED_STATE["set_speed_offset"] = max(0.0, self.user_target_speed - observed_set_speed)
    _SHARED_STATE["user_target"] = float(self.user_target_speed)
    _SHARED_STATE["state"] = int(state)
    return button, active
