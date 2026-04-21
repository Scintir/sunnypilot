"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

EV Power Limiter — classic-CAN Hyundai HYBRID, stock-long only.

Decides whether to press CLU11 SET_DECEL or RES_ACCEL to bias the stock
SCC set speed so the car stays in EV mode as long as practical.

Controller is GAP-BASED (per user direction 2026-04-21):
    ceiling = min(user_target_speed, vEgo + max_gap)
and the limiter drives `observed_set_speed` toward that ceiling:
  - observed > ceiling           -> press SET_DECEL  (no load gate — the
                                    gap is the priority)
  - observed < ceiling AND not
    currently overdrawing power  -> press RES_ACCEL
  - otherwise                     -> hold

The power threshold is used ONLY as the safety gate on RES_ACCEL — we
never ask for MORE speed while the drivetrain is already over the
configured EV-power threshold.

Driver's own wheel presses are handled in the *background*: a SET- press
adjusts `user_target_speed` down by 1 mph (and RES+ up by 1 mph). The
limiter's gap logic then picks up the new target on subsequent frames.
Driver's button presses never directly move `observed_set_speed` out of
the limiter's control — the SCC may blip by 1 mph, but the next
commanded frame pulls it back to ceiling.

An "initial catch-up" mode kicks in when `observed - ceiling` is large
(fresh engagement from low vEgo with a high user target): larger burst
count and shorter cooldown so the set speed collapses onto the vEgo + 5
ceiling in roughly a second rather than 12.

Limiter runs at ALL speeds — there is no `MinSpeed` gate anymore per
user direction.
"""
from opendbc.car import structs
from opendbc.car.hyundai.values import Buttons, HyundaiFlags


try:
  from openpilot.common.params import Params as _Params
  _PARAMS_AVAILABLE = True
except Exception:  # opendbc may run outside openpilot (tests, standalone)
  _Params = None
  _PARAMS_AVAILABLE = False


ButtonType = structs.CarState.ButtonEvent.Type

# Mapping from CLU11 button codes (what the Hyundai wire protocol uses) to
# the button-event type carstate emits after edge detection.
TX_BUTTON_TO_EVENT_TYPE = {
  Buttons.RES_ACCEL: ButtonType.accelCruise,
  Buttons.SET_DECEL: ButtonType.decelCruise,
  Buttons.CANCEL:    ButtonType.cancel,
}

# Press cadences / burst counts
PRESS_COOLDOWN_FRAMES = 20             # 200 ms at 100 Hz — normal between presses
PRESS_BURST_COPIES = 5                  # normal duplicate presses per commanded frame
INITIAL_CATCHUP_COOLDOWN_FRAMES = 10    # 100 ms when catching up a big gap
INITIAL_CATCHUP_BURST_COPIES = 25       # bigger burst for rapid gap close (match stock RES pattern)

DRIVER_OVERRIDE_BACKOFF_FRAMES = 300    # 3 s back off after driver pedal interaction (NOT button press)
STUCK_LOOP_MAX_FRAMES = 6000            # 60 s continuous active -> safety reset
ECHO_FILTER_FRAMES = 30                 # drop button events within 300 ms of our own TX

# Unit constants (cruiseState.speed is m/s regardless of is_metric)
MPH_TO_MS = 0.44704
KPH_TO_MS = 1.0 / 3.6

# Clamp the "user target speed" we track to a sane range (m/s). An unbounded
# counter could drift arbitrarily high after many RES presses or arbitrarily
# low during sustained SET presses.
USER_TARGET_MIN_MS = 0.0
USER_TARGET_MAX_MS = 95.0 * MPH_TO_MS   # ~42 m/s ~ 153 kph — well above any street-legal target

# Module-level singleton so the CarController-side limiter can share state
# with the CarState-side publisher without passing references.
_SHARED_STATE: dict = {
  "active": False,
  "set_speed_offset": 0.0,  # m/s, user_target - observed
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
    self.last_tx_frame = -10000
    self.last_tx_button = Buttons.NONE
    self.driver_interacted_frame = -10000
    self.active_frames = 0
    self.active = False
    self.user_target_speed = 0.0
    self.was_cc_enabled = False
    # Dynamic burst count — CarController reads this to decide how many
    # copies of the CLU11 button to TX on the commanded frame.
    self.current_burst_count = PRESS_BURST_COPIES

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
    """Advance the limiter and return (button, active)."""

    if not self.supported:
      self._hard_reset()
      return Buttons.NONE, False

    if not self._read_bool("EVLimiterEnabled", False):
      self._hard_reset()
      return Buttons.NONE, False

    # Tunables (read every frame so changes take effect without restart)
    power_threshold_w = float(self._read_int("EVLimiterPowerThresholdKW", 40)) * 1000.0
    dte_floor = float(self._read_int("EVLimiterDTEFloor", 5))
    max_gap_ms = float(self._read_int("EVLimiterMaxGapMph", 5)) * MPH_TO_MS

    # Inputs
    cc_enabled = bool(CC.enabled)
    est_power_w = float(getattr(CS, "est_power_w", 0.0))
    dte_raw = float(getattr(CS, "dte_raw", 0.0))
    observed_set_speed = float(CS.out.cruiseState.speed)
    v_ego = float(CS.out.vEgo)
    gas_pressed = bool(CS.out.gasPressed)
    brake_pressed = bool(CS.out.brakePressed)

    # -------- user_target tracking ----------
    # On fresh engagement, take the SCC's current set speed as the user's
    # intended target. After that, only the driver's own wheel presses
    # (below) mutate user_target.
    if cc_enabled and not self.was_cc_enabled:
      self.user_target_speed = observed_set_speed
    self.was_cc_enabled = cc_enabled

    # Filter button events and apply driver's intent to user_target.
    for event in CS.out.buttonEvents:
      if not event.pressed:
        continue
      # Drop our own echoes: if we TX'd a button within ECHO_FILTER_FRAMES
      # and this event matches that button type, it's our echo, not the driver.
      our_event_type = TX_BUTTON_TO_EVENT_TYPE.get(self.last_tx_button)
      if our_event_type is not None \
          and (frame - self.last_tx_frame) < ECHO_FILTER_FRAMES \
          and event.type == our_event_type:
        # Consume the echo so it can't double-fire against the next event.
        self.last_tx_button = Buttons.NONE
        continue
      # Real driver press — adjust user_target in the background.
      if event.type == ButtonType.decelCruise:
        self.user_target_speed -= MPH_TO_MS
      elif event.type == ButtonType.accelCruise:
        self.user_target_speed += MPH_TO_MS

    # Keep user_target in a sane range.
    self.user_target_speed = max(USER_TARGET_MIN_MS, min(USER_TARGET_MAX_MS, self.user_target_speed))

    # Publish helper (called at every exit so the UI never sees stale state).
    def _publish_and_return(button: int, active: bool) -> tuple[int, bool]:
      _SHARED_STATE["active"] = bool(active)
      _SHARED_STATE["set_speed_offset"] = max(0.0, self.user_target_speed - observed_set_speed)
      return button, active

    # -------- driver override (pedal only) ----------
    # Wheel button presses now feed user_target and should NOT themselves
    # suppress the limiter. The pedal IS a real override though.
    if gas_pressed or brake_pressed:
      self.driver_interacted_frame = frame
    driver_active = (frame - self.driver_interacted_frame) < DRIVER_OVERRIDE_BACKOFF_FRAMES

    # -------- engagement gates (no min-speed gate anymore) ----------
    gates_ok = (
      cc_enabled
      and not driver_active
      and dte_raw > dte_floor
    )
    if not gates_ok:
      self.active = False
      self.active_frames = 0
      self.current_burst_count = PRESS_BURST_COPIES
      return _publish_and_return(Buttons.NONE, False)

    # -------- gap-based ceiling ----------
    ceiling = min(self.user_target_speed, v_ego + max_gap_ms)
    tolerance = 0.1  # m/s — avoid oscillation around the exact ceiling
    over_ceiling = observed_set_speed > ceiling + tolerance
    below_ceiling = observed_set_speed < ceiling - tolerance
    over_power = est_power_w > power_threshold_w

    # -------- initial catch-up mode ----------
    # If we're well above the ceiling (fresh engagement, target way above
    # current vEgo), use the bigger burst + shorter cooldown so the set speed
    # collapses fast.
    big_gap = (observed_set_speed - ceiling) > (10 * MPH_TO_MS)
    cooldown = INITIAL_CATCHUP_COOLDOWN_FRAMES if big_gap else PRESS_COOLDOWN_FRAMES
    self.current_burst_count = INITIAL_CATCHUP_BURST_COPIES if big_gap else PRESS_BURST_COPIES

    can_press = (frame - self.last_press_frame) >= cooldown
    button = Buttons.NONE
    self.active = False

    if over_ceiling:
      # Pull down: regardless of power (we're over the gap-from-vEgo ceiling
      # or over user_target — both are hard caps).
      self.active = True
      if can_press:
        button = Buttons.SET_DECEL
        self.last_press_frame = frame
    elif below_ceiling and not over_power:
      # Room to raise AND not currently overdrawing — ramp up toward ceiling.
      self.active = True
      if can_press:
        button = Buttons.RES_ACCEL
        self.last_press_frame = frame
    # else: hold (observed ≈ ceiling, or below but overdrawing)

    # Stuck-loop safety
    if self.active:
      self.active_frames += 1
      if self.active_frames > STUCK_LOOP_MAX_FRAMES:
        self._hard_reset()
        return Buttons.NONE, False
    else:
      self.active_frames = 0

    # Record what we're about to TX so future frames can filter our echo.
    if button != Buttons.NONE:
      self.last_tx_frame = frame
      self.last_tx_button = button

    return _publish_and_return(button, self.active)

  def _hard_reset(self) -> None:
    self.active = False
    self.active_frames = 0
    self.current_burst_count = PRESS_BURST_COPIES
    _SHARED_STATE["active"] = False
    _SHARED_STATE["set_speed_offset"] = 0.0
