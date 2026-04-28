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

Key design points (per drive-#3 retro + gpt-5.5 review, 2026-04-27, with
follow-up fixes from gpt-5.5 v2 review):
  - SOFT_CAP target is `vEgo + 2 mph` (NOT vEgo + 20 — that was functionally
    no cap). When the cap fires it actually pulls observed down to remove
    accel demand from the SCC, preventing ICE engagement.
  - SOFT_CAP entry: power > threshold (slider value at face value, no 0.75
    multiplier) OR aBasis > +0.7 m/s² (catches accel pulses where power
    proxy may be muted). 300 ms persistence; 2 s clean exit; 1 s minimum
    dwell once entered (prevents cap/recover/cap cycling on rolling grades).
    Brake forces immediate exit AND clears the pre_cap_set latch (brake
    is authoritative).
  - Recovery aims at `pre_cap_set` (latched observed at SOFT_CAP entry).
    Driver RES EXTENDS pre_cap_set (max of current target, user_target, and
    observed-after-step) — but ONLY when a latch already exists; RES with
    no prior cap event does NOT create a recovery target out of thin air.
    Driver SET / CANCEL / brake clears pre_cap_set.
  - Driver-priority windows: SET blocks our RES for 2 s, RES blocks our SET
    for 3 s. Override windows take priority over LIMITING/RECOVERING in
    state reporting (driver respect signal beats limiter activity signal).
  - One-direction-at-a-time: after our own SET, block our own RES for
    1.5 s (and vice versa). Prevents visible oscillation.
  - 2 s settle delay between SOFT_CAP exit and RECOVERY firing.
  - user_target is seeded from observed at engage and primarily mutated by
    real driver edges (±1 mph each). It is also snapped to observed during
    a 300 ms post-edge window when the SCC's 5-mph quantization step lands
    after our edge — this is display/recovery-target tracking only and
    cannot itself trigger any TX. The hard user-target cap that previously
    used user_target as a control input was removed in iter-3.
  - Burst count 2 copies per frame; global rate limit 6 logical presses/sec.
  - Echo filter: 80 ms, first-matching-event-only (for buttonEvent stream).
  - Quantization observation window uses 200 ms TX attribution (different
    timing concern from the buttonEvent echo filter).
  - Standstill (vEgo<2 OR standstill_flag OR brake<5mph) emits zero presses.
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
SET_COOLDOWN_FRAMES = 30                 # 300 ms between commanded SET frames (default)
RES_COOLDOWN_FRAMES = 80                 # 800 ms between commanded RES frames
                                          # (was 400 ms = 2.5 mph/s. Drive #4: user reported
                                          # recovery ramp felt "way too aggressive". Halved
                                          # to 1.25 mph/s for gentler ramp toward user_target;
                                          # the load gate still pauses entirely if power/aBasis
                                          # crosses pause thresholds, so this is just a softer
                                          # baseline rate, not a safety mechanism.)
# Auto-resume guard: faster SET cadence right after engage so SOFT_CAP can
# keep up with Hyundai SCC's autonomous resume-ramp (~5-8 mph/s observed).
# Active for AUTO_RESUME_GUARD_FRAMES post-engage and only when SOFT_CAP is
# firing — so we don't burn rate-limit budget when there's nothing to fight.
AUTO_RESUME_GUARD_FRAMES = 500           # 5 s post-engage window
AUTO_RESUME_SET_COOLDOWN_FRAMES = 15     # 150 ms during guard + SOFT_CAP (vs 300 ms default)
# Disabled-frame RES press lookback: a wheel RES button event can land 1-2
# frames before cc_enabled rises (CAN ordering / SCC state propagation), so
# if we look only at the engage frame's buttonEvents we'll miss it. 20 frames
# = 200 ms is more than enough latitude for that race without false positives
# from older disabled-state presses.
DISABLED_RES_ENGAGE_WINDOW_FRAMES = 20
GLOBAL_RATE_LIMIT_PRESSES_PER_SEC = 6    # LOGICAL presses per rolling second; burst copies
                                          # are reliability dupes for the cluster, not separate
                                          # commands, so they don't count.
ECHO_FILTER_FRAMES = 8                   # 80 ms, first matching event only

# Driver-priority window lengths
DRIVER_OVERRIDE_SET_FRAMES = 200         # 2 s after last driver SET
DRIVER_OVERRIDE_RES_FRAMES = 300         # 3 s after last driver RES (extended per drive #3 fix)

# Standstill entry/exit
STANDSTILL_V_EGO_MS = 2 * 0.44704        # 2 mph
STANDSTILL_EXIT_V_EGO_MS = 3 * 0.44704   # 3 mph
BRAKE_LOW_SPEED_V_EGO_MS = 5 * 0.44704   # 5 mph (brake gates standstill only under this)
STANDSTILL_CONFIRM_FRAMES = 20           # 200 ms persistence on entry
RECOVERY_AFTER_STANDSTILL_FRAMES = 30    # 300 ms clean after standstill before RES allowed

# SOFT_CAP entry/exit
# Power threshold from EVLimiterPowerThresholdKW param is taken at face value
# now (no hidden 0.75x multiplier). Slider says "40 kW" -> entry at 40 kW.
SOFT_CAP_V_EGO_FLOOR_MS = 15 * 0.44704   # 15 mph
SOFT_CAP_V_EGO_EXIT_MS = 12 * 0.44704    # 12 mph exit hysteresis
SOFT_CAP_POWER_EXIT_KW_MARGIN = 8.0      # exit when power drops 8 kW below entry threshold
SOFT_CAP_ABASIS_FALLBACK = 0.7           # m/s^2 — secondary trigger for accel pulses where
                                          # power proxy may be muted (regen, low-SOC, brief grade).
                                          # Raised from 0.45 per gpt-5.5 review — 0.45 was firing
                                          # on ordinary highway acceleration / lane changes.
SOFT_CAP_ENTER_FRAMES = 30               # 300 ms persistence (was 200 ms)
SOFT_CAP_EXIT_FRAMES = 200               # 2 s clean exit (was 500 ms — prevents cap/recover/cap cycling)
SOFT_CAP_MIN_DWELL_FRAMES = 100          # 1 s minimum hold once entered

# SOFT_CAP target ceiling: pull observed_set down to vEgo + this many mph
# when cap fires (was 20 mph — functionally no cap; per drive #3 + gpt-5.5 review)
SOFT_CAP_CEILING_MARGIN_MPH = 2.0

# Recovery
RECOVERY_DEADBAND_MS = 1.0 * 0.44704     # 1 mph deadband around recovery target
RECOVERY_V_EGO_FLOOR_MS = 3 * 0.44704    # must be moving
RECOVERY_AFTER_CAP_EXIT_FRAMES = 30      # 300 ms settle after SOFT_CAP exits before RES'ing
                                          # (was 2 s — double-counted SOFT_CAP_EXIT_FRAMES'
                                          # 2 s clean-exit hysteresis. Drive #4 forensic
                                          # showed deterministic 2 s dead time before recovery
                                          # ramp; 300 ms is enough for the SCC to acknowledge
                                          # cap exit before the first RES press lands.)

# Recovery load gate — hysteresis on power headroom and aBasis.
# Pause recovery when load is nearing the SOFT_CAP threshold so RES presses
# don't push the motor across the ICE-engage boundary, and when commanded
# accel is rising (transient veto). Resume only when both signals are well
# below the pause thresholds — single-threshold gating produced press/no-press
# chatter at the boundary in iter4 simulator runs.
RECOVERY_POWER_PAUSE_FRAC = 0.60         # pause when est_power_w > 60% of cap
RECOVERY_POWER_RESUME_FRAC = 0.50        # resume when est_power_w < 50% of cap
RECOVERY_ABASIS_PAUSE_MS2 = 0.4          # pause when commanded longitudinal accel > 0.4 m/s²
RECOVERY_ABASIS_RESUME_MS2 = 0.25        # resume when it falls below 0.25 m/s²

# One-direction-at-a-time cooldowns to prevent visible oscillation
LIMITER_OPPOSITE_DIR_BLOCK_FRAMES = 150  # after our SET, block our RES for 1.5 s, and vice versa

# Driver-adjust observation window: after a driver button edge, watch a short
# follow-up window to catch the SCC's eventual +5 mph quantization step on
# held buttons. During this window we update HUD-display user_target and the
# recovery-target latch from observed; we do NOT use it for any cap/control.
DRIVER_ADJUST_WINDOW_FRAMES = 30         # 300 ms — long enough to see the cluster respond
TX_ECHO_ATTRIBUTION_FRAMES = 20          # 200 ms — observed changes within this window of
                                          # our last TX are attributed to us, not the driver

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


class EVLimiter:
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

    # SOFT_CAP persistence + dwell
    self._soft_cap_trigger_frames = 0
    self._soft_cap_clean_frames = 0
    self._soft_cap_on = False
    self._soft_cap_entered_frame = -10000
    self._soft_cap_exited_frame = -10000
    self._pre_cap_set_speed = 0.0  # latched at SOFT_CAP entry — recovery target

    # STANDSTILL persistence
    self._standstill_trigger_frames = 0
    self._standstill_on = False
    self._left_standstill_at_frame = -10000

    # Driver-adjust observation window — see DRIVER_ADJUST_WINDOW_FRAMES doc
    self._driver_adjust_until = -10000

    # Recovery load-gate hysteresis state
    self._recovery_load_paused = False

    # Pre-engage observed setpoint — frozen during DISABLED, used to seed
    # user_target on RES re-engage (drive #4: SCC autonomously snaps cluster
    # up on RES; reading post-engage observed gives a polluted seed).
    self._observed_set_speed_at_disable = 0.0
    # Last frame a physical RES press was seen while DISABLED. The engage
    # frame may not contain the press itself if cc_enabled rises 1-2 frames
    # later, so engage classification looks back this far.
    self._last_disabled_res_press_frame = -10000
    # Engage frame, used to gate the auto-resume faster-SET-cadence window.
    self._engage_frame = -10000

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

  def _consume_global_rate_limit(self, frame: int, n_logical: int) -> bool:
    """Return True and record the TX if adding `n_logical` logical button
    commands in the last 1 s stays at or under GLOBAL_RATE_LIMIT_PRESSES_PER_SEC.

    Burst copies are reliability duplicates (same logical press repeated for
    the cluster to see), so callers pass 1 per logical command — not BURST_COPIES.
    """
    cutoff = frame - FRAMES_PER_SEC
    while self.press_history and self.press_history[0][0] < cutoff:
      self.press_history.popleft()
    total = sum(c for _, c in self.press_history)
    if total + n_logical > GLOBAL_RATE_LIMIT_PRESSES_PER_SEC:
      return False
    self.press_history.append((frame, n_logical))
    return True

  def _process_button_events(self, CS, observed_set_speed: float, frame: int,
                              just_engaged: bool) -> None:
    """Feed driver wheel input into user_target + driver-override windows.
    First matching event within ECHO_FILTER_FRAMES of our TX is swallowed
    as our own echo; subsequent events pass through.

    Per drive #3 fix + gpt-5.5 v2 review:
    - Physical driver edges adjust user_target by ±1 mph each.
    - The SCC's 5-mph quantization step that follows a held button is
      caught later in _maybe_observe_quantization (display + recovery-
      target tracking only — never used as a control input).
    - Driver RES extends pre_cap_set ONLY if a latch already exists (no
      cap, no recovery target — RES alone shouldn't manufacture one).
    - Driver SET / CANCEL clears pre_cap_set.

    Per drive #4 fix:
    - When `just_engaged` (cc_enabled rising edge this frame), accelCruise
      events are the user re-engaging cruise — NOT intent to accelerate.
      Skip them entirely so we don't open a 3 s DRIVER_OVERRIDE_RES window
      that suppresses SOFT_CAP during Hyundai SCC's autonomous resume-
      ramp behavior. The engage-edge code already seeded user_target from
      observed; bumping it +1 mph here would also be wrong.
    """
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
          self._pending_echo_button = Buttons.NONE
          echo_window_open = False
          continue
      # Real driver press
      if just_engaged and event.type in (ButtonType.accelCruise, ButtonType.decelCruise):
        # Engage-edge SET/RES press: enabling cruise, NOT directional intent.
        # Don't open DRIVER_OVERRIDE windows, don't bump user_target on top
        # of the rising-edge seed. Stock SCC will autonomously resume on
        # RES — the limiter must remain free to fire SOFT_CAP if that
        # resume crosses the load threshold (drive #4 fix).
        continue
      if event.type == ButtonType.decelCruise:
        self.user_target_speed -= MPH_TO_MS
        self._driver_set_last_frame = frame
        self._driver_adjust_until = frame + DRIVER_ADJUST_WINDOW_FRAMES
        # Driver wants observed lower — abandon recovery toward an older value.
        self._pre_cap_set_speed = 0.0
      elif event.type == ButtonType.accelCruise:
        self.user_target_speed += MPH_TO_MS
        self._driver_res_last_frame = frame
        self._driver_adjust_until = frame + DRIVER_ADJUST_WINDOW_FRAMES
        # Driver RES aligns with recovery direction. Only EXTEND an existing
        # recovery latch — don't create one from zero (that would let the
        # limiter fire RES toward a target that no SOFT_CAP cycle ever set).
        if self._pre_cap_set_speed > 0.5 * MPH_TO_MS:
          self._pre_cap_set_speed = max(
            self._pre_cap_set_speed,
            self.user_target_speed,
            observed_set_speed,
          )
      elif event.type == ButtonType.cancel:
        self._pre_cap_set_speed = 0.0
    self.user_target_speed = max(USER_TARGET_MIN_MS, min(USER_TARGET_MAX_MS, self.user_target_speed))

  def _maybe_observe_quantization(self, observed_set_speed: float, frame: int) -> None:
    """During the 300 ms after a real driver edge, watch for the Hyundai SCC's
    delayed +5 mph quantization step (held button → cluster bumps observed by
    5 mph a few frames after our buttonEvent edge). Two effects, both display/
    target-tracking only — never used as a control input or cap:

      1. user_target snaps UP to observed if observed > user_target (driver
         intent surfaced via SCC step). Keeps HUD `EV TARGET` honest.
      2. If a recovery latch is active, extend pre_cap_set up to observed.
         Catches the case where driver RES'es during a cap cycle and the
         SCC's 5-mph step lifts the high-water mark above what we recorded
         on the edge frame.

    Driver SET-side: snap user_target DOWN if observed < user_target.
    """
    if frame >= self._driver_adjust_until:
      return
    # Don't react to observed changes that are likely our own SET/RES landing.
    last_tx_frame = max(self.last_set_frame, self.last_res_frame)
    if (frame - last_tx_frame) < TX_ECHO_ATTRIBUTION_FRAMES:
      return
    # Pick whichever driver edge is MORE RECENT (so a SET right after a RES
    # routes through the SET branch correctly).
    res_age = frame - self._driver_res_last_frame
    set_age = frame - self._driver_set_last_frame
    res_recent = res_age < DRIVER_ADJUST_WINDOW_FRAMES
    set_recent = set_age < DRIVER_ADJUST_WINDOW_FRAMES
    most_recent_is_res = res_recent and (not set_recent or res_age <= set_age)
    most_recent_is_set = set_recent and (not res_recent or set_age < res_age)
    if most_recent_is_res and observed_set_speed > self.user_target_speed + 0.5 * MPH_TO_MS:
      self.user_target_speed = min(USER_TARGET_MAX_MS, observed_set_speed)
      if self._pre_cap_set_speed > 0.5 * MPH_TO_MS:
        self._pre_cap_set_speed = max(self._pre_cap_set_speed, observed_set_speed)
    elif most_recent_is_set and observed_set_speed < self.user_target_speed - 0.5 * MPH_TO_MS:
      self.user_target_speed = max(USER_TARGET_MIN_MS, observed_set_speed)

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

  def _update_soft_cap(self, v_ego, brake_pressed, est_power_w, abasis,
                       power_threshold_w, frame: int) -> bool:
    """Detect high-load conditions and gate SOFT_CAP entry/exit.

    Two parallel triggers:
      - Power load: estimated motor power exceeds the user's threshold
        directly (no 0.75x multiplier — slider value is what fires).
        Best signal for steady high-load conditions like grades.
      - aBasis fallback: commanded longitudinal accel sustained above
        +0.7 m/s². Catches accel pulses where the power proxy may be
        muted (regen interaction, low-SOC, brief transients).

    Either trigger met for SOFT_CAP_ENTER_FRAMES (300 ms) -> enter.
    Power drops `SOFT_CAP_POWER_EXIT_KW_MARGIN` below threshold AND
    aBasis below fallback for SOFT_CAP_EXIT_FRAMES (2 s) -> exit.
    Brake forces immediate exit AND clears the recovery latch. Minimum
    dwell of 1 s prevents rapid cap/recover/cap cycling on rolling grades.
    """
    if brake_pressed:
      # Record exit frame so the post-cap settle window applies after brake
      # release — otherwise recovery could fire immediately on brake release.
      if self._soft_cap_on:
        self._soft_cap_exited_frame = frame
      self._soft_cap_on = False
      self._soft_cap_trigger_frames = 0
      self._soft_cap_clean_frames = 0
      # Brake is authoritative override — kill any pending recovery target.
      self._pre_cap_set_speed = 0.0
      return False

    high_load = est_power_w > power_threshold_w
    high_abasis = abasis > SOFT_CAP_ABASIS_FALLBACK
    enter_cond = (v_ego > SOFT_CAP_V_EGO_FLOOR_MS) and (high_load or high_abasis)
    # Clamp exit threshold so a very low slider value doesn't push it negative
    # (which would deadlock — `est_power_w < negative` never true). Use a small
    # positive floor and `<=` so power=0 always satisfies the exit threshold.
    exit_threshold_w = max(500.0, power_threshold_w - SOFT_CAP_POWER_EXIT_KW_MARGIN * 1000.0)
    exit_cond = (
      v_ego < SOFT_CAP_V_EGO_EXIT_MS
      or (est_power_w <= exit_threshold_w and abasis < SOFT_CAP_ABASIS_FALLBACK)
    )

    if enter_cond:
      self._soft_cap_trigger_frames += 1
      self._soft_cap_clean_frames = 0
      if not self._soft_cap_on and self._soft_cap_trigger_frames >= SOFT_CAP_ENTER_FRAMES:
        self._soft_cap_on = True
        self._soft_cap_entered_frame = frame
    elif exit_cond:
      self._soft_cap_clean_frames += 1
      self._soft_cap_trigger_frames = 0
      # Honor minimum dwell — don't drop the cap the instant load eases.
      held_long_enough = (frame - self._soft_cap_entered_frame) >= SOFT_CAP_MIN_DWELL_FRAMES
      if (
        self._soft_cap_on
        and held_long_enough
        and self._soft_cap_clean_frames >= SOFT_CAP_EXIT_FRAMES
      ):
        self._soft_cap_on = False
        self._soft_cap_exited_frame = frame
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

    # Handle DISABLED state EARLY — track engage-classification inputs but
    # do NOT process button events through the normal driver-adjust path.
    # Drive #4 race fix: a wheel RES press can land 1-2 frames before
    # cc_enabled rises (CAN ordering). If we processed it as if cruise were
    # active we'd open DRV_RES + driver-adjust windows that survive the
    # engage transition and corrupt the engage-edge seed.
    if not cc_enabled:
      # Track cluster's last-observed set so RES re-engage can seed
      # user_target from this (vs SCC-polluted post-engage observed).
      self._observed_set_speed_at_disable = observed_set_speed
      # Note physical RES presses for engage classification — but only
      # ones that are NOT our own TX echoes (we shouldn't TX while
      # disabled, but defensive against stale echoes).
      echo_window_open = (
        self._pending_echo_button != Buttons.NONE
        and (frame - self._pending_echo_frame) < ECHO_FILTER_FRAMES
      )
      our_type = TX_BUTTON_TO_EVENT_TYPE.get(self._pending_echo_button) if echo_window_open else None
      for e in CS.out.buttonEvents:
        if not e.pressed or e.type != ButtonType.accelCruise:
          continue
        if echo_window_open and our_type == ButtonType.accelCruise:
          # Consume our own echo, don't count as physical press.
          self._pending_echo_button = Buttons.NONE
          echo_window_open = False
          our_type = None
          continue
        self._last_disabled_res_press_frame = frame
        break
      # Drop stale driver-adjust / override state from a previous engaged
      # session so the engage frame starts clean.
      self._driver_adjust_until = -10000
      self._driver_res_last_frame = -10000
      self._driver_set_last_frame = -10000
      # Reset control internals.
      self._reset_tx_cadence()
      self._soft_cap_on = False
      self._soft_cap_trigger_frames = 0
      self._standstill_trigger_frames = 0
      self._recovery_load_paused = False
      self.was_cc_enabled = cc_enabled
      return self._publish(Buttons.NONE, STATE_DISABLED, observed_set_speed)

    # cc_enabled is True from here on.
    # Engage rising edge -> seed user_target; clear pre_cap latch.
    just_engaged = not self.was_cc_enabled
    engage_via_res = False
    if just_engaged:
      self._engage_frame = frame
      # Engage via RES? Check current-frame buttonEvents AND recent
      # disabled-frame RES presses (the wheel press may have landed before
      # cc_enabled rose). Engage via SET / main-switch leaves observed
      # clean (no SCC autonomous resume snap), so we only redirect the
      # seed for engage_via_res cases.
      for e in CS.out.buttonEvents:
        if e.pressed and e.type == ButtonType.accelCruise:
          engage_via_res = True
          break
      if not engage_via_res:
        recent_disabled_res = (frame - self._last_disabled_res_press_frame) <= DISABLED_RES_ENGAGE_WINDOW_FRAMES
        if recent_disabled_res:
          engage_via_res = True
      if engage_via_res and self._observed_set_speed_at_disable > 0.5 * MPH_TO_MS:
        # Drive #4: SCC autonomously snaps cluster up on RES re-engage.
        # Seed from the frozen pre-disable value instead.
        self.user_target_speed = self._observed_set_speed_at_disable
      else:
        self.user_target_speed = observed_set_speed
      self._pre_cap_set_speed = 0.0
      # Defensive: ensure no stale adjust window survives into engaged
      # state (DISABLED branch already clears these but make it explicit
      # for readers expecting engage-edge invariants).
      self._driver_adjust_until = -10000
    self.was_cc_enabled = cc_enabled

    # Driver wheel input — only processed when cc_enabled (disabled-state
    # button events are handled in the DISABLED branch above). On the
    # just_engaged frame, accelCruise events are skipped (engage press
    # isn't accel intent).
    self._process_button_events(CS, observed_set_speed, frame, just_engaged)

    # Observe the SCC's 5-mph quantization step that lands a few frames after
    # a held driver button — display/recovery-target tracking only, never
    # used as a control input.
    self._maybe_observe_quantization(observed_set_speed, frame)

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

    # Soft cap state. Power threshold from param is taken at face value
    # (no 0.75x multiplier). Latches pre_cap_set on rising edge.
    soft_cap_prev = self._soft_cap_on
    soft_cap = self._update_soft_cap(v_ego, brake_pressed, est_power_w, abasis,
                                     power_threshold_w, frame)
    if soft_cap and not soft_cap_prev:
      # Latch the observed set speed at the moment cap engaged — that's where
      # recovery should aim. If a previous latch is still alive (driver pressed
      # RES during the prior cap cycle), keep the higher value.
      self._pre_cap_set_speed = max(self._pre_cap_set_speed, observed_set_speed)

    # Soft ceiling: pull observed down to vEgo + small margin (was vEgo + 20).
    # Margin keeps SCC from commanding *deceleration*; just removes accel demand.
    soft_ceiling_ms = v_ego + SOFT_CAP_CEILING_MARGIN_MPH * MPH_TO_MS
    over_soft_ceiling = observed_set_speed > soft_ceiling_ms + 0.5 * MPH_TO_MS

    # One-direction-at-a-time gates: after we just pressed SET, block our own
    # RES for a window so the cluster doesn't tick down/up/down. Same
    # symmetrically for RES blocking SET.
    self_set_recent = (frame - self.last_set_frame) < LIMITER_OPPOSITE_DIR_BLOCK_FRAMES
    self_res_recent = (frame - self.last_res_frame) < LIMITER_OPPOSITE_DIR_BLOCK_FRAMES

    # Recovery latch maintenance: clear once observed catches up to the
    # recovery target. ONLY clear when soft cap is off — during an active
    # cap cycle, observed sits at or above the latched value (the latch was
    # taken at the cap's rising edge and the SET cascade hasn't pulled
    # observed down yet), so we'd otherwise spuriously clear the latch in
    # the very first cap-active frame. After the cap exits, the SET cascade
    # has lowered observed below the latch, and recovery (or driver RES)
    # gradually brings it back up; when it reaches the target, clear.
    if (
      not self._soft_cap_on
      and self._pre_cap_set_speed > 0.5 * MPH_TO_MS
      and observed_set_speed >= self._pre_cap_set_speed - 0.1 * MPH_TO_MS
    ):
      self._pre_cap_set_speed = 0.0

    # Update recovery load-gate hysteresis. Power thresholds scale with
    # the user's slider so the gate stays meaningful at any cap setting.
    pause_pwr_w = power_threshold_w * RECOVERY_POWER_PAUSE_FRAC
    resume_pwr_w = power_threshold_w * RECOVERY_POWER_RESUME_FRAC
    if self._recovery_load_paused:
      if est_power_w < resume_pwr_w and abasis < RECOVERY_ABASIS_RESUME_MS2:
        self._recovery_load_paused = False
    else:
      if est_power_w > pause_pwr_w or abasis > RECOVERY_ABASIS_PAUSE_MS2:
        self._recovery_load_paused = True

    button = Buttons.NONE

    # State reporting from internal flags (NOT TX outcome). Driver overrides
    # take priority over cap/recovery for HUD purposes — when the driver is
    # actively pressing buttons, the relevant signal to the driver is "we're
    # respecting your input," even if soft_cap is technically still on.
    # Pick the more-recent driver edge if both windows overlap, so HUD
    # shows the freshest driver intent.
    if in_override_set and in_override_res:
      set_age = frame - self._driver_set_last_frame
      res_age = frame - self._driver_res_last_frame
      base_state = STATE_DRIVER_OVERRIDE_SET if set_age <= res_age else STATE_DRIVER_OVERRIDE_RES
    elif in_override_set:
      base_state = STATE_DRIVER_OVERRIDE_SET
    elif in_override_res:
      base_state = STATE_DRIVER_OVERRIDE_RES
    elif soft_cap:
      base_state = STATE_SOFT_CAP_ACTIVE
    elif self._pre_cap_set_speed > 0.5 * MPH_TO_MS \
         and observed_set_speed < self._pre_cap_set_speed - RECOVERY_DEADBAND_MS \
         and not gas_pressed and not brake_pressed and v_ego > RECOVERY_V_EGO_FLOOR_MS \
         and (frame - self._soft_cap_exited_frame) >= RECOVERY_AFTER_CAP_EXIT_FRAMES:
      # Only show RECOVERING once the post-cap settle delay has elapsed —
      # before that, RES TX is blocked anyway.
      base_state = STATE_RECOVERY_ACTIVE
    else:
      base_state = STATE_IDLE
    state = base_state

    want_set = soft_cap and over_soft_ceiling
    if want_set:
      # SOFT_CAP fires only after 300 ms of high-load persistence (or aBasis
      # fallback) — by the time we get here, the load signal is real, not
      # noise. The driver-RES override window exists to respect "I want to
      # accelerate" intent, but it does NOT extend to letting the motor
      # cross the ICE-engage boundary. Per drive #4: SCC's autonomous
      # resume on re-engage was suppressing SOFT_CAP for 3 s while the
      # cluster ran 12 mph above target. Driver still has CANCEL/brake/SET
      # to push back if SOFT_CAP is over-firing.
      # Self-RES suppression remains — that prevents oscillation against
      # our own recent RES bursts and is unrelated to driver intent.
      suppressed = self_res_recent
      if not suppressed:
        # During the post-engage auto-resume window, fire SET at 150 ms
        # cadence so we can keep up with SCC's 5-8 mph/s autonomous resume
        # ramp. Outside the window, the default 300 ms cadence applies.
        in_auto_resume_guard = (frame - self._engage_frame) < AUTO_RESUME_GUARD_FRAMES
        set_cooldown = AUTO_RESUME_SET_COOLDOWN_FRAMES if in_auto_resume_guard else SET_COOLDOWN_FRAMES
        if (frame - self.last_set_frame) >= set_cooldown:
          if self._consume_global_rate_limit(frame, 1):
            button = Buttons.SET_DECEL
    else:
      # Recovery aims at pre_cap_set (latched at last cap entry, possibly
      # extended by driver RES). Won't fire if pre_cap_set wasn't latched
      # (no cap event), driver SET cleared it, or driver pressed gas/brake.
      recovery_target = self._pre_cap_set_speed
      below_target = (
        recovery_target > 0.5 * MPH_TO_MS
        and observed_set_speed < recovery_target - RECOVERY_DEADBAND_MS
      )
      standstill_clear = (frame - self._left_standstill_at_frame) >= RECOVERY_AFTER_STANDSTILL_FRAMES
      cap_settle_clear = (frame - self._soft_cap_exited_frame) >= RECOVERY_AFTER_CAP_EXIT_FRAMES
      may_recover = (
        below_target
        and not soft_cap
        and not in_override_set
        and not gas_pressed
        and not brake_pressed
        and not self_set_recent
        and v_ego > RECOVERY_V_EGO_FLOOR_MS
        and standstill_clear
        and cap_settle_clear
        and not self._recovery_load_paused
      )
      if may_recover:
        if (frame - self.last_res_frame) >= RES_COOLDOWN_FRAMES:
          if self._consume_global_rate_limit(frame, 1):
            button = Buttons.RES_ACCEL

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
