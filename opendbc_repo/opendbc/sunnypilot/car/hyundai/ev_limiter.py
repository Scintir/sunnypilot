"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

EV Power Limiter — classic-CAN Hyundai HYBRID, stock-long only.

Prevents the ICE from kicking on during ACC by biasing the stock SCC set
speed via CLU11 button injection.

Iter6 (2026-04-28) — sliding-cap rewrite. The state-machine of iters 4-5
(SOFT_CAP entry triggered by power+aBasis, latched pre_cap_set, recovery
gated by load hysteresis, post-cap settle timers) was the wrong abstraction:
- aBasis>0.7 fallback fired on every Hyundai launch (1.5-2 m/s² stock-SCC
  launch accel), forcing 6 s of SOFT_CAP + 13 s of slow recovery on every
  red-light takeoff (drive #5: three 20 s dwells)
- power-threshold-only SOFT_CAP fired AFTER cluster gap had already built
  up SCC's accel demand into the ICE region (drive #5: ICE engaged at
  +1.99 s with cluster_set 13 mph above vEgo for the prior 15 s)

Iter6 replaces all of that with a continuous live cap. Each frame:
  margin = dynamic_margin(vEgo)
    20 mph at vEgo=0 → 5 mph at vEgo>=30, linear in between
  target_set = min(user_target, vEgo + margin)
  push DOWN (SET) when observed > target_set + deadband, OR when est_power
    is above threshold and there's a gap to close (observed > vEgo)
  push UP (RES) when observed < target_set - deadband
The dynamic margin caps SCC's accel demand by construction (small gap at
high speed → small accel command → bounded motor power). No more "cap →
release → recover" cycling.

States (published as evLimiterState uint8 — same enum as iter5; 2 and 3
re-purposed):
  0 IDLE                — at target, no press needed
  1 STANDSTILL_HOLD     — vehicle at/near stop; emit absolutely nothing
  2 SOFT_CAP_ACTIVE     — actively pushing set DOWN (HUD: LIMITING)
  3 RECOVERY_ACTIVE     — actively pushing set UP   (HUD: RECOVERING)
  4 DRIVER_OVERRIDE_SET — driver pressed wheel SET recently; we don't RES
  5 DRIVER_OVERRIDE_RES — driver pressed wheel RES recently
  6 BUS_FAULT_HOLD      — reserved
  7 DISABLED            — not supported / not enabled / CC off

State derivation is latched on recent activity (LIMITING_LATCH_FRAMES /
RECOVERING_LATCH_FRAMES) so HUD doesn't flicker between IDLE and active
on non-press frames. SOFT_CAP_ACTIVE has higher priority than driver
overrides so HUD reflects active control intent (and per drive #4: limiter
SET fires DURING DRIVER_OVERRIDE_RES — the driver-respect window does not
extend to letting motor cross ICE boundary).

What we KEEP from iter4/iter5:
  - Engagement edge fix: DISABLED handled before button processing,
    `_observed_set_speed_at_disable` frozen for engage seed,
    `engage_via_res` classification with disabled-frame lookback (drive #4
    race: RES press can land 1-2 frames before cc_enabled rises)
  - just_engaged skips both accelCruise + decelCruise on the engage frame
    (engage press isn't directional intent)
  - DRIVER_OVERRIDE_RES does NOT suppress our SET (drive #4 lesson)
  - Standstill suppression (no synthetic buttons below ~2 mph)
  - Echo filter (80 ms, first-match) so our own TX doesn't get re-counted
  - Self-direction-block (1.5 s after our SET, block our RES) — anti-osc
  - Quantization observer: 300 ms post-driver-edge window snaps user_target
    to observed if SCC's 5-mph step landed after a held button
  - Global rate limit: 6 LOGICAL presses/sec (burst copies don't count)

What we REMOVED (vs iter5):
  - SOFT_CAP entry/exit state machine, persistence frames, min-dwell
  - aBasis>0.7 fallback trigger (root cause of drive #5 dwells)
  - `_pre_cap_set_speed` latched recovery target (live target instead)
  - Recovery load gate hysteresis (`_recovery_load_paused`)
  - RECOVERY_AFTER_CAP_EXIT_FRAMES post-cap settle timer
  - AUTO_RESUME_GUARD post-engage fast-SET window (now baseline behavior)
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

# iter11 Fix B: set of states the arbiter treats as "active" (subject to
# min-dwell on exit). All other states are quiescent or driver-priority.
ACTIVE_STATES = (STATE_SOFT_CAP_ACTIVE, STATE_RECOVERY_ACTIVE)
STATE_DRIVER_OVERRIDE_SET = 4
STATE_DRIVER_OVERRIDE_RES = 5
STATE_BUS_FAULT_HOLD      = 6
STATE_DISABLED            = 7

# Frame rate — carcontroller runs at 100 Hz.
FRAMES_PER_SEC = 100

# Burst / cadence
BURST_COPIES = 2                         # copies per commanded frame
SET_COOLDOWN_FRAMES = 15                 # 150 ms between commanded SET frames
                                          # (fast pull-down keeps up with SCC's autonomous
                                          # resume ramp ~5-8 mph/s; was 300 ms baseline
                                          # in iter5 with a separate 150 ms guard window —
                                          # consolidating to always-fast since the sliding
                                          # cap pulls down often enough that we need it.)
RES_COOLDOWN_FRAMES = 80                 # 800 ms between commanded RES frames
                                          # (1.25 mph/s gentle pull-up toward user_target;
                                          # user explicitly OK with slow recovery rate.)

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

# Sliding cap (iter6 core mechanism). cluster_set is held within
# `vEgo + dynamic_margin(vEgo, est_power)`. Margin shape:
#   - Wider at low speed so a stored set of e.g. 60 mph at standstill
#     doesn't drive aggressive launch accel.
#   - Tight at highway speed so SCC's accel demand stays bounded.
#   - PLUS a low-load bonus (iter7): when est_power_w is well below the
#     user's threshold, allow extra margin so SCC has room to accelerate
#     toward user_target on flats. Drive #6 confirmed iter6's flat 5 mph
#     cap throttled natural recovery to a crawl — vehicle and cluster_set
#     stuck together at vEgo+5 because SCC's accel demand for a 5 mph gap
#     under low load is essentially zero.
LOW_SPEED_MARGIN_MPH = 20.0              # base margin at vEgo = 0
HIGH_SPEED_MARGIN_MPH = 5.0              # base margin at vEgo >= MARGIN_BLEND_END_MPH
MARGIN_BLEND_END_MPH = 30.0              # vEgo above this uses HIGH_SPEED_MARGIN_MPH; below
                                          # this, base margin interpolates linearly
LOW_LOAD_BONUS_MPH = 10.0                # extra margin when load is well below threshold
LOAD_BONUS_LOW_FRAC = 0.4                # below 40% of threshold = full bonus
LOAD_BONUS_HIGH_FRAC = 0.8               # above 80% of threshold = no bonus
                                          # (linear taper between)

# Standstill SET. Iter7 added SET-only-at-standstill (pull set toward 20 mph
# cap when stopped); iter8 lowered the pulse cap from 30 → 10 because the
# drive #6 simulation suggested Hyundai SCC may ignore subsequent SETs at
# vEgo=0 (real iter6 fired 1 SET → cluster dropped 1 mph → no further
# response observed). Decel-fast cadence (below) does the actual cluster
# pull-down work BEFORE standstill latches, so 10 pulses is plenty as the
# residual safety net.
STANDSTILL_SET_PULSE_CAP = 10            # max synthetic SETs per single standstill window

# Decel-fast SET regime (iter8). When vEgo is in the low-speed range AND
# decelerating, SCC pulls cluster set down via our SET cascade. The default
# 150 ms SET cadence (= 6.7 mph/s pull-down rate) loses the race against
# typical brake-decel of 8-15 mph/s, leaving cluster set high when
# raw_standstill latches. During decel-fast we use a tighter cadence and a
# higher rate-limit ceiling so cluster keeps up with vEgo. Outside this
# regime, default cadence + rate limit apply.
DECEL_FAST_VEGO_THRESHOLD_MS = 30.0 * 0.44704     # below 30 mph
DECEL_FAST_AEGO_MS2 = -0.5                          # noticeable decel (negative aEgo)
SET_COOLDOWN_DECEL_FAST_FRAMES = 6                  # 60 ms minimum spacing between SETs
DECEL_FAST_RATE_LIMIT_PRESSES_PER_SEC = 12          # sustained 12 SETs/sec (=12 mph/s pull-down
                                                     # rate, vs default 6 mph/s); minimum
                                                     # spacing of 60 ms is below this and the
                                                     # rate limit is the binding constraint.
                                                     # Active only during decel-fast — RES
                                                     # isn't fired during decel so no conflict.

# Down-trigger (SET) deadbands
SET_TRIGGER_DEADBAND_MS = 0.5 * 0.44704  # 0.5 mph above target_set before we push down
POWER_GAP_DEADBAND_MS = 0.5 * 0.44704    # observed must be > vEgo + 0.5 mph to attribute high
                                          # power to SCC's accel demand (vs grade/drag/HVAC)

# Up-trigger (RES) constants
RECOVERY_DEADBAND_MS = 1.0 * 0.44704     # 1 mph below target_set before we push up
RECOVERY_V_EGO_FLOOR_MS = 3 * 0.44704    # must be moving (matches former soft-cap floor)

# State-latch durations — HUD/log state is "limiting" or "recovering" if
# we emitted a press recently OR want to emit one this frame. Avoids 100 Hz
# flicker when the underlying button cadence is slower than per-frame.
LIMITING_LATCH_FRAMES = 50               # 500 ms after our last SET counts as LIMITING
RECOVERING_LATCH_FRAMES = 100            # 1 s after our last RES counts as RECOVERING

# iter10 (drive #8) Layer 2: state machine dwell + hysteresis. Drive #8
# 7:36-7:45 oscillation period had 96 state transitions in 9 minutes
# (10.7/min). Without dwell, the soft-cap ↔ recovery loop fires whenever
# estPowerW crosses threshold, which happens ~once/s on grade with the
# noisy LP-filtered observer.
#
# Semantics:
#   IDLE → active state: governed by ENTER_SUSTAIN_FRAMES only (no min-dwell).
#     EXCEPTION: power_too_high bypasses entry sustain → immediate SOFT_CAP
#     entry. Preserves iter9 fast-protection.
#   active → IDLE: requires (time-in-state ≥ MIN_ACTIVE_STATE_DWELL_FRAMES)
#     AND (clear sustained for EXIT_SUSTAIN_FRAMES).
#   active ↔ active (cap ↔ recovery): allowed only after MIN_ACTIVE_STATE_DWELL.
#   any → STANDSTILL_HOLD / DRIVER_OVERRIDE_* / DISABLED: immediate (driver
#     priority states bypass all timers).
#   Sustain counter reset rule: increments while predicate true, resets to 0
#     when predicate false.
#   Cross-predicate priority: if set_too_high AND under_target both true
#     simultaneously after governor clamp, SOFT_CAP wins (down has priority).
MIN_ACTIVE_STATE_DWELL_FRAMES = 200      # 2.0 s minimum in any active state before exit
SOFT_CAP_ENTER_SUSTAIN_FRAMES = 30       # 0.3 s of (set_too_high or power_too_high)
SOFT_CAP_EXIT_SUSTAIN_FRAMES = 200       # 2.0 s of clear before allowing exit
RECOVERY_ENTER_SUSTAIN_FRAMES = 20       # 0.2 s of under_target
RECOVERY_EXIT_SUSTAIN_FRAMES = 100       # 1.0 s of at-target before allowing exit

# iter10 (drive #8) Layer 1: bounded reference governor.
# Replaces iter9's blanket `gas_pressed` block on want_res with mode-based
# bound computation. Prevents Event A (dwell at 21 mph) and Event B
# (cluster < vEgo oscillation) by construction.
#
# Modes (selected exclusively each tick):
GOVERNOR_MODE_NORMAL       = 0   # default: max-deficit + no-below-vEgo invariants
GOVERNOR_MODE_GAS_CATCHUP  = 1   # driver pressing gas — cluster tracks vEgo upward
GOVERNOR_MODE_DECEL        = 2   # SCC commanding decel (lead, brake-recent) — let cluster dip
GOVERNOR_MODE_BRAKE        = 3   # brake pressed — limiter should not act
GOVERNOR_MODE_STANDSTILL   = 4   # at/near standstill — STANDSTILL_HOLD owns it

MAX_DEFICIT_DEFAULT_MPH = 7.0    # default cap on (user_target - cluster_set), UI-tunable
GAS_HEADROOM_MPH = 2.0           # while gas pressed, cluster ≤ vEgo + 2
GAS_CATCHUP_MIN_DEFICIT_MPH = 3.0  # arming: cluster < user_target - 3 mph
GAS_HOLD_MIN_FRAMES = 50         # 0.5 s @ 100 Hz before catch-up arms
SCC_DECEL_DETECT_MS2 = -0.5      # accelDemand below this = SCC commanding decel
SCC_DECEL_PERSIST_FRAMES = 50    # 0.5 s sustain to qualify as decel intent
BRAKE_RECENT_FRAMES = 100        # 1 s after brake release still counts as decel intent
GAS_RES_INTERVAL_FRAMES = 150    # 1.5 s between RES presses while gas held

# UI-tunable param key for max deficit
MAX_DEFICIT_PARAM = "EvLimiterMaxDeficitMph"

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

# iter10: governor constants (require MPH_TO_MS, defined here)
EGO_SLOP_MS = 0.5                # 1.1 mph slop on "cluster ≥ vEgo" invariant
STANDSTILL_V_THRESHOLD_MS = 2 * MPH_TO_MS    # below this = STANDSTILL mode


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

    # STANDSTILL persistence
    self._standstill_trigger_frames = 0
    self._standstill_on = False
    self._left_standstill_at_frame = -10000
    # iter7: bounded SET-pulse counter at standstill (prevents runaway TX
    # if Hyundai SCC ignores SET commands at vEgo=0).
    self._standstill_set_pulses = 0
    self._was_in_standstill_last_frame = False

    # Driver-adjust observation window — see DRIVER_ADJUST_WINDOW_FRAMES doc
    self._driver_adjust_until = -10000

    # Pre-engage observed setpoint — frozen during DISABLED, used to seed
    # user_target on RES re-engage (drive #4: SCC autonomously snaps cluster
    # up on RES; reading post-engage observed gives a polluted seed).
    self._observed_set_speed_at_disable = 0.0
    # Last frame a physical RES press was seen while DISABLED. The engage
    # frame may not contain the press itself if cc_enabled rises 1-2 frames
    # later, so engage classification looks back this far.
    self._last_disabled_res_press_frame = -10000

    # Published burst count (carcontroller reads this each frame)
    self.current_burst_count = BURST_COPIES

    # Debug state (for logging)
    self.state = STATE_DISABLED

    # iter10 Layer 2: state machine dwell + hysteresis (drive #8 Event B fix).
    # Tracks frame of last state transition + sustain counters for predicate
    # debouncing. Resets to 0 when entering states / when predicate clears.
    self._state_entered_frame = 0
    self._softcap_enter_sustain = 0   # increments while (set_too_high or power_too_high)
    self._softcap_exit_sustain = 0    # increments while NOT (set_too_high or power_too_high)
    self._recovery_enter_sustain = 0  # increments while under_target
    self._recovery_exit_sustain = 0   # increments while NOT under_target

    # iter10 Layer 1: bounded reference governor state.
    # Track gas-hold duration (catch-up arming), brake recency, SCC sustained
    # decel (lead-following / commanded slowdown intent), and last cluster set
    # we committed to (gas-catchup lower-bound anchor).
    self._gas_hold_frames = 0
    self._last_brake_frame = -10000
    self._scc_decel_persistent_frames = 0
    self._last_committed_observed_set = 0.0
    # Diagnostic: count bound inversions detected in NORMAL/DECEL modes.
    # MODE_GAS_CATCHUP is non-inverting by construction.
    self._bound_inversion_count = 0

    # iter11 Fix B: arbiter forensic counters
    self._transitions_blocked_by_dwell = 0
    self._transitions_blocked_by_sustain = 0
    self._power_too_high_recent = False   # set true on power_too_high entry, cleared on exit
    self._transition_log = deque(maxlen=10)

    # iter11 Fix A: recovery escape (rate-limited recovery below max-deficit floor)
    self._recovery_escape_active = False
    self._recovery_escape_start_t = 0.0
    self._recovery_escape_start_cluster = 0.0
    self._engaged_at_frame = -10000   # for engagement-transient safety override

    # iter11 Fix D: ineffective-RES watchdog
    self._res_sequence_start_frame = -10000
    self._res_sequence_press_count = 0
    self._res_sequence_start_cluster_ms = 0.0
    self._res_escape_until_frame = -10000
    self._res_escape_start_cluster_ms = 0.0
    self._last_escape_attempt_frame = -10000
    self._ineffective_res_events = 0

    # iter11 Fix F: highway SET cooldown counter
    self._power_high_pending_frames = 0

    # iter11 Fix B: standstill 10-frame hysteresis
    self._standstill_entry_frames = 0
    self._standstill_exit_frames = 0

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

  def _consume_global_rate_limit(self, frame: int, n_logical: int,
                                  limit: int = GLOBAL_RATE_LIMIT_PRESSES_PER_SEC) -> bool:
    """Return True and record the TX if adding `n_logical` logical button
    commands in the last 1 s stays at or under `limit`.

    Burst copies are reliability duplicates (same logical press repeated for
    the cluster to see), so callers pass 1 per logical command — not BURST_COPIES.

    iter8: callers can pass a higher `limit` during the decel-fast regime
    (DECEL_FAST_RATE_LIMIT_PRESSES_PER_SEC = 12) so cluster pull-down can
    keep up with hard braking. Default behavior unchanged.
    """
    cutoff = frame - FRAMES_PER_SEC
    while self.press_history and self.press_history[0][0] < cutoff:
      self.press_history.popleft()
    total = sum(c for _, c in self.press_history)
    if total + n_logical > limit:
      return False
    self.press_history.append((frame, n_logical))
    return True

  def _process_button_events(self, CS, observed_set_speed: float, frame: int,
                              just_engaged: bool) -> None:
    """Feed driver wheel input into user_target + driver-override windows.

    First matching event within ECHO_FILTER_FRAMES of our TX is swallowed
    as our own echo; subsequent events pass through. Physical driver edges
    adjust user_target by ±1 mph each. The SCC's 5-mph quantization step
    that follows a held button is caught later in _maybe_observe_quantization
    (display tracking only — never used as a control input).

    Per drive #4 fix: on the just_engaged frame, accelCruise + decelCruise
    events are the user enabling cruise (RES re-engage / SET-from-off), NOT
    directional intent. Skip them entirely so we don't open driver-override
    windows that survive into the engaged session.
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
        continue
      if event.type == ButtonType.decelCruise:
        self.user_target_speed -= MPH_TO_MS
        self._driver_set_last_frame = frame
        self._driver_adjust_until = frame + DRIVER_ADJUST_WINDOW_FRAMES
      elif event.type == ButtonType.accelCruise:
        self.user_target_speed += MPH_TO_MS
        self._driver_res_last_frame = frame
        self._driver_adjust_until = frame + DRIVER_ADJUST_WINDOW_FRAMES
      # cancel handled by cc_enabled going False on the next frame
    self.user_target_speed = max(USER_TARGET_MIN_MS, min(USER_TARGET_MAX_MS, self.user_target_speed))

  def _maybe_observe_quantization(self, observed_set_speed: float, frame: int) -> None:
    """During the 300 ms after a real driver edge, watch for the Hyundai SCC's
    delayed +5 mph quantization step (held button → cluster bumps observed
    by 5 mph a few frames after our buttonEvent edge). Snap user_target to
    observed so the HUD `User XX mph` line stays consistent with what the
    driver actually got. Display tracking only — never used as a control
    input.
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

  @staticmethod
  def _dynamic_margin_ms(v_ego_ms: float, est_power_w: float, power_threshold_w: float) -> float:
    """Sliding cap formula with iter7 power-aware bonus.

    Base margin (vEgo-only):
      vEgo = 0       → LOW_SPEED_MARGIN_MPH (20 mph default)
      vEgo = 30 mph  → HIGH_SPEED_MARGIN_MPH (5 mph default)
      vEgo > 30 mph  → flat at HIGH_SPEED_MARGIN_MPH

    Low-load bonus (iter7 dwell fix):
      load_frac <= LOAD_BONUS_LOW_FRAC (0.4)  → +LOW_LOAD_BONUS_MPH (10 mph)
      load_frac >= LOAD_BONUS_HIGH_FRAC (0.8) → +0 mph
      between                                 → linear taper

    Fail safe: if power_threshold_w is invalid (≤0), bonus is 0 — never
    treat invalid threshold as low load.
    """
    v_ego_mph = v_ego_ms / MPH_TO_MS
    if v_ego_mph >= MARGIN_BLEND_END_MPH:
      base_mph = HIGH_SPEED_MARGIN_MPH
    elif v_ego_mph <= 0.0:
      base_mph = LOW_SPEED_MARGIN_MPH
    else:
      frac = v_ego_mph / MARGIN_BLEND_END_MPH
      base_mph = LOW_SPEED_MARGIN_MPH + frac * (HIGH_SPEED_MARGIN_MPH - LOW_SPEED_MARGIN_MPH)

    if power_threshold_w <= 0.0:
      bonus_mph = 0.0
    else:
      load_frac = max(0.0, est_power_w) / power_threshold_w
      if load_frac <= LOAD_BONUS_LOW_FRAC:
        bonus_mph = LOW_LOAD_BONUS_MPH
      elif load_frac >= LOAD_BONUS_HIGH_FRAC:
        bonus_mph = 0.0
      else:
        taper = (LOAD_BONUS_HIGH_FRAC - load_frac) / (LOAD_BONUS_HIGH_FRAC - LOAD_BONUS_LOW_FRAC)
        bonus_mph = LOW_LOAD_BONUS_MPH * taper

    margin_mph = base_mph + bonus_mph
    # Defensive clamp — should never trigger given the math above, but cheap insurance.
    if margin_mph < base_mph:
      margin_mph = base_mph
    elif margin_mph > base_mph + LOW_LOAD_BONUS_MPH:
      margin_mph = base_mph + LOW_LOAD_BONUS_MPH
    return margin_mph * MPH_TO_MS

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

  # ----- iter10 Layer 1: bounded reference governor -----------------------

  def _has_decel_intent(self, frame: int) -> bool:
    """Return True iff POSITIVE evidence of intentional decel.
    Conservative: uncertain → False (caller defaults to MODE_NORMAL, which
    enforces the no-cluster-below-vEgo invariant). v2 critique resolved.
    Lead-radar TTC clause is plumbed via radar_decel_intent set by card.py
    (defaults to None when unavailable on classic-CAN HYBRID).
    """
    if (frame - self._last_brake_frame) < BRAKE_RECENT_FRAMES:
      return True
    if self._scc_decel_persistent_frames >= SCC_DECEL_PERSIST_FRAMES:
      return True
    return False

  def _select_governor_mode(self, frame: int, v_ego: float, gas_pressed: bool,
                             brake_pressed: bool, in_standstill: bool,
                             observed_set_speed: float) -> int:
    """Select exclusive governor mode for this tick. Order matters:
    standstill > brake > gas-catchup (if armed) > decel-intent > normal."""
    if in_standstill or v_ego < STANDSTILL_V_THRESHOLD_MS:
      return GOVERNOR_MODE_STANDSTILL
    if brake_pressed:
      return GOVERNOR_MODE_BRAKE
    deficit_ms = self.user_target_speed - observed_set_speed
    catchup_armed = (
      gas_pressed
      and self._gas_hold_frames >= GAS_HOLD_MIN_FRAMES
      and deficit_ms > GAS_CATCHUP_MIN_DEFICIT_MPH * MPH_TO_MS
    )
    if catchup_armed:
      return GOVERNOR_MODE_GAS_CATCHUP
    if self._has_decel_intent(frame):
      return GOVERNOR_MODE_DECEL
    return GOVERNOR_MODE_NORMAL

  def _compute_governor_bounds(self, mode: int, v_ego: float, frame: int,
                                observed_set_speed: float,
                                dynamic_ceiling: float) -> tuple[float, float]:
    """iter11 Fix A: max-deficit invariant is HARD in NORMAL mode, subject
    only to enumerated safety overrides:
    - lead/brake/decel handled via MODE_DECEL/MODE_BRAKE selection upstream
    - engagement transient (just engaged or vEgo too low to act) suspends floor

    Drive #9-13 forensics: the v0/iter10 'low-speed dynamic ceiling' suspension
    turned a hard invariant into a conditional and produced 22-44 mph offsets.
    iter11: floor is hard. If sliding cap wants cluster lower, it loses to
    the floor (cluster simply won't be cap-down'd below floor)."""
    upper_bound = self.user_target_speed
    max_deficit_mph = self._read_int(MAX_DEFICIT_PARAM, int(MAX_DEFICIT_DEFAULT_MPH))
    default_lower = max(USER_TARGET_MIN_MS, self.user_target_speed - max_deficit_mph * MPH_TO_MS)

    if mode == GOVERNOR_MODE_NORMAL:
      # iter11 Fix A: engagement-transient safety override. Lets cluster sit
      # wherever it lands at engage edge (probably below floor) for first 1.0 s
      # before strict floor enforcement.
      ENGAGEMENT_TRANSIENT_FRAMES = 100   # 1.0 s @ 100 Hz
      is_engagement_transient = (
        not self.was_cc_enabled
        or (frame - self._engaged_at_frame) < ENGAGEMENT_TRANSIENT_FRAMES
      )

      if is_engagement_transient:
        lower_bound = USER_TARGET_MIN_MS
      else:
        # HARD invariant: cluster ≥ user_target - max_deficit, always.
        lower_bound = default_lower

      # Invariant: never command cluster < vEgo - slop when user_target above ego.
      # Prevents Event B (cluster<vEgo oscillation).
      if self.user_target_speed > v_ego + EGO_SLOP_MS and not is_engagement_transient:
        lower_bound = max(lower_bound, v_ego - EGO_SLOP_MS)
        if upper_bound < lower_bound:
          upper_bound = lower_bound
          self._bound_inversion_count += 1

    elif mode == GOVERNOR_MODE_GAS_CATCHUP:
      # v2 fix: non-inverting by construction.
      default_upper = min(self.user_target_speed, v_ego + GAS_HEADROOM_MPH * MPH_TO_MS)
      lower_bound = max(USER_TARGET_MIN_MS, observed_set_speed)
      upper_bound = max(default_upper, observed_set_speed)
      # If cluster < vEgo+headroom: upper=vEgo+2, lower=observed → catch-up via RES (Event A).
      # If cluster > vEgo+headroom: upper=lower=observed → cluster holds (no RES, no SET).

    elif mode == GOVERNOR_MODE_DECEL:
      # SCC commanding decel — let cluster dip below vEgo for legitimate slowdown.
      lower_bound = USER_TARGET_MIN_MS

    elif mode == GOVERNOR_MODE_BRAKE:
      lower_bound = USER_TARGET_MIN_MS

    else:  # GOVERNOR_MODE_STANDSTILL
      lower_bound = USER_TARGET_MIN_MS

    if lower_bound > upper_bound:
      # Defensive: should only happen in NORMAL with degenerate user_target/vEgo.
      # Saturate to permissive bounds and log.
      self._bound_inversion_count += 1
      lower_bound = USER_TARGET_MIN_MS

    return lower_bound, upper_bound

  # ----- Main update ------------------------------------------------------

  def update(self, CC, CS, frame: int) -> tuple[int, bool]:
    """Advance the limiter one tick. Return (button, active)."""

    if not self.supported:
      return self._publish(Buttons.NONE, STATE_DISABLED, 0.0, frame=frame, reason="unsupported")

    if not self._read_bool("EVLimiterEnabled", False):
      self.user_target_speed = 0.0
      return self._publish(Buttons.NONE, STATE_DISABLED, 0.0, frame=frame, reason="disabled_param")

    # Tunables (read every frame so params changes take effect live)
    power_threshold_w = float(self._read_int("EVLimiterPowerThresholdKW", 40)) * 1000.0
    dte_floor = float(self._read_int("EVLimiterDTEFloor", 5))

    # Inputs
    cc_enabled = bool(CC.enabled)
    v_ego = float(CS.out.vEgo)
    a_ego = float(CS.out.aEgo)
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
      self._standstill_trigger_frames = 0
      self.was_cc_enabled = cc_enabled
      return self._publish(Buttons.NONE, STATE_DISABLED, observed_set_speed,
                           frame=frame, reason="cc_off")

    # cc_enabled is True from here on.
    # Engage rising edge -> seed user_target.
    just_engaged = not self.was_cc_enabled
    if just_engaged:
      self._engaged_at_frame = frame   # iter11 Fix A: engagement-transient timing
      # Engage via RES? Check current-frame buttonEvents AND recent
      # disabled-frame RES presses (the wheel press may have landed before
      # cc_enabled rose). Engage via SET / main-switch leaves observed
      # clean (no SCC autonomous resume snap), so we only redirect the
      # seed for engage_via_res cases.
      engage_via_res = False
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
      return self._publish(Buttons.NONE, STATE_DISABLED, observed_set_speed,
                           frame=frame, reason="dte_floor")

    # Standstill gate. raw_standstill fires immediately (no 20-frame confirm)
    # so engaging while already stopped — or a brief sub-2 mph dip — suppresses
    # RES in-frame. The confirm-based `_standstill_on` continues to track the
    # post-standstill-clear hold-off (RECOVERY_AFTER_STANDSTILL).
    #
    # iter7: SET is allowed at standstill (pulse-bounded) so cluster_set can
    # be pulled down toward LOW_SPEED_MARGIN_MPH (20 mph) at red lights,
    # rather than being frozen wherever the deceleration SET cascade landed
    # when raw_standstill kicked in (drive #6: cluster frozen at 31 with
    # user_target=62). RES remains blocked at standstill.
    raw_standstill = (
      v_ego < STANDSTILL_V_EGO_MS
      or standstill_flag
      or (brake_pressed and v_ego < BRAKE_LOW_SPEED_V_EGO_MS)
    )
    standstill = self._update_standstill(v_ego, brake_pressed, standstill_flag)
    in_standstill = raw_standstill or standstill

    # Reset SET-pulse counter on each entry to standstill.
    if in_standstill and not self._was_in_standstill_last_frame:
      self._standstill_set_pulses = 0
    self._was_in_standstill_last_frame = in_standstill

    if in_standstill:
      self._left_standstill_at_frame = frame
      # Allow SET only when cluster set is meaningfully above the standstill cap.
      standstill_cap_ms = LOW_SPEED_MARGIN_MPH * MPH_TO_MS
      set_above_cap = observed_set_speed > standstill_cap_ms + SET_TRIGGER_DEADBAND_MS
      if (
        set_above_cap
        and not gas_pressed
        and not brake_pressed
        and self._standstill_set_pulses < STANDSTILL_SET_PULSE_CAP
        and (frame - self.last_set_frame) >= SET_COOLDOWN_FRAMES
        and self._consume_global_rate_limit(frame, 1)
      ):
        self._standstill_set_pulses += 1
        self._record_tx(frame, Buttons.SET_DECEL)
        return self._publish(Buttons.SET_DECEL, STATE_SOFT_CAP_ACTIVE, observed_set_speed,
                             frame=frame, reason="standstill_set_pulse")
      # Don't reset last_set_frame — the cooldown gate above relies on it
      # to space SET fires properly within the standstill window. The
      # consequence is up to 1.5 s of self_set_recent post-exit blocking
      # RES, which is acceptable since vehicle is just leaving stop and
      # SCC's accel demand will lift cluster_set via the sliding-cap path.
      return self._publish(Buttons.NONE, STATE_STANDSTILL_HOLD, observed_set_speed,
                           frame=frame, reason="standstill")

    # Driver override windows.
    in_override_set = self._in_driver_override_set(frame)
    in_override_res = self._in_driver_override_res(frame)

    # One-direction-at-a-time anti-oscillation: after our SET, block our RES
    # for 1.5 s. Same symmetrically for RES blocking SET.
    self_set_recent = (frame - self.last_set_frame) < LIMITER_OPPOSITE_DIR_BLOCK_FRAMES
    self_res_recent = (frame - self.last_res_frame) < LIMITER_OPPOSITE_DIR_BLOCK_FRAMES

    # iter8: decel-fast regime. When in low-speed range AND decelerating, use
    # tighter SET cadence + higher rate-limit so cluster pull-down keeps up
    # with brake decel (typical 8-15 mph/s). Default 6.7 mph/s pull-down lost
    # the race in drive #6 (cluster frozen at 31 when raw_standstill latched).
    # Note: aEgo is the kinematic ground-frame accel from wheel speed, so a
    # negative value means actual decel regardless of cause (driver brake,
    # SCC command, coasting downhill).
    decel_active = (v_ego < DECEL_FAST_VEGO_THRESHOLD_MS) and (a_ego < DECEL_FAST_AEGO_MS2)
    set_cooldown_frames = SET_COOLDOWN_DECEL_FAST_FRAMES if decel_active else SET_COOLDOWN_FRAMES
    rate_limit = DECEL_FAST_RATE_LIMIT_PRESSES_PER_SEC if decel_active else GLOBAL_RATE_LIMIT_PRESSES_PER_SEC

    # iter10 Layer 1: update governor state-tracking counters every tick.
    if gas_pressed:
      self._gas_hold_frames += 1
    else:
      self._gas_hold_frames = 0
    if brake_pressed:
      self._last_brake_frame = frame
    # SCC decel detected via accelDemand (TCS13.aBasis aggregate). Available
    # on CarState.accel_demand — extracted by carstate_ext upstream.
    abasis_signal = float(getattr(CS, "accel_demand", 0.0))
    if abasis_signal < SCC_DECEL_DETECT_MS2:
      self._scc_decel_persistent_frames += 1
    else:
      self._scc_decel_persistent_frames = 0

    # Sliding cap (iter6 core). Cluster set is held within
    #   target_set = min(user_target, vEgo + dynamic_margin(vEgo))
    # Push DOWN when observed > target_set + deadband (sliding cap violation)
    #   OR when est_power > threshold AND there's a gap to close (load gate).
    # Push UP when observed < target_set - deadband (gentle recovery).
    margin = self._dynamic_margin_ms(v_ego, est_power_w, power_threshold_w)
    dynamic_ceiling = v_ego + margin
    target_set = min(self.user_target_speed, dynamic_ceiling)

    # iter10 Layer 1: clamp target_set to bounded-governor [lower, upper].
    # Mode selection runs each tick; bounds enforce invariants that prevent
    # Event A (dwell) and Event B (cluster < vEgo) by construction.
    governor_mode = self._select_governor_mode(
      frame, v_ego, gas_pressed, brake_pressed, in_standstill, observed_set_speed
    )
    lower_bound, upper_bound = self._compute_governor_bounds(
      governor_mode, v_ego, frame, observed_set_speed, dynamic_ceiling
    )
    target_set = max(lower_bound, min(upper_bound, target_set))

    # iter11 Fix A: max-deficit violation forensic counter
    if governor_mode == GOVERNOR_MODE_NORMAL and not gas_pressed and not brake_pressed:
      if observed_set_speed < lower_bound - 0.5:   # any sustained dip below floor
        if not hasattr(self, '_max_deficit_violation_frames'):
          self._max_deficit_violation_frames = 0
        self._max_deficit_violation_frames += 1

    set_too_high = observed_set_speed > target_set + SET_TRIGGER_DEADBAND_MS
    power_too_high = (
      est_power_w > power_threshold_w
      and observed_set_speed > v_ego + POWER_GAP_DEADBAND_MS
    )
    # iter11 Fix B: arbiter reads this to allow IDLE→SOFT_CAP without sustain
    # (preserves iter9 fast-protect on power_too_high)
    self._power_too_high_recent = power_too_high
    under_target = observed_set_speed < target_set - RECOVERY_DEADBAND_MS

    # iter11 Fix A: recovery escape — when cluster < lower_bound, RES is
    # allowed even with power_too_high, but rate-limited to ≤2 mph/s elapsed.
    cluster_below_floor = observed_set_speed < lower_bound - 0.1   # 0.1 m/s slop
    if cluster_below_floor and not self._recovery_escape_active:
      # Entering escape mode
      self._recovery_escape_active = True
      self._recovery_escape_start_t = frame * 0.01   # 100 Hz → seconds
      self._recovery_escape_start_cluster = observed_set_speed
    elif not cluster_below_floor and self._recovery_escape_active:
      self._recovery_escape_active = False

    # Down-trigger: fires whenever sliding cap is violated OR load is high.
    # Driver overrides do NOT suppress us:
    # - DRIVER_OVERRIDE_RES (drive #4 lesson): respect window doesn't extend
    #   to letting motor cross ICE boundary
    # - DRIVER_OVERRIDE_SET: driver SET is the SAME direction as our SET,
    #   so silencing our SET during a driver-SET window would just leave a
    #   gap if conditions still warrant pulling cluster set down
    # iter9: anti-oscillation block (`self_res_recent`) is bypassed when
    # `power_too_high` — drive #7 forensics showed 53% of high-power frames
    # were stuck in RECOVERY because the limiter had recently pressed RES,
    # while estPowerW was already in ICE territory. Power protection beats
    # cosmetic anti-oscillation.
    suppress_set = self_res_recent and not power_too_high
    want_set = (
      (set_too_high or power_too_high)
      and not suppress_set
      and not gas_pressed
      and not brake_pressed
    )

    # Up-trigger: gentle recovery toward target_set when below. Mutually
    # exclusive with want_set — never both same frame. Also explicitly
    # cancelled by power_too_high (defense-in-depth: even if some other
    # gate cleared want_set, RES toward user_target is wrong when motor
    # is already at ICE-territory power).
    # iter10 Layer 1: removed `not gas_pressed` from this chain. Gas behavior
    # is now governed by mode selection + bound clamping (Event A fix). The
    # MODE_GAS_CATCHUP upper-bound (vEgo + 2 mph) prevents over-recovery while
    # gas is held — RES will track vEgo upward but won't overshoot.
    standstill_clear = (frame - self._left_standstill_at_frame) >= RECOVERY_AFTER_STANDSTILL_FRAMES

    # iter11 Fix A: recovery escape — when cluster is below max-deficit floor,
    # power_too_high no longer cancels want_res. Recovery toward floor is
    # escape priority. Rate-limited via permitted-cluster check below.
    want_res = (
      under_target
      and not want_set
      and (not power_too_high or self._recovery_escape_active)
      and not in_override_set
      and not self_set_recent
      and not brake_pressed
      and v_ego > RECOVERY_V_EGO_FLOOR_MS
      and standstill_clear
    )

    # iter11 Fix A: rate-limited recovery — max 2 mph/sec elapsed-time-based.
    # When in escape mode, throttle RES so cluster gain never exceeds 2 mph/sec
    # since escape entry. Prevents jumpy recovery if SCC is responsive.
    if want_res and self._recovery_escape_active:
      elapsed_s = max(0.001, frame * 0.01 - self._recovery_escape_start_t)
      max_permitted_cluster = self._recovery_escape_start_cluster + 2.0 * MPH_TO_MS * elapsed_s
      if observed_set_speed >= max_permitted_cluster:
        want_res = False   # rate-limit hit; wait

    # iter10 Layer 1: gas-time RES rate cap. While gas held, throttle RES
    # cadence to one press per 1.5 s (vs 6/s normal global limit) so cluster
    # ratchets slowly behind vEgo, not in lockstep. Prevents over-recovery /
    # cluster jumping ahead.
    if want_res and gas_pressed:
      if (frame - self.last_res_frame) < GAS_RES_INTERVAL_FRAMES:
        want_res = False

    # iter11 Fix F: highway SET cadence. On highway (vEgo > 25 m/s), force
    # 5-sec SET cooldown + burst=1, NO bypass for power_too_high (only counter).
    HIGHWAY_VEGO_THRESHOLD_MS = 25.0
    SET_COOLDOWN_HIGHWAY_FRAMES = 500
    is_highway = v_ego > HIGHWAY_VEGO_THRESHOLD_MS
    if is_highway:
      if want_set and (frame - self.last_set_frame) < SET_COOLDOWN_HIGHWAY_FRAMES:
        want_set = False
        if power_too_high:
          self._power_high_pending_frames += 1
      effective_burst = 1   # force single-frame SET on highway
    else:
      effective_burst = BURST_COPIES

    # iter11 Fix D: ineffective-RES watchdog escape. When in escape window,
    # send continuous-frame RES (every tick) instead of discrete bursts.
    in_res_escape = frame < self._res_escape_until_frame
    if in_res_escape:
      # Hard-cap: abort escape if cluster already moved up by 3 mph
      MAX_ESCAPE_DELTA_MS = 3.0 * MPH_TO_MS
      if observed_set_speed - self._res_escape_start_cluster_ms >= MAX_ESCAPE_DELTA_MS:
        self._res_escape_until_frame = -10000
        in_res_escape = False

    button = Buttons.NONE
    if want_set and (frame - self.last_set_frame) >= (
        SET_COOLDOWN_HIGHWAY_FRAMES if is_highway else set_cooldown_frames):
      if self._consume_global_rate_limit(frame, 1, limit=rate_limit):
        button = Buttons.SET_DECEL
        self.current_burst_count = effective_burst
    elif in_res_escape:
      # Continuous-frame RES during escape (no cooldown)
      button = Buttons.RES_ACCEL
      self.current_burst_count = 1
    elif want_res and (frame - self.last_res_frame) >= RES_COOLDOWN_FRAMES:
      if self._consume_global_rate_limit(frame, 1):
        button = Buttons.RES_ACCEL
        # iter11 Fix D: track RES sequence for ineffective-button detection
        if self._res_sequence_press_count == 0:
          self._res_sequence_start_frame = frame
          self._res_sequence_start_cluster_ms = observed_set_speed
        self._res_sequence_press_count += 1

    # iter11 Fix D: detect ineffective RES sequence and trigger escape
    INEFFECTIVE_RES_WINDOW_FRAMES = 1000   # 10 s
    INEFFECTIVE_RES_MIN_PRESSES = 8
    INEFFECTIVE_RES_MIN_GAIN_MS = 1.0 * MPH_TO_MS
    INEFFECTIVE_RES_HOLD_FRAMES = 50       # 0.5 s continuous press
    INEFFECTIVE_RES_BACKOFF_FRAMES = 500   # 5 s between attempts
    if (self._res_sequence_press_count >= INEFFECTIVE_RES_MIN_PRESSES
        and (frame - self._res_sequence_start_frame) <= INEFFECTIVE_RES_WINDOW_FRAMES
        and (observed_set_speed - self._res_sequence_start_cluster_ms) < INEFFECTIVE_RES_MIN_GAIN_MS
        and (frame - self._last_escape_attempt_frame) > INEFFECTIVE_RES_BACKOFF_FRAMES
        and not in_res_escape):
      # Trigger ineffective-RES escape
      self._res_escape_until_frame = frame + INEFFECTIVE_RES_HOLD_FRAMES
      self._res_escape_start_cluster_ms = observed_set_speed
      self._last_escape_attempt_frame = frame
      self._ineffective_res_events += 1
      # Reset sequence
      self._res_sequence_press_count = 0
      self._res_sequence_start_frame = -10000

    # Reset RES sequence tracking on state/mode changes (handled implicitly:
    # any frame without RES press resets press_count IF we've moved out of
    # the window). Reset on override or brake or target change:
    if (in_override_set or in_override_res or brake_pressed
        or gas_pressed or self._res_sequence_press_count == 0):
      pass   # sequence already reset or invalid; do nothing

    if button != Buttons.NONE:
      self._record_tx(frame, button)

    # iter10 Layer 2: dwell + hysteresis state derivation.
    # Update sustain counters every frame.
    softcap_pred = set_too_high or power_too_high
    recovery_pred = under_target and not softcap_pred
    if softcap_pred:
      self._softcap_enter_sustain += 1
      self._softcap_exit_sustain = 0
    else:
      self._softcap_enter_sustain = 0
      self._softcap_exit_sustain += 1
    if recovery_pred:
      self._recovery_enter_sustain += 1
      self._recovery_exit_sustain = 0
    else:
      self._recovery_enter_sustain = 0
      self._recovery_exit_sustain += 1

    cur_state = self.state
    time_in_state = frame - self._state_entered_frame
    new_state = cur_state  # default: hold

    # Driver-priority states bypass dwell/sustain entirely (immediate).
    # IDLE → active uses sustain only (no min-dwell delay on entry).
    # active → IDLE/active needs (min-dwell) AND (exit sustain).
    # If we reach this block coming from DISABLED/STANDSTILL_HOLD/BUS_FAULT,
    # we just left those (engage / left-standstill / bus-recovery) — treat
    # like IDLE-entry for state derivation purposes.
    if cur_state in (STATE_DISABLED, STATE_STANDSTILL_HOLD, STATE_BUS_FAULT_HOLD):
      cur_state = STATE_IDLE

    if cur_state == STATE_IDLE:
      # Power-too-high bypasses entry sustain — preserves iter9 fast-protect.
      if want_set or (frame - self.last_set_frame) < LIMITING_LATCH_FRAMES:
        new_state = STATE_SOFT_CAP_ACTIVE
      elif power_too_high:
        new_state = STATE_SOFT_CAP_ACTIVE
      elif self._softcap_enter_sustain >= SOFT_CAP_ENTER_SUSTAIN_FRAMES:
        new_state = STATE_SOFT_CAP_ACTIVE
      elif want_res or (frame - self.last_res_frame) < RECOVERING_LATCH_FRAMES:
        new_state = STATE_RECOVERY_ACTIVE
      elif self._recovery_enter_sustain >= RECOVERY_ENTER_SUSTAIN_FRAMES:
        new_state = STATE_RECOVERY_ACTIVE
      elif in_override_set:
        new_state = STATE_DRIVER_OVERRIDE_SET
      elif in_override_res:
        new_state = STATE_DRIVER_OVERRIDE_RES

    elif cur_state == STATE_SOFT_CAP_ACTIVE:
      # Driver/override states: immediate (priority).
      if in_override_set:
        new_state = STATE_DRIVER_OVERRIDE_SET
      # Min-dwell + exit-sustain required to leave SOFT_CAP.
      elif (time_in_state >= MIN_ACTIVE_STATE_DWELL_FRAMES
            and self._softcap_exit_sustain >= SOFT_CAP_EXIT_SUSTAIN_FRAMES):
        # Exit allowed: prefer RECOVERY if predicate sustained, else IDLE.
        if self._recovery_enter_sustain >= RECOVERY_ENTER_SUSTAIN_FRAMES:
          new_state = STATE_RECOVERY_ACTIVE
        else:
          new_state = STATE_IDLE

    elif cur_state == STATE_RECOVERY_ACTIVE:
      # Driver SET (immediate) or sustained SOFT_CAP predicate trumps recovery.
      if in_override_set:
        new_state = STATE_DRIVER_OVERRIDE_SET
      elif power_too_high:
        # Immediate SOFT_CAP entry on power excess (preserves fast-protect).
        new_state = STATE_SOFT_CAP_ACTIVE
      elif (time_in_state >= MIN_ACTIVE_STATE_DWELL_FRAMES
            and self._recovery_exit_sustain >= RECOVERY_EXIT_SUSTAIN_FRAMES):
        if self._softcap_enter_sustain >= SOFT_CAP_ENTER_SUSTAIN_FRAMES:
          new_state = STATE_SOFT_CAP_ACTIVE
        else:
          new_state = STATE_IDLE

    elif cur_state == STATE_DRIVER_OVERRIDE_SET:
      # Override windows are external-driven; let existing latch decide.
      if not in_override_set:
        new_state = STATE_IDLE

    elif cur_state == STATE_DRIVER_OVERRIDE_RES:
      if not in_override_res:
        new_state = STATE_IDLE

    # else (STANDSTILL_HOLD, DISABLED, BUS_FAULT_HOLD): handled before this
    # block returns from earlier branches in update().

    # iter11 Fix B: arbiter handles _state_entered_frame mutation; remove
    # the manual update here. Pass hard_preempt=False so arbiter enforces
    # min-dwell on active-state exits.
    # iter11 Fix F: pass effective_burst (1 on highway, BURST_COPIES otherwise)
    return self._publish(button, new_state, observed_set_speed,
                         frame=frame, reason="derived", hard_preempt=False,
                         burst_count=effective_burst if button == Buttons.SET_DECEL else None)

  # ----- iter11 Fix B: centralized state arbiter --------------------------

  def _arbitrate_state_transition(self, frame: int, requested: int, reason: str,
                                   hard_preempt: bool = False) -> int:
    """SOLE site that mutates self.state. Returns actual state after arbitration.
    Hard preempts (driver override, brake, disable, bus fault, standstill-with-
    hysteresis) always succeed. Active-state exits require min-dwell. Active-
    state entries from IDLE require enter sustain (unless hard preempt or
    power_too_high — preserves iter9 fast-protect)."""
    cur = self.state
    if cur == requested:
      return cur

    blocked_by = None
    allow = False

    if hard_preempt:
      allow = True
    elif cur in ACTIVE_STATES:
      # Min-dwell on active-state EXIT
      time_in_state = frame - self._state_entered_frame
      if time_in_state >= MIN_ACTIVE_STATE_DWELL_FRAMES:
        allow = True
      else:
        blocked_by = f"min_dwell_{MIN_ACTIVE_STATE_DWELL_FRAMES - time_in_state}f"
        self._transitions_blocked_by_dwell += 1
    else:
      # IDLE → active needs sustain (unless power_too_high recent for SOFT_CAP)
      if cur == STATE_IDLE and requested == STATE_SOFT_CAP_ACTIVE:
        if (self._softcap_enter_sustain >= SOFT_CAP_ENTER_SUSTAIN_FRAMES
            or self._power_too_high_recent):
          allow = True
        else:
          blocked_by = "softcap_enter_sustain"
          self._transitions_blocked_by_sustain += 1
      elif cur == STATE_IDLE and requested == STATE_RECOVERY_ACTIVE:
        if self._recovery_enter_sustain >= RECOVERY_ENTER_SUSTAIN_FRAMES:
          allow = True
        else:
          blocked_by = "recovery_enter_sustain"
          self._transitions_blocked_by_sustain += 1
      else:
        allow = True   # any other transition (IDLE→DISABLED, IDLE→OVR, etc) allowed

    actual = requested if allow else cur
    self._record_transition(frame, cur, requested, actual, reason, blocked_by)
    if allow:
      self._state_entered_frame = frame
      self.state = actual   # SOLE state mutation (besides __init__)
    return actual

  def _record_transition(self, frame: int, prev: int, requested: int, actual: int,
                          reason: str, blocked_by: str | None) -> None:
    """Append to bounded ring of recent transitions for forensic publish."""
    if not hasattr(self, '_transition_log'):
      self._transition_log = deque(maxlen=10)
    if prev != actual:   # only log actual transitions, not blocked ones
      self._transition_log.append(f"{frame},{prev},{actual},{reason[:8]}")

  # ----- Publish ----------------------------------------------------------

  def _publish(self, button: int, state: int, observed_set_speed: float,
               frame: int = 0, reason: str = "publish",
               hard_preempt: bool = True,
               burst_count: int | None = None) -> tuple[int, bool]:
    """Publish-and-arbitrate. iter11: state changes route through the arbiter
    even from this entry point. `hard_preempt=True` by default because most
    publish callers are early-return paths (DISABLED, STANDSTILL, BUS_FAULT)
    which legitimately preempt. The state-derivation block in update() passes
    hard_preempt=False explicitly. `burst_count=None` keeps default BURST_COPIES;
    iter11 highway SET path passes burst_count=1 to suppress multi-frame burst."""
    actual = self._arbitrate_state_transition(frame, state, reason, hard_preempt)
    active = actual in (STATE_SOFT_CAP_ACTIVE, STATE_RECOVERY_ACTIVE)
    self.current_burst_count = burst_count if burst_count is not None else BURST_COPIES
    _SHARED_STATE["active"] = bool(active)
    _SHARED_STATE["set_speed_offset"] = max(0.0, self.user_target_speed - observed_set_speed)
    _SHARED_STATE["user_target"] = float(self.user_target_speed)
    _SHARED_STATE["state"] = int(actual)
    return button, active
