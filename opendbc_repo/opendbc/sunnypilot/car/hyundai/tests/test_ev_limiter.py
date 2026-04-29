"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Regression coverage for the EV power limiter — sliding-cap iter6 architecture.

Drive #4 (kept): RES re-engage doesn't get classified as driver intent
to accelerate; user_target seeds from frozen pre-disable observed.

Drive #5 (new in iter6): the previous SOFT_CAP/RECOVERY state machine fired
on every Hyundai launch (aBasis>0.7 fallback) causing 20 s dwells, and let
cluster set sit 13 mph above user_target for 15 s before SOFT_CAP fired
power-only — too late, ICE engaged. Sliding cap addresses both by holding
cluster set within `vEgo + dynamic_margin(vEgo)` continuously.
"""

import unittest
from dataclasses import dataclass, field
from typing import Any

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.ev_limiter import (
  EVLimiter,
  STATE_DISABLED,
  STATE_DRIVER_OVERRIDE_RES,
  STATE_IDLE,
  STATE_SOFT_CAP_ACTIVE,
  STATE_RECOVERY_ACTIVE,
  STATE_STANDSTILL_HOLD,
  DISABLED_RES_ENGAGE_WINDOW_FRAMES,
  HIGH_SPEED_MARGIN_MPH,
  LOW_SPEED_MARGIN_MPH,
  LOW_LOAD_BONUS_MPH,
  MARGIN_BLEND_END_MPH,
  MPH_TO_MS,
  SET_COOLDOWN_FRAMES,
  SET_COOLDOWN_DECEL_FAST_FRAMES,
  STANDSTILL_SET_PULSE_CAP,
  DECEL_FAST_VEGO_THRESHOLD_MS,
  DECEL_FAST_AEGO_MS2,
  DECEL_FAST_RATE_LIMIT_PRESSES_PER_SEC,
  GLOBAL_RATE_LIMIT_PRESSES_PER_SEC,
)
from opendbc.car.hyundai.values import Buttons

ButtonType = structs.CarState.ButtonEvent.Type


@dataclass
class FakeCP:
  flags: int = HyundaiFlags.HYBRID
  openpilotLongitudinalControl: bool = False


@dataclass
class FakeCPSP:
  flags: int = 0


@dataclass
class FakeButtonEvent:
  pressed: bool = True
  type: Any = ButtonType.accelCruise


@dataclass
class FakeCruiseState:
  speed: float = 0.0
  standstill: bool = False


@dataclass
class FakeCarStateOut:
  vEgo: float = 0.0
  aEgo: float = 0.0
  brakePressed: bool = False
  gasPressed: bool = False
  cruiseState: FakeCruiseState = field(default_factory=FakeCruiseState)
  buttonEvents: list = field(default_factory=list)


@dataclass
class FakeCS:
  out: FakeCarStateOut = field(default_factory=FakeCarStateOut)
  est_power_w: float = 0.0
  accel_demand: float = 0.0
  dte_raw: float = 100.0


@dataclass
class FakeCC:
  enabled: bool = False


def _make_limiter(power_threshold_kw: int = 40, dte_floor: int = 5):
  cp = FakeCP()
  cp_sp = FakeCPSP()
  lim = EVLimiter(cp, cp_sp)
  lim._read_bool = lambda key, default: True if key == "EVLimiterEnabled" else default
  lim._read_int = lambda key, default: {
    "EVLimiterPowerThresholdKW": power_threshold_kw,
    "EVLimiterDTEFloor": dte_floor,
  }.get(key, default)
  return lim


def _step(limiter, frame, cc_enabled, vEgo, observed_mph, button=None,
          est_power_w=0.0, abasis=0.0, brake=False, gas=False, aEgo=0.0):
  cs = FakeCS()
  cs.out.vEgo = vEgo
  cs.out.aEgo = aEgo
  cs.out.cruiseState.speed = observed_mph * MPH_TO_MS
  cs.out.brakePressed = brake
  cs.out.gasPressed = gas
  cs.out.buttonEvents = [FakeButtonEvent(pressed=True, type=button)] if button is not None else []
  cs.est_power_w = est_power_w
  cs.accel_demand = abasis
  cs.dte_raw = 100.0
  cc = FakeCC(enabled=cc_enabled)
  return limiter.update(cc, cs, frame)


class TestDynamicMargin(unittest.TestCase):
  """The sliding-cap formula. iter7: now takes (vEgo, est_power, threshold).

  Tests use load=0 (low load → full bonus) for the base-formula tests, then
  separate tests cover the load-bonus taper.
  """
  THR = 40_000.0  # 40 kW threshold for these tests

  def test_standstill_base_with_full_bonus(self):
    # Low load → full +10 mph bonus
    margin = EVLimiter._dynamic_margin_ms(0.0, est_power_w=0.0, power_threshold_w=self.THR)
    expected_mph = LOW_SPEED_MARGIN_MPH + LOW_LOAD_BONUS_MPH  # 30 mph
    self.assertAlmostEqual(margin, expected_mph * MPH_TO_MS, places=4)

  def test_blend_end_with_full_bonus(self):
    margin = EVLimiter._dynamic_margin_ms(MARGIN_BLEND_END_MPH * MPH_TO_MS,
                                           est_power_w=0.0, power_threshold_w=self.THR)
    expected_mph = HIGH_SPEED_MARGIN_MPH + LOW_LOAD_BONUS_MPH  # 15 mph
    self.assertAlmostEqual(margin, expected_mph * MPH_TO_MS, places=4)

  def test_high_speed_flat_with_full_bonus(self):
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=0.0, power_threshold_w=self.THR)
    expected_mph = HIGH_SPEED_MARGIN_MPH + LOW_LOAD_BONUS_MPH  # 15 mph
    self.assertAlmostEqual(margin, expected_mph * MPH_TO_MS, places=4)

  def test_midpoint_linear_base_with_full_bonus(self):
    # vEgo = 15 mph → base = (20 + 5) / 2 = 12.5 mph; + 10 bonus = 22.5
    margin = EVLimiter._dynamic_margin_ms(15.0 * MPH_TO_MS,
                                           est_power_w=0.0, power_threshold_w=self.THR)
    self.assertAlmostEqual(margin, 22.5 * MPH_TO_MS, places=2)

  def test_low_load_bonus_full_below_40pct(self):
    # est_power = 30% of threshold = 12 kW → full +10 bonus
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=12_000.0, power_threshold_w=self.THR)
    self.assertAlmostEqual(margin, (HIGH_SPEED_MARGIN_MPH + LOW_LOAD_BONUS_MPH) * MPH_TO_MS, places=2)

  def test_load_bonus_taper_at_60pct(self):
    # est_power = 60% of threshold = halfway between 40% and 80% → half bonus
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=24_000.0, power_threshold_w=self.THR)
    self.assertAlmostEqual(margin, (HIGH_SPEED_MARGIN_MPH + LOW_LOAD_BONUS_MPH * 0.5) * MPH_TO_MS, places=2)

  def test_load_bonus_zero_above_80pct(self):
    # est_power = 90% of threshold → no bonus, base only
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=36_000.0, power_threshold_w=self.THR)
    self.assertAlmostEqual(margin, HIGH_SPEED_MARGIN_MPH * MPH_TO_MS, places=2)

  def test_invalid_threshold_no_bonus(self):
    """Fail-safe: invalid threshold (≤0) gives base margin only, NOT max bonus."""
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=0.0, power_threshold_w=0.0)
    self.assertAlmostEqual(margin, HIGH_SPEED_MARGIN_MPH * MPH_TO_MS, places=2)
    margin = EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS,
                                           est_power_w=0.0, power_threshold_w=-100.0)
    self.assertAlmostEqual(margin, HIGH_SPEED_MARGIN_MPH * MPH_TO_MS, places=2)


class TestSlidingCapDownTrigger(unittest.TestCase):
  """Push-down behavior: SET fires when cluster set exceeds target_set or
  when est power is high with a vEgo gap to close."""

  def setUp(self):
    self.lim = _make_limiter(power_threshold_kw=40)
    # Engage cleanly with vEgo > standstill so we can run the sliding cap.
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=10.0, observed_mph=30.0)
    _step(self.lim, 5, cc_enabled=True, vEgo=10.0, observed_mph=30.0,
          button=ButtonType.decelCruise)  # SET-engage from main switch
    # Wait past STANDSTILL exit clean window
    for f in range(6, 40):
      _step(self.lim, f, cc_enabled=True, vEgo=10.0, observed_mph=30.0)

  def test_drive5_ice_set_to_high_fires(self):
    """Drive #5 ICE event regression: cluster=60, vEgo=46, user_target=51
    must immediately fire SET (observed > target_set + deadband). The old
    architecture only fired SOFT_CAP after power crossed threshold — too late."""
    self.lim.user_target_speed = 51.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000  # clear cooldown
    btn, _ = _step(self.lim, 100, cc_enabled=True, vEgo=46.0 * MPH_TO_MS,
                   observed_mph=60.0, est_power_w=10_000.0, abasis=0.1)
    self.assertEqual(btn, Buttons.SET_DECEL,
                      "set_too_high should fire SET when observed (60) > target_set (51)")

  def test_set_fires_during_driver_override_set(self):
    """Driver SET is the SAME direction as our SET — driver-SET override
    must NOT suppress our SET, even with the override window open. Blocking
    our SET here would let cluster_set sit above target_set for the override
    window with no limiter response (gpt-5.5 review of iter6 caught this)."""
    self.lim.user_target_speed = 30.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    # Simulate a recent driver SET press
    self.lim._driver_set_last_frame = 95  # 5 frames before our test frame
    # vEgo=10, margin=~16 mph, ceiling=26, target=min(30, 26)=26
    # observed=60 is 34 mph above target → set_too_high True
    btn, _ = _step(self.lim, 100, cc_enabled=True, vEgo=10.0 * MPH_TO_MS,
                   observed_mph=60.0, est_power_w=10_000.0, abasis=0.1)
    self.assertEqual(btn, Buttons.SET_DECEL,
                      "Driver SET override window must NOT block our SET (same direction)")

  def test_under_target_does_not_fire_set(self):
    """When cluster set already at target_set, SET must not fire."""
    self.lim.user_target_speed = 60.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    # vEgo=50 → margin=5 → ceiling=55 → target=min(60,55)=55
    # observed=55 → set_too_high False, want_res only if under_target
    btn, _ = _step(self.lim, 200, cc_enabled=True, vEgo=50.0 * MPH_TO_MS,
                   observed_mph=55.0, est_power_w=10_000.0, abasis=0.0)
    self.assertNotEqual(btn, Buttons.SET_DECEL)

  def test_power_too_high_with_gap_fires_set(self):
    """Power above threshold AND observed > vEgo + 0.5 mph → SET."""
    self.lim.user_target_speed = 60.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    # vEgo=20 mph → margin ≈ 10 mph → ceiling=30, target=min(60,30)=30
    # observed=30 → set_too_high False (right at boundary), but power > 40 kW
    # AND observed > vEgo + 0.5 → power_too_high → SET
    btn, _ = _step(self.lim, 300, cc_enabled=True, vEgo=20.0 * MPH_TO_MS,
                   observed_mph=30.0, est_power_w=50_000.0, abasis=0.5)
    self.assertEqual(btn, Buttons.SET_DECEL)

  def test_power_too_high_no_gap_does_not_fire(self):
    """Power high but observed near vEgo (no gap) → SET would not help."""
    self.lim.user_target_speed = 60.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    # vEgo=30, observed=30 (no gap), high power (e.g. grade)
    # → power_too_high False (gap requirement); set_too_high False (target=35)
    btn, _ = _step(self.lim, 400, cc_enabled=True, vEgo=30.0 * MPH_TO_MS,
                   observed_mph=30.0, est_power_w=60_000.0, abasis=0.0)
    self.assertNotEqual(btn, Buttons.SET_DECEL)


class TestStandstillSuppression(unittest.TestCase):
  """Iter7: standstill BLOCKS RES, but ALLOWS SET (pulse-bounded) to pull
  cluster set down toward LOW_SPEED_MARGIN_MPH (20 mph). Drive #6 retro:
  iter6 froze cluster_set wherever the deceleration SET cascade landed."""

  def test_standstill_set_fires_when_observed_above_cap(self):
    """At standstill with cluster_set above 20 mph cap, SET must fire on
    the engage frame (no cooldown to satisfy yet)."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=44.0)
    btn, _ = _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=44.0,
                    button=ButtonType.accelCruise)
    self.assertEqual(btn, Buttons.SET_DECEL,
                      "Standstill SET must fire when observed cluster set is above 20 mph cap")

  def test_standstill_no_set_when_at_cap(self):
    """At standstill with cluster_set already at 20 mph, no SET needed."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=20.0)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=20.0,
          button=ButtonType.accelCruise)
    btn, _ = _step(lim, 26, cc_enabled=True, vEgo=0.0, observed_mph=20.0)
    self.assertEqual(btn, Buttons.NONE,
                      "Standstill SET must NOT fire when observed already at cap")

  def test_standstill_brake_blocks_set(self):
    """Standstill SET must NOT fire if brake is pressed."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=44.0, brake=True)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=44.0,
          button=ButtonType.accelCruise, brake=True)
    btn, _ = _step(lim, 26, cc_enabled=True, vEgo=0.0, observed_mph=44.0, brake=True)
    self.assertEqual(btn, Buttons.NONE)

  def test_standstill_res_never_fires(self):
    """Even if cluster_set is below user_target at standstill, RES must
    NEVER fire (vehicle is stopped)."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=10.0)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=10.0,
          button=ButtonType.accelCruise)
    # user_target seeded from the frozen 10 mph (engage_via_res). Bump it up
    # via simulated driver presses so the under_target condition would otherwise fire.
    lim.user_target_speed = 50.0 * MPH_TO_MS
    for f in range(26, 60):
      btn, _ = _step(lim, f, cc_enabled=True, vEgo=0.0, observed_mph=10.0)
      self.assertNotEqual(btn, Buttons.RES_ACCEL,
                           f"Standstill must never fire RES (frame {f})")

  def test_standstill_set_pulse_cap(self):
    """SET pulse count is bounded per single standstill window."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=80.0)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=80.0,
          button=ButtonType.accelCruise)
    set_count = 0
    for f in range(26, 26 + 100 * STANDSTILL_SET_PULSE_CAP):  # plenty of frames
      btn, _ = _step(lim, f, cc_enabled=True, vEgo=0.0, observed_mph=80.0)
      if btn == Buttons.SET_DECEL:
        set_count += 1
    self.assertLessEqual(set_count, STANDSTILL_SET_PULSE_CAP,
                          f"Standstill SET pulses must be capped at {STANDSTILL_SET_PULSE_CAP}; got {set_count}")


class TestGasBrakePause(unittest.TestCase):
  """Gas or brake pause suppresses both SET and RES."""

  def setUp(self):
    self.lim = _make_limiter()
    # Engage cleanly above standstill
    for f in range(0, 25):
      _step(self.lim, f, cc_enabled=False, vEgo=10.0, observed_mph=30.0)
    _step(self.lim, 25, cc_enabled=True, vEgo=10.0, observed_mph=30.0,
          button=ButtonType.decelCruise)
    for f in range(26, 50):
      _step(self.lim, f, cc_enabled=True, vEgo=10.0, observed_mph=30.0)

  def test_gas_suppresses_set(self):
    self.lim.user_target_speed = 30.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    # Conditions that would normally fire SET:
    btn, _ = _step(self.lim, 100, cc_enabled=True, vEgo=10.0,
                   observed_mph=60.0, est_power_w=50_000.0, abasis=1.0, gas=True)
    self.assertEqual(btn, Buttons.NONE, "Gas must suppress SET")

  def test_brake_suppresses_set(self):
    self.lim.user_target_speed = 30.0 * MPH_TO_MS
    self.lim.last_set_frame = -10000
    btn, _ = _step(self.lim, 200, cc_enabled=True, vEgo=10.0,
                   observed_mph=60.0, est_power_w=50_000.0, abasis=1.0, brake=True)
    self.assertEqual(btn, Buttons.NONE, "Brake must suppress SET")

  def test_gas_suppresses_res(self):
    self.lim.user_target_speed = 30.0 * MPH_TO_MS
    self.lim.last_res_frame = -10000
    # Set up under-target conditions
    btn, _ = _step(self.lim, 300, cc_enabled=True, vEgo=10.0,
                   observed_mph=15.0, est_power_w=0.0, abasis=0.0, gas=True)
    self.assertEqual(btn, Buttons.NONE, "Gas must suppress RES")


class TestStateLatching(unittest.TestCase):
  """HUD state derived from recent activity, not button-this-frame."""

  def test_limiting_state_persists_after_set(self):
    """After a SET press, state must remain SOFT_CAP_ACTIVE for at least
    LIMITING_LATCH_FRAMES frames even though the cooldown blocks more SETs."""
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=10.0, observed_mph=44.0)
    _step(lim, 25, cc_enabled=True, vEgo=10.0, observed_mph=44.0,
          button=ButtonType.decelCruise)
    for f in range(26, 50):
      _step(lim, f, cc_enabled=True, vEgo=10.0, observed_mph=44.0)
    lim.user_target_speed = 30.0 * MPH_TO_MS
    lim.last_set_frame = -10000
    # Frame 100: SET fires
    _step(lim, 100, cc_enabled=True, vEgo=10.0, observed_mph=60.0,
          est_power_w=50_000.0, abasis=0.5)
    # Frame 110 (10 frames later, SET cooldown blocks new TX, but state
    # should still report SOFT_CAP_ACTIVE because last_set_frame is recent)
    _step(lim, 110, cc_enabled=True, vEgo=10.0, observed_mph=59.0,
          est_power_w=50_000.0, abasis=0.5)
    self.assertEqual(lim.state, STATE_SOFT_CAP_ACTIVE,
                      "State should latch SOFT_CAP_ACTIVE for ~500 ms after our SET")


class TestEngagePathDriveFour(unittest.TestCase):
  """Drive #4 regression coverage — kept from iter5."""

  def setUp(self):
    self.lim = _make_limiter()

  def test_disabled_tracks_observed_set_speed(self):
    _step(self.lim, 0, cc_enabled=False, vEgo=20.0, observed_mph=70.0)
    self.assertAlmostEqual(self.lim._observed_set_speed_at_disable,
                            70.0 * MPH_TO_MS, places=4)

  def test_disabled_res_press_recorded(self):
    _step(self.lim, 5, cc_enabled=False, vEgo=20.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    self.assertEqual(self.lim._last_disabled_res_press_frame, 5)

  def test_engage_via_res_seeds_user_target_from_pre_disable(self):
    for f in range(0, 10):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    _step(self.lim, 10, cc_enabled=True, vEgo=24.0, observed_mph=56.0,
          button=ButtonType.accelCruise)
    self.assertAlmostEqual(self.lim.user_target_speed, 70.0 * MPH_TO_MS, places=4)

  def test_engage_via_res_via_disabled_lookback(self):
    """Wheel RES landed BEFORE cc_enabled rose — engage_via_res via lookback."""
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    _step(self.lim, 5, cc_enabled=False, vEgo=24.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    _step(self.lim, 6, cc_enabled=True, vEgo=24.0, observed_mph=56.0)
    self.assertAlmostEqual(self.lim.user_target_speed, 70.0 * MPH_TO_MS, places=4)

  def test_engage_via_res_does_not_latch_drv_res(self):
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=48.0)
    _step(self.lim, 5, cc_enabled=True, vEgo=24.0, observed_mph=56.0,
          button=ButtonType.accelCruise)
    self.assertLess(self.lim._driver_res_last_frame, 0)
    self.assertFalse(self.lim._in_driver_override_res(5))

  def test_engage_via_set_seeds_from_observed(self):
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    _step(self.lim, 5, cc_enabled=True, vEgo=24.0, observed_mph=24.0,
          button=ButtonType.decelCruise)
    self.assertAlmostEqual(self.lim.user_target_speed, 24.0 * MPH_TO_MS, places=4)

  def test_disabled_res_lookback_window_expires(self):
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    _step(self.lim, 5, cc_enabled=False, vEgo=24.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    for f in range(6, 6 + DISABLED_RES_ENGAGE_WINDOW_FRAMES + 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    engage_frame = 6 + DISABLED_RES_ENGAGE_WINDOW_FRAMES + 5
    _step(self.lim, engage_frame, cc_enabled=True, vEgo=24.0, observed_mph=24.0)
    self.assertAlmostEqual(self.lim.user_target_speed, 24.0 * MPH_TO_MS, places=4)


class TestPowerEstimator(unittest.TestCase):
  """Iter7 power formula: mass·v·(max(0,abasis) + max(0,grade)) + road_load.

  Drive #6 7:40 ICE event proved the iter6 formula was blind in a key regime:
  abasis=-0.4 (SCC commanding decel) + grade=+0.4 (uphill) yielded
  max(0, abasis+grade) = max(0, 0) = 0 even though motor was doing real work.
  Iter7 clamps both terms positive separately, plus adds a road-load baseline.
  """
  def setUp(self):
    # Late import: carstate_ext requires opendbc structs which require numpy
    from opendbc.sunnypilot.car.hyundai.carstate_ext import (
      road_load_power_w, VEHICLE_MASS_KG, GRAVITY_MS2,
    )
    self.road_load = road_load_power_w
    self.MASS = VEHICLE_MASS_KG
    self.G = GRAVITY_MS2

  def _formula(self, abasis, grade_f, v_mph):
    """Compute the iter7 power formula directly (mirrors carstate_ext logic)."""
    v = v_mph * MPH_TO_MS
    abasis_pos = max(0.0, abasis)
    grade_pos = max(0.0, grade_f)
    p_accel_grade = self.MASS * v * (abasis_pos + grade_pos)
    p_road = self.road_load(v)
    return max(0.0, p_accel_grade + p_road)

  def test_road_load_zero_at_standstill(self):
    self.assertAlmostEqual(self.road_load(0.0), 0.0, places=2)

  def test_road_load_increases_with_speed(self):
    # Cubic in speed → highway speeds dominated by aero
    p_30 = self.road_load(30.0 * MPH_TO_MS)
    p_60 = self.road_load(60.0 * MPH_TO_MS)
    p_75 = self.road_load(75.0 * MPH_TO_MS)
    self.assertLess(p_30, p_60)
    self.assertLess(p_60, p_75)
    # Sanity: 75 mph baseline should be 20-30 kW for an SUV
    self.assertGreater(p_75, 18_000.0)
    self.assertLess(p_75, 35_000.0)

  def test_drive6_ice_blind_spot_caught(self):
    """7:40 ICE event scenario: at 73 mph holding speed against grade,
    iter6 read 0 kW. Iter7 must read above the 37 kW threshold (the lowest
    user-reasonable threshold), and ideally above the 40 kW default too."""
    # vEgo=73 mph, abasis=-0.4 (SCC commanding decel), grade=+0.4 (uphill)
    pwr = self._formula(abasis=-0.4, grade_f=+0.4, v_mph=73.0)
    self.assertGreater(pwr, 40_000.0,
                        f"Iter6 ICE blind spot must be caught (got {pwr/1000:.1f} kW)")

  def test_high_speed_cruise_no_grade_below_threshold(self):
    """At 73 mph holding speed against road load only (no grade), motor
    power equals road load — should be 20-30 kW, well below 40 kW default."""
    pwr = self._formula(abasis=-0.4, grade_f=0.0, v_mph=73.0)
    self.assertGreater(pwr, 18_000.0)
    self.assertLess(pwr, 32_000.0)

  def test_downhill_does_not_cancel_accel(self):
    """Negative grade (downhill) must NOT cancel positive aBasis. The
    protective estimator stays conservative — false positives are OK,
    false negatives lead to ICE."""
    # Compare: same abasis+v, with vs without downhill grade
    pwr_with_downhill = self._formula(abasis=+0.3, grade_f=-0.3, v_mph=60.0)
    pwr_no_grade = self._formula(abasis=+0.3, grade_f=0.0, v_mph=60.0)
    # Downhill clamps to 0 in our formula, so power is identical
    self.assertAlmostEqual(pwr_with_downhill, pwr_no_grade, places=2,
                            msg="Downhill grade must not change protective power estimate")

  def test_decel_does_not_cancel_grade(self):
    """SCC commanded decel (negative abasis) must NOT cancel positive grade.
    This is the iter6 formula bug that drive #6 ICE event exposed."""
    pwr_with_decel = self._formula(abasis=-0.4, grade_f=+0.4, v_mph=60.0)
    pwr_no_abasis = self._formula(abasis=0.0, grade_f=+0.4, v_mph=60.0)
    self.assertAlmostEqual(pwr_with_decel, pwr_no_abasis, places=2,
                            msg="Negative abasis must not cancel positive grade contribution")


class TestDecelFastCadence(unittest.TestCase):
  """iter8: SET cadence shortened during low-speed deceleration so cluster
  can be pulled toward target_set faster than default 6.7 mph/s rate. Drive
  #6 7:34 retro: hard decel from cruise to red light left cluster set frozen
  high because default cadence lost the race.
  """

  def setUp(self):
    self.lim = _make_limiter()
    # Engage at moderate speed so we have a clean baseline above standstill
    for f in range(0, 25):
      _step(self.lim, f, cc_enabled=False, vEgo=10.0, observed_mph=44.0)
    _step(self.lim, 25, cc_enabled=True, vEgo=10.0, observed_mph=44.0,
          button=ButtonType.decelCruise)
    # Settle past standstill exit hold-off
    for f in range(26, 60):
      _step(self.lim, f, cc_enabled=True, vEgo=10.0, observed_mph=44.0,
            est_power_w=0.0, abasis=0.0, aEgo=0.0)
    self.lim.user_target_speed = 50.0 * MPH_TO_MS

  def test_decel_fast_set_cooldown_active(self):
    """vEgo < 30 mph + aEgo < -0.5 m/s² → SET fires at faster cadence."""
    # Force conditions: low vEgo (10 m/s ≈ 22 mph, < 30), strong decel
    # (aEgo = -1.0). cluster=44 vs target_set=22+margin
    self.lim.last_set_frame = -10000
    # First SET on frame 100
    btn1, _ = _step(self.lim, 100, cc_enabled=True, vEgo=10.0,
                    observed_mph=44.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=-1.0)
    self.assertEqual(btn1, Buttons.SET_DECEL)
    # 7 frames later (70 ms) — should fire if decel-fast cadence (6 frames)
    # is active, would NOT fire under default cadence (15 frames)
    btn2, _ = _step(self.lim, 107, cc_enabled=True, vEgo=10.0,
                    observed_mph=43.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=-1.0)
    self.assertEqual(btn2, Buttons.SET_DECEL,
                      "Decel-fast: SET should fire at 60 ms cadence (got NONE — using default 150 ms?)")

  def test_decel_fast_inactive_at_high_speed(self):
    """vEgo >= 30 mph → default cadence even with strong decel."""
    self.lim.last_set_frame = -10000
    # vEgo = 50 mph (= 22.4 m/s, above the 30 mph threshold) with decel
    btn1, _ = _step(self.lim, 200, cc_enabled=True, vEgo=22.4,
                    observed_mph=80.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=-1.0)
    self.assertEqual(btn1, Buttons.SET_DECEL)
    # 7 frames later — should NOT fire (default cadence is 15)
    btn2, _ = _step(self.lim, 207, cc_enabled=True, vEgo=22.4,
                    observed_mph=80.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=-1.0)
    self.assertEqual(btn2, Buttons.NONE,
                      "At highway speed, default 150 ms cadence must apply (got SET)")

  def test_decel_fast_inactive_when_not_decelerating(self):
    """vEgo < 30 mph but aEgo ~ 0 → default cadence (low-speed cruising
    isn't a brake-decel scenario)."""
    self.lim.last_set_frame = -10000
    btn1, _ = _step(self.lim, 300, cc_enabled=True, vEgo=10.0,
                    observed_mph=44.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=0.0)
    self.assertEqual(btn1, Buttons.SET_DECEL)
    # 7 frames later — no decel, default cadence
    btn2, _ = _step(self.lim, 307, cc_enabled=True, vEgo=10.0,
                    observed_mph=44.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=0.0)
    self.assertEqual(btn2, Buttons.NONE)

  def test_decel_fast_rate_limit_allows_higher_rate(self):
    """During decel-fast, 12 Hz rate limit allows faster sustained SET
    than the default 6 Hz would."""
    # Pre-fill rate limit history with 6 presses in last second to saturate
    # the default limit; verify a 7th still fires under decel-fast.
    self.lim.last_set_frame = -10000
    f = 1000
    for _ in range(6):
      btn, _ = _step(self.lim, f, cc_enabled=True, vEgo=10.0,
                     observed_mph=44.0, est_power_w=5_000.0, abasis=0.0,
                     aEgo=-1.0)
      self.assertEqual(btn, Buttons.SET_DECEL)
      f += SET_COOLDOWN_DECEL_FAST_FRAMES
    # 6 SETs fired in ~36 frames (360 ms). 7th press 6 frames later — under
    # default 6 Hz limit this would be blocked, under decel-fast 12 Hz it fires.
    btn7, _ = _step(self.lim, f, cc_enabled=True, vEgo=10.0,
                    observed_mph=44.0, est_power_w=5_000.0, abasis=0.0,
                    aEgo=-1.0)
    self.assertEqual(btn7, Buttons.SET_DECEL,
                      "Decel-fast 12 Hz limit must allow >6 SETs/sec")

  def test_standstill_pulse_cap_lowered_to_10(self):
    """iter8: STANDSTILL_SET_PULSE_CAP reduced 30 → 10."""
    self.assertEqual(STANDSTILL_SET_PULSE_CAP, 10)
    lim = _make_limiter()
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=80.0)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=80.0,
          button=ButtonType.accelCruise)
    set_count = 0
    for f in range(26, 26 + 50 * STANDSTILL_SET_PULSE_CAP):
      btn, _ = _step(lim, f, cc_enabled=True, vEgo=0.0, observed_mph=80.0)
      if btn == Buttons.SET_DECEL:
        set_count += 1
    self.assertLessEqual(set_count, STANDSTILL_SET_PULSE_CAP)
    # Also make sure we hit the cap (not under-firing)
    self.assertGreaterEqual(set_count, STANDSTILL_SET_PULSE_CAP - 1)


if __name__ == "__main__":
  unittest.main()
