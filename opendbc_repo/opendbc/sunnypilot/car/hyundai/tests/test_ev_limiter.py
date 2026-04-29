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
  MARGIN_BLEND_END_MPH,
  MPH_TO_MS,
  SET_COOLDOWN_FRAMES,
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
          est_power_w=0.0, abasis=0.0, brake=False, gas=False):
  cs = FakeCS()
  cs.out.vEgo = vEgo
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
  """The sliding-cap formula itself."""

  def test_standstill_margin(self):
    self.assertAlmostEqual(EVLimiter._dynamic_margin_ms(0.0),
                            LOW_SPEED_MARGIN_MPH * MPH_TO_MS, places=4)

  def test_blend_end_margin(self):
    self.assertAlmostEqual(EVLimiter._dynamic_margin_ms(MARGIN_BLEND_END_MPH * MPH_TO_MS),
                            HIGH_SPEED_MARGIN_MPH * MPH_TO_MS, places=4)

  def test_high_speed_flat(self):
    # vEgo well above blend end → still high-speed margin
    self.assertAlmostEqual(EVLimiter._dynamic_margin_ms(40.0 * MPH_TO_MS),
                            HIGH_SPEED_MARGIN_MPH * MPH_TO_MS, places=4)

  def test_midpoint_linear(self):
    # vEgo = 15 mph → halfway → margin = (20 + 5) / 2 = 12.5 mph
    margin = EVLimiter._dynamic_margin_ms(15.0 * MPH_TO_MS)
    self.assertAlmostEqual(margin, 12.5 * MPH_TO_MS, places=2)


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
  """Standstill suppresses BOTH SET and RES. Sliding cap takes over
  immediately on exit-standstill (no hidden cooldown holding back the
  first SET)."""

  def test_standstill_no_buttons(self):
    lim = _make_limiter()
    # Engage at standstill with stored set high
    for f in range(0, 25):  # > STANDSTILL_CONFIRM_FRAMES
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=44.0)
    btn, _ = _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=44.0,
                   button=ButtonType.accelCruise)
    self.assertEqual(btn, Buttons.NONE)
    # Subsequent frames at standstill: still no buttons
    for f in range(26, 60):
      btn, _ = _step(lim, f, cc_enabled=True, vEgo=0.0, observed_mph=44.0)
      self.assertEqual(btn, Buttons.NONE,
                        f"Standstill must suppress all buttons at frame {f}")

  def test_exit_standstill_no_hidden_cooldown_blocks_first_set(self):
    """Drive #5 dwell prevention: the moment standstill clears with stored
    cluster set well above target, SET must be eligible immediately. There
    must be NO settle / engage-edge / cooldown hidden delay."""
    lim = _make_limiter()
    # Engage at standstill with stored set 44 mph (above any margin at 0 mph)
    for f in range(0, 25):
      _step(lim, f, cc_enabled=False, vEgo=0.0, observed_mph=44.0)
    _step(lim, 25, cc_enabled=True, vEgo=0.0, observed_mph=44.0,
          button=ButtonType.accelCruise)
    # Hold at standstill long enough to pass STANDSTILL_CONFIRM_FRAMES
    for f in range(26, 60):
      _step(lim, f, cc_enabled=True, vEgo=0.0, observed_mph=44.0)
    # Now exit standstill — vEgo crosses STANDSTILL_EXIT (3 mph). Stored
    # observed_mph=44, user_target=44, dynamic_ceiling at vEgo=3 is ~18.5 mph,
    # so target_set=18.5, observed (44) is 25 mph above. set_too_high=True.
    saw_set_within_5_frames = False
    for f in range(60, 65):
      btn, _ = _step(lim, f, cc_enabled=True, vEgo=3.5 * MPH_TO_MS,
                     observed_mph=44.0, est_power_w=5_000.0, abasis=0.5)
      if btn == Buttons.SET_DECEL:
        saw_set_within_5_frames = True
        break
    self.assertTrue(saw_set_within_5_frames,
                     "First SET must fire within 5 frames of exit-standstill")


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


if __name__ == "__main__":
  unittest.main()
