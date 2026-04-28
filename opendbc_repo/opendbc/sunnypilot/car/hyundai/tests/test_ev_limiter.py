"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Minimal regression coverage for the EV power limiter, focused on the drive #4
failure mode: Hyundai SCC autonomously snaps the cluster set speed up on
RES re-engage, and earlier limiter revisions either (a) processed the engage
RES as driver intent and opened a 3 s DRV_RES override window suppressing
SOFT_CAP, or (b) seeded user_target from the SCC-snapped post-engage value.

These tests exercise the engage classification + seeding path with synthetic
button events. Broader behavioural coverage (power-driven SOFT_CAP entry/exit,
driver override windows, recovery cadence) is intentionally out of scope here.
"""

import unittest
from dataclasses import dataclass, field
from typing import Any

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai import ev_limiter as ev_lim
from opendbc.sunnypilot.car.hyundai.ev_limiter import (
  EVLimiter,
  STATE_DISABLED,
  STATE_DRIVER_OVERRIDE_RES,
  STATE_IDLE,
  STATE_SOFT_CAP_ACTIVE,
  AUTO_RESUME_GUARD_FRAMES,
  AUTO_RESUME_SET_COOLDOWN_FRAMES,
  DISABLED_RES_ENGAGE_WINDOW_FRAMES,
  DRIVER_OVERRIDE_RES_FRAMES,
  MPH_TO_MS,
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
  # Override params reads (no Params backend in tests).
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


class TestEVLimiterEngagePath(unittest.TestCase):
  """Drive #4 regression coverage for engage classification + seeding."""

  def setUp(self):
    self.lim = _make_limiter()

  def test_supported_hybrid(self):
    self.assertTrue(self.lim.supported)

  def test_disabled_returns_no_button(self):
    btn, active = _step(self.lim, 0, cc_enabled=False, vEgo=0.0, observed_mph=0.0)
    self.assertEqual(btn, Buttons.NONE)
    self.assertFalse(active)
    self.assertEqual(self.lim.state, STATE_DISABLED)

  def test_disabled_tracks_observed_set_speed(self):
    """Cluster set during DISABLED is captured for later engage seed."""
    _step(self.lim, 0, cc_enabled=False, vEgo=20.0, observed_mph=70.0)
    self.assertAlmostEqual(self.lim._observed_set_speed_at_disable, 70.0 * MPH_TO_MS, places=4)

  def test_disabled_res_press_recorded(self):
    """A RES press while disabled stamps _last_disabled_res_press_frame."""
    _step(self.lim, 5, cc_enabled=False, vEgo=20.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    self.assertEqual(self.lim._last_disabled_res_press_frame, 5)

  def test_disabled_res_does_not_open_driver_adjust_window(self):
    """Drive #4 race: disabled-frame RES must NOT open driver-adjust /
    DRV_RES windows that survive into the engaged session."""
    _step(self.lim, 5, cc_enabled=False, vEgo=20.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    self.assertEqual(self.lim._driver_adjust_until, -10000)
    self.assertEqual(self.lim._driver_res_last_frame, -10000)

  def test_engage_via_res_seeds_user_target_from_pre_disable(self):
    """RES re-engage with frozen pre-disable observed = 70: user_target
    must seed from 70, NOT from a polluted post-engage observed of 56."""
    # Spend a few frames in DISABLED with cluster at 70.
    for f in range(0, 10):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    # Engage frame: cc rises, observed has already snapped to 56 (SCC's
    # autonomous resume), and the wheel RES press lands this frame.
    _step(self.lim, 10, cc_enabled=True, vEgo=24.0, observed_mph=56.0,
          button=ButtonType.accelCruise)
    self.assertAlmostEqual(self.lim.user_target_speed, 70.0 * MPH_TO_MS, places=4)

  def test_engage_via_res_via_disabled_lookback(self):
    """Wheel RES landed 1 frame BEFORE cc_enabled rose. Engage frame has
    no accelCruise event but engage_via_res must still classify True."""
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    # RES press while still disabled.
    _step(self.lim, 5, cc_enabled=False, vEgo=24.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    # Next frame: cc_enabled rises, no buttonEvent this frame.
    _step(self.lim, 6, cc_enabled=True, vEgo=24.0, observed_mph=56.0)
    # Seed must come from frozen 70, proving engage_via_res classified True
    # via the disabled-frame lookback.
    self.assertAlmostEqual(self.lim.user_target_speed, 70.0 * MPH_TO_MS, places=4)

  def test_engage_via_res_does_not_latch_drv_res(self):
    """Drive #4 fix: the RES that re-engages cruise must NOT open a
    DRIVER_OVERRIDE_RES window (which previously suppressed SOFT_CAP for 3 s)."""
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=48.0)
    _step(self.lim, 5, cc_enabled=True, vEgo=24.0, observed_mph=56.0,
          button=ButtonType.accelCruise)
    # _driver_res_last_frame stays at sentinel (DISABLED branch cleared it
    # to -10000, engage-frame accelCruise was skipped by just_engaged guard).
    self.assertLess(self.lim._driver_res_last_frame, 0)
    # Override-window check must therefore say False on engage frame.
    self.assertFalse(self.lim._in_driver_override_res(5))

  def test_engage_via_set_seeds_from_observed(self):
    """SET re-engage doesn't trigger SCC autonomous resume; observed is
    already fresh and should be the seed."""
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    _step(self.lim, 5, cc_enabled=True, vEgo=24.0, observed_mph=24.0,
          button=ButtonType.decelCruise)
    self.assertAlmostEqual(self.lim.user_target_speed, 24.0 * MPH_TO_MS, places=4)

  def test_first_engage_no_prior_disable_seeds_from_observed(self):
    """No prior disable history (fresh process start with cc already on):
    seed from current observed."""
    _step(self.lim, 0, cc_enabled=True, vEgo=20.0, observed_mph=65.0,
          button=ButtonType.accelCruise)
    self.assertAlmostEqual(self.lim.user_target_speed, 65.0 * MPH_TO_MS, places=4)

  def test_disabled_res_lookback_window_expires(self):
    """RES press that's older than DISABLED_RES_ENGAGE_WINDOW_FRAMES is
    not classified as engage_via_res."""
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    # Old RES press during disabled.
    _step(self.lim, 5, cc_enabled=False, vEgo=24.0, observed_mph=70.0,
          button=ButtonType.accelCruise)
    # Many frames pass, still disabled.
    for f in range(6, 6 + DISABLED_RES_ENGAGE_WINDOW_FRAMES + 5):
      _step(self.lim, f, cc_enabled=False, vEgo=24.0, observed_mph=70.0)
    # Engage frame, no buttonEvent this frame.
    engage_frame = 6 + DISABLED_RES_ENGAGE_WINDOW_FRAMES + 5
    _step(self.lim, engage_frame, cc_enabled=True, vEgo=24.0, observed_mph=24.0)
    # Lookback expired; engage_via_res = False; seed from current observed (24).
    self.assertAlmostEqual(self.lim.user_target_speed, 24.0 * MPH_TO_MS, places=4)


class TestEVLimiterAutoResumeGuard(unittest.TestCase):
  """SOFT_CAP must be allowed to fire SET during the post-engage
  auto-resume window even though older revs blocked it via DRV_RES."""

  def setUp(self):
    self.lim = _make_limiter(power_threshold_kw=40)

  def _engage_via_res(self):
    for f in range(0, 5):
      _step(self.lim, f, cc_enabled=False, vEgo=20.0, observed_mph=48.0)
    # Engage with RES; SCC has snapped cluster up to 56.
    _step(self.lim, 5, cc_enabled=True, vEgo=20.0, observed_mph=56.0,
          button=ButtonType.accelCruise)
    return 5

  def test_soft_cap_can_fire_during_auto_resume_guard(self):
    """Sustain high power for >300 ms after engage. SOFT_CAP must enter
    AND emit at least one SET press within the auto-resume guard window."""
    engage_frame = self._engage_via_res()
    # Sustained high power above cap (40 kW threshold) for 50 frames (500 ms).
    saw_set = False
    for f in range(engage_frame + 1, engage_frame + 1 + 60):
      btn, _ = _step(self.lim, f, cc_enabled=True, vEgo=20.0, observed_mph=56.0,
                     est_power_w=50_000.0, abasis=0.5)
      if btn == Buttons.SET_DECEL:
        saw_set = True
        break
    self.assertTrue(saw_set, "SOFT_CAP failed to emit SET during auto-resume guard")

  def test_auto_resume_guard_uses_faster_set_cadence(self):
    """During the guard, repeated SOFT_CAP frames should fire SET more
    often than the default 300 ms cadence allows."""
    engage_frame = self._engage_via_res()
    # Build up SOFT_CAP entry persistence first.
    f = engage_frame + 1
    while f < engage_frame + 1 + 35 and not self.lim._soft_cap_on:
      _step(self.lim, f, cc_enabled=True, vEgo=20.0, observed_mph=56.0,
            est_power_w=50_000.0, abasis=0.5)
      f += 1
    self.assertTrue(self.lim._soft_cap_on, "SOFT_CAP did not enter after sustained high load")
    # Count SET presses over a 1-second window during the guard.
    set_count = 0
    for f2 in range(f, f + 100):
      btn, _ = _step(self.lim, f2, cc_enabled=True, vEgo=20.0, observed_mph=56.0,
                     est_power_w=50_000.0, abasis=0.5)
      if btn == Buttons.SET_DECEL:
        set_count += 1
    # With AUTO_RESUME_SET_COOLDOWN_FRAMES = 15 (150 ms = 6.67 Hz wanted)
    # capped by GLOBAL_RATE_LIMIT_PRESSES_PER_SEC = 6 logical/sec, expect
    # at least 4 (allow some slack for cadence alignment / global limiter).
    self.assertGreaterEqual(set_count, 4,
                             f"Expected ≥4 SET in 1 s during auto-resume; got {set_count}")


if __name__ == "__main__":
  unittest.main()
