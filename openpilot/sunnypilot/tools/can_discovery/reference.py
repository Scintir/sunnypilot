"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Reference tractive-power estimate built from the car's own vEgo/aEgo and curb mass. This is what candidate
CAN fields are correlated against. It is deliberately simple: the goal is to find the signal that *is* the
battery power/current, not to model the vehicle precisely.

  P_ref = m * a * v            (inertial)
        + m * g * Crr * v      (rolling)
        + 0.5 * rho * CdA * v^3 (aero)

Positive = discharging the pack, negative = regen. Friction braking is not observable here, so samples with
brakePressed set are masked out of the correlation by default.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from openpilot.sunnypilot.tools.can_discovery.capture import Capture, sample_hold

G = 9.81
RHO_AIR = 1.2


@dataclass
class ReferenceSeries:
  grid: np.ndarray        # seconds
  valid: np.ndarray       # bool mask: car state fresh, not friction braking (if masked)
  v: np.ndarray           # m/s
  a: np.ndarray           # m/s^2
  p_ref: np.ndarray       # W, estimated tractive power
  p_inertial: np.ndarray  # W, m*a*v only (cleaner sign during hard accel/regen)

  def __len__(self) -> int:
    return len(self.grid)


def build_reference(cap: Capture, rate_hz: float = 20.0, crr: float = 0.010, cda: float = 0.75,
                    mass_kg: float | None = None, mask_friction_brake: bool = True,
                    min_speed: float = 0.5) -> ReferenceSeries:
  mass = mass_kg if mass_kg else cap.mass_kg
  if not mass:
    raise ValueError("no vehicle mass: pass --mass or use a route with carParams logged")

  car = cap.car
  if len(car.times) < 2:
    raise ValueError("route has no carState messages; the reference needs vEgo/aEgo")

  grid = np.arange(0.0, cap.duration_s, 1.0 / rate_hz)
  max_age = 0.25
  v, ok_v = sample_hold(car.times, car.v_ego, grid, max_age)
  a, _ = sample_hold(car.times, car.a_ego, grid, max_age)
  brake, _ = sample_hold(car.times, car.brake_pressed, grid, max_age)

  valid = ok_v & (v > min_speed)
  if mask_friction_brake:
    valid &= ~brake

  p_inertial = mass * a * v
  p_ref = p_inertial + mass * G * crr * v + 0.5 * RHO_AIR * cda * v ** 3
  return ReferenceSeries(grid, valid, v, a, p_ref, p_inertial)
