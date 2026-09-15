"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Load everything the discovery tools need out of a route's rlogs: every raw CAN frame per (bus, address),
the car's own speed/accel estimate, CarParams, and the per-bus panda health counters.
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

# bus values >= 128 are TX echoes / safety-rejected TX of our own frames, not vehicle traffic
MAX_RX_BUS = 3


@dataclass
class MessageSeries:
  bus: int
  address: int
  times: np.ndarray        # float64 seconds (logMonoTime based), sorted
  payloads: np.ndarray     # (N, L) uint8, zero padded to the longest frame seen
  lengths: np.ndarray      # (N,) uint8 raw DLC bytes per frame

  @property
  def count(self) -> int:
    return len(self.times)

  @property
  def rate_hz(self) -> float:
    if self.count < 2:
      return 0.0
    span = float(self.times[-1] - self.times[0])
    return (self.count - 1) / span if span > 0 else 0.0

  @property
  def is_fd(self) -> bool:
    return bool(self.lengths.max() > 8) if self.count else False

  @property
  def changing_bytes(self) -> np.ndarray:
    """Indices of payload bytes that take more than one value across the capture."""
    if self.count == 0:
      return np.zeros(0, dtype=int)
    return np.flatnonzero((self.payloads != self.payloads[0]).any(axis=0))


@dataclass
class CarSeries:
  times: np.ndarray = field(default_factory=lambda: np.zeros(0))
  v_ego: np.ndarray = field(default_factory=lambda: np.zeros(0))
  a_ego: np.ndarray = field(default_factory=lambda: np.zeros(0))
  gas_pressed: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
  brake_pressed: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
  regen_braking: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))


@dataclass
class BusHealth:
  total_rx: int = 0
  rx_lost: int = 0
  bus_off_cnt: int = 0
  can_speed_kbps: float = 0.0
  can_data_speed_kbps: float = 0.0
  canfd_enabled: bool = False


@dataclass
class Capture:
  messages: dict[tuple[int, int], MessageSeries]
  car: CarSeries
  car_fingerprint: str = ""
  brand: str = ""
  mass_kg: float = 0.0
  bus_health: dict[int, BusHealth] = field(default_factory=dict)
  duration_s: float = 0.0

  def buses(self) -> list[int]:
    return sorted({bus for bus, _ in self.messages})

  def by_bus(self, bus: int) -> list[MessageSeries]:
    return sorted((m for (b, _), m in self.messages.items() if b == bus), key=lambda m: m.address)


def load_capture(identifier: str, buses: list[int] | None = None, max_bus: int = MAX_RX_BUS) -> Capture:
  """identifier: anything openpilot's LogReader accepts (route, segment, local path, URL)."""
  from openpilot.tools.lib.logreader import LogReader, ReadMode

  lr = LogReader(identifier, default_mode=ReadMode.RLOG, sort_by_time=True)

  times: dict[tuple[int, int], list[float]] = defaultdict(list)
  payloads: dict[tuple[int, int], list[bytes]] = defaultdict(list)
  car_t, v, a, gas, brake, regen = [], [], [], [], [], []
  health: dict[int, BusHealth] = {}
  car_fingerprint = brand = ""
  mass = 0.0
  t0 = None
  t_last = 0.0

  for msg in lr:
    t = msg.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    t_last = t
    which = msg.which()
    if which == "can":
      for c in msg.can:
        if c.src >= max_bus or (buses is not None and c.src not in buses):
          continue
        key = (c.src, c.address)
        times[key].append(t)
        payloads[key].append(c.dat)
    elif which == "carState":
      cs = msg.carState
      car_t.append(t)
      v.append(cs.vEgo)
      a.append(cs.aEgo)
      gas.append(cs.gasPressed)
      brake.append(cs.brakePressed)
      regen.append(cs.regenBraking)
    elif which == "carParams":
      cp = msg.carParams
      car_fingerprint = cp.carFingerprint
      brand = cp.brand
      mass = cp.mass
    elif which == "pandaStates":
      for ps in msg.pandaStates:
        for bus, cs_name in enumerate(("canState0", "canState1", "canState2")):
          st = getattr(ps, cs_name)
          health[bus] = BusHealth(
            total_rx=st.totalRxCnt,
            rx_lost=st.totalRxLostCnt,
            bus_off_cnt=st.busOffCnt,
            can_speed_kbps=st.canSpeed / 10.0,
            can_data_speed_kbps=st.canDataSpeed / 10.0,
            canfd_enabled=st.canfdEnabled,
          )

  t0 = t0 or 0.0
  messages: dict[tuple[int, int], MessageSeries] = {}
  for key, ts in times.items():
    raw = payloads[key]
    lengths = np.fromiter((len(p) for p in raw), dtype=np.uint8, count=len(raw))
    max_len = int(lengths.max()) if len(lengths) else 0
    buf = np.zeros((len(raw), max_len), dtype=np.uint8)
    for i, p in enumerate(raw):
      buf[i, :len(p)] = np.frombuffer(p, dtype=np.uint8)
    messages[key] = MessageSeries(key[0], key[1], np.asarray(ts) - t0, buf, lengths)

  car = CarSeries(
    times=np.asarray(car_t) - t0,
    v_ego=np.asarray(v, dtype=np.float64),
    a_ego=np.asarray(a, dtype=np.float64),
    gas_pressed=np.asarray(gas, dtype=bool),
    brake_pressed=np.asarray(brake, dtype=bool),
    regen_braking=np.asarray(regen, dtype=bool),
  )
  return Capture(messages, car, car_fingerprint, brand, mass, health, t_last - t0)


def sample_hold(times: np.ndarray, values: np.ndarray, grid: np.ndarray, max_age: float) -> tuple[np.ndarray, np.ndarray]:
  """Resample an irregular series onto `grid` with zero-order hold. Returns (values_on_grid, valid_mask)."""
  idx = np.searchsorted(times, grid, side="right") - 1
  valid = idx >= 0
  idx_c = np.clip(idx, 0, max(len(times) - 1, 0))
  if len(times) == 0:
    return np.zeros(len(grid), dtype=values.dtype), np.zeros(len(grid), dtype=bool)
  age = grid - times[idx_c]
  valid &= age <= max_age
  return values[idx_c], valid
