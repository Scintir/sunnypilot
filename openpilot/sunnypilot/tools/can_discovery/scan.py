"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Rank candidate bit-fields across all captured messages by how well they track the reference tractive power.
Also flags fields that look like a pack voltage (a few hundred volts, nearly constant) so V*I pairs can be
spotted by eye.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from openpilot.sunnypilot.tools.can_discovery.capture import Capture, MessageSeries, sample_hold
from openpilot.sunnypilot.tools.can_discovery.fields import (FieldSpec, enumerate_specs, extract_many, looks_like_checksum,
                                                             looks_like_counter, payload_bits, to_signed)
from openpilot.sunnypilot.tools.can_discovery.reference import ReferenceSeries

# raw*scale medians inside this window are treated as plausible pack voltages
VOLTAGE_RANGE_V = (200.0, 1000.0)
VOLTAGE_SCALES = (1.0, 0.5, 0.25, 0.2, 0.125, 0.1, 0.01)


@dataclass
class Candidate:
  bus: int
  address: int
  spec: FieldSpec
  r_power: float      # Pearson r vs P_ref
  r_inertial: float   # Pearson r vs m*a*v
  r_speed: float      # Pearson r vs v (to spot fields that are just speed)
  slope: float        # least-squares raw units per watt (1/slope ~ DBC factor if the field is power in W)
  n: int
  raw_min: int
  raw_max: int
  unique: int

  @property
  def score(self) -> float:
    # prefer power correlation, penalize fields that are explained by speed alone
    return abs(self.r_power) - 0.3 * max(abs(self.r_speed) - abs(self.r_power), 0.0)


@dataclass
class VoltageCandidate:
  bus: int
  address: int
  spec: FieldSpec
  scale: float
  median_v: float
  cv: float           # coefficient of variation of raw values
  unique: int


def _pearson_matrix(x: np.ndarray, Y: np.ndarray) -> np.ndarray:
  """r between vector x (N,) and each column of Y (N, C)."""
  xc = x - x.mean()
  Yc = Y - Y.mean(axis=0)
  denom = np.sqrt(np.dot(xc, xc)) * np.sqrt((Yc * Yc).sum(axis=0))
  with np.errstate(invalid="ignore", divide="ignore"):
    r = (xc @ Yc) / denom
  return np.nan_to_num(r, nan=0.0)


def _slope(x: np.ndarray, y: np.ndarray) -> float:
  xc = x - x.mean()
  d = float(np.dot(xc, xc))
  return float(np.dot(xc, y - y.mean()) / d) if d > 0 else 0.0


def scan_message(m: MessageSeries, ref: ReferenceSeries, min_unique: int = 4,
                 widths=(8, 12, 16), start_step: int = 4, batch: int = 256) -> list[Candidate]:
  if m.count < 32 or m.payloads.shape[1] == 0:
    return []
  # resample: index of the last frame at/before each grid tick
  max_age = max(2.0 / m.rate_hz, 0.1) if m.rate_hz > 0 else 1.0
  row_idx, ok = sample_hold(m.times, np.arange(m.count), ref.grid, max_age)
  mask = ok & ref.valid
  if mask.sum() < 64:
    return []
  rows = row_idx[mask]
  p = ref.p_ref[mask]
  pi = ref.p_inertial[mask]
  v = ref.v[mask]

  bits_all = payload_bits(m.payloads)
  out: list[Candidate] = []
  specs_by_size: dict[int, list[FieldSpec]] = {}
  for s in enumerate_specs(m.payloads.shape[1], widths, start_step):
    specs_by_size.setdefault(s.size, []).append(s)

  for size, specs in specs_by_size.items():
    for i in range(0, len(specs), batch):
      chunk = specs[i:i + batch]
      raw_all = extract_many(bits_all, chunk)          # (N_raw, C) unsigned, every frame
      keep = []
      for c, spec in enumerate(chunk):
        col = raw_all[:, c]
        nu = len(np.unique(col))
        if nu < min_unique or looks_like_counter(col, size) or looks_like_checksum(col, size):
          continue
        keep.append((c, spec, nu))
      if not keep:
        continue
      cols = [c for c, _, _ in keep]
      sampled_u = raw_all[rows][:, cols].astype(np.float64)
      sampled_s = to_signed(raw_all[rows][:, cols], size).astype(np.float64)
      for variant, Y in ((False, sampled_u), (True, sampled_s)):
        r_p = _pearson_matrix(p, Y)
        r_i = _pearson_matrix(pi, Y)
        r_v = _pearson_matrix(v, Y)
        for j, (c, spec, nu) in enumerate(keep):
          if variant and raw_all[:, c].max() < (1 << (size - 1)):
            continue  # MSB never set: signed == unsigned, skip duplicate
          fs = FieldSpec(spec.start_bit, spec.size, spec.little_endian, variant)
          out.append(Candidate(m.bus, m.address, fs, float(r_p[j]), float(r_i[j]), float(r_v[j]),
                               _slope(p, Y[:, j]), int(mask.sum()),
                               int(Y[:, j].min()), int(Y[:, j].max()), nu))
  return out


def scan_capture(cap: Capture, ref: ReferenceSeries, buses: list[int] | None = None, min_rate_hz: float = 2.0,
                 top: int = 40, **kw) -> list[Candidate]:
  cands: list[Candidate] = []
  for (bus, _), m in cap.messages.items():
    if buses is not None and bus not in buses:
      continue
    if m.rate_hz < min_rate_hz:
      continue
    cands.extend(scan_message(m, ref, **kw))
  # best score first; on a tie prefer the wider field (a 12-bit view of a 16-bit signal scores identically)
  cands.sort(key=lambda c: (round(c.score, 6), c.spec.size), reverse=True)
  # one entry per (bus, addr, start, endian): narrower/signed/unsigned views of the same bits are noise
  seen: set[tuple[int, int, int, bool]] = set()
  dedup: list[Candidate] = []
  for c in cands:
    key = (c.bus, c.address, c.spec.start_bit, c.spec.little_endian)
    if key in seen:
      continue
    seen.add(key)
    dedup.append(c)
    if len(dedup) >= top:
      break
  return dedup


def find_voltage_candidates(cap: Capture, buses: list[int] | None = None, min_rate_hz: float = 1.0,
                            max_cv: float = 0.06, min_unique: int = 4) -> list[VoltageCandidate]:
  out: list[VoltageCandidate] = []
  for (bus, _), m in cap.messages.items():
    if buses is not None and bus not in buses:
      continue
    if m.rate_hz < min_rate_hz or m.count < 32 or m.payloads.shape[1] == 0:
      continue
    bits = payload_bits(m.payloads)
    specs_by_size: dict[int, list[FieldSpec]] = {}
    for s in enumerate_specs(m.payloads.shape[1], (12, 16), 4):
      specs_by_size.setdefault(s.size, []).append(s)
    for specs in specs_by_size.values():
      raw = extract_many(bits, specs).astype(np.float64)
      med = np.median(raw, axis=0)
      std = raw.std(axis=0)
      for j, spec in enumerate(specs):
        if med[j] <= 0:
          continue
        nu = len(np.unique(raw[:, j]))
        if nu < min_unique or std[j] / med[j] > max_cv:
          continue
        for scale in VOLTAGE_SCALES:
          val = med[j] * scale
          if VOLTAGE_RANGE_V[0] <= val <= VOLTAGE_RANGE_V[1]:
            out.append(VoltageCandidate(bus, m.address, spec, scale, float(val), float(std[j] / med[j]), nu))
            break
  out.sort(key=lambda c: (c.cv, -c.unique))
  return out
