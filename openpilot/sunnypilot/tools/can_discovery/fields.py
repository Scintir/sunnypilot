"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Candidate bit-field enumeration and extraction over raw CAN payloads.

Bit numbering follows the DBC convention so a result can be pasted straight into a DBC:
  bit i of byte b has DBC index b*8 + i, i=0 is the LSB of the byte.
  little-endian (@1): start_bit is the LSB of the field, bits ascend.
  big-endian   (@0): start_bit is the MSB of the field, bits walk down within the byte, then to bit 7 of the next byte.
"""
from __future__ import annotations

from dataclasses import dataclass
from collections.abc import Iterator

import numpy as np

DEFAULT_WIDTHS = (8, 12, 16)
DEFAULT_START_STEP = 4  # nibble aligned starts


@dataclass(frozen=True)
class FieldSpec:
  start_bit: int
  size: int
  little_endian: bool
  signed: bool

  @property
  def dbc(self) -> str:
    """DBC-style `start|size@endian sign` fragment."""
    return f"{self.start_bit}|{self.size}@{1 if self.little_endian else 0}{'-' if self.signed else '+'}"

  def bit_positions(self) -> np.ndarray:
    """DBC bit indices of the field, ordered LSB first."""
    if self.little_endian:
      return np.arange(self.start_bit, self.start_bit + self.size)
    pos = []
    b, i = divmod(self.start_bit, 8)
    for _ in range(self.size):
      pos.append(b * 8 + i)
      if i > 0:
        i -= 1
      else:
        b, i = b + 1, 7
    return np.asarray(pos[::-1])  # MSB first was built, return LSB first


def payload_bits(payloads: np.ndarray) -> np.ndarray:
  """(N, L) uint8 -> (N, 8L) uint8 of bits, column index == DBC bit index."""
  payloads = np.ascontiguousarray(payloads, dtype=np.uint8)
  return np.unpackbits(payloads, axis=1, bitorder="little")


def enumerate_specs(payload_len: int, widths=DEFAULT_WIDTHS, start_step=DEFAULT_START_STEP) -> Iterator[FieldSpec]:
  """All (start, width, endian) combos that fit in payload_len bytes; unsigned only (signed is derived later)."""
  nbits = payload_len * 8
  for size in widths:
    for start in range(0, nbits, start_step):
      # little endian: needs start + size <= nbits
      if start + size <= nbits:
        yield FieldSpec(start, size, True, False)
      # big endian: needs the walk to stay in range
      spec = FieldSpec(start, size, False, False)
      if spec.bit_positions().max() < nbits:
        yield spec


def extract(bits: np.ndarray, spec: FieldSpec) -> np.ndarray:
  """Return the field value per row as int64 (signed if spec.signed)."""
  pos = spec.bit_positions()
  weights = (1 << np.arange(spec.size, dtype=np.int64))
  vals = bits[:, pos].astype(np.int64) @ weights
  if spec.signed:
    vals = np.where(vals >= (1 << (spec.size - 1)), vals - (1 << spec.size), vals)
  return vals


def extract_many(bits: np.ndarray, specs: list[FieldSpec]) -> np.ndarray:
  """(N, len(specs)) int64 matrix of unsigned values for equal-size specs (vectorized)."""
  if not specs:
    return np.zeros((bits.shape[0], 0), dtype=np.int64)
  size = specs[0].size
  assert all(s.size == size for s in specs)
  pos = np.stack([s.bit_positions() for s in specs])  # (C, size)
  weights = (1 << np.arange(size, dtype=np.int64))
  gathered = bits[:, pos].astype(np.int64)  # (N, C, size)
  return gathered @ weights


def looks_like_counter(vals: np.ndarray, size: int) -> bool:
  """Mostly +1 steps modulo 2^size between consecutive raw messages."""
  if len(vals) < 16:
    return False
  d = np.diff(vals) % (1 << size)
  return float(np.mean(d == 1)) > 0.7


def looks_like_checksum(vals: np.ndarray, size: int) -> bool:
  """Near-uniform occupancy of the value space with no lag-1 structure."""
  if len(vals) < 64 or size > 16:
    return False
  uniq = len(np.unique(vals))
  if uniq < 0.6 * min(1 << size, len(vals)):
    return False
  x = vals.astype(np.float64)
  x -= x.mean()
  denom = float(np.dot(x, x))
  if denom == 0:
    return False
  lag1 = float(np.dot(x[:-1], x[1:])) / denom
  return abs(lag1) < 0.2


def to_signed(vals: np.ndarray, size: int) -> np.ndarray:
  return np.where(vals >= (1 << (size - 1)), vals - (1 << size), vals)
