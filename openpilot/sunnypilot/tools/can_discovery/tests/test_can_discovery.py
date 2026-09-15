"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.sunnypilot.tools.can_discovery.capture import Capture, CarSeries, MessageSeries
from openpilot.sunnypilot.tools.can_discovery.fields import FieldSpec, extract, extract_many, looks_like_checksum, looks_like_counter, payload_bits
from openpilot.sunnypilot.tools.can_discovery.reference import build_reference
from openpilot.sunnypilot.tools.can_discovery.scan import find_voltage_candidates, scan_capture


def _pack_le(value: int, start_bit: int, size: int, buf: bytearray):
  value &= (1 << size) - 1
  for j in range(size):
    bit = start_bit + j
    if (value >> j) & 1:
      buf[bit // 8] |= 1 << (bit % 8)


def _pack_be(value: int, start_bit: int, size: int, buf: bytearray):
  """DBC big-endian: start_bit is the MSB, walk down within the byte then to bit 7 of the next byte."""
  value &= (1 << size) - 1
  b, i = divmod(start_bit, 8)
  for j in range(size):
    if (value >> (size - 1 - j)) & 1:
      buf[b] |= 1 << i
    if i > 0:
      i -= 1
    else:
      b, i = b + 1, 7


class TestFields:
  def test_little_endian_roundtrip(self):
    rng = np.random.default_rng(0)
    vals = rng.integers(0, 1 << 12, size=200)
    payloads = np.zeros((200, 8), dtype=np.uint8)
    for k, v in enumerate(vals):
      buf = bytearray(8)
      _pack_le(int(v), 20, 12, buf)
      payloads[k] = np.frombuffer(bytes(buf), dtype=np.uint8)
    bits = payload_bits(payloads)
    got = extract(bits, FieldSpec(20, 12, True, False))
    assert np.array_equal(got, vals)

  def test_big_endian_roundtrip_matches_dbc_convention(self):
    # classic Motorola 16-bit at 7|16@0: MSB is bit 7 of byte 0, LSB is bit 0 of byte 1
    payloads = np.array([[0x12, 0x34, 0, 0, 0, 0, 0, 0]], dtype=np.uint8)
    got = extract(payload_bits(payloads), FieldSpec(7, 16, False, False))
    assert got[0] == 0x1234
    # unaligned Motorola start: 3|12@0 spans byte0 bits 3..0 and byte1 bits 7..0
    buf = bytearray(8)
    _pack_be(0xABC, 3, 12, buf)
    got = extract(payload_bits(np.frombuffer(bytes(buf), dtype=np.uint8)[None, :]), FieldSpec(3, 12, False, False))
    assert got[0] == 0xABC

  def test_signed(self):
    payloads = np.array([[0xFF, 0xFF, 0, 0, 0, 0, 0, 0], [0x00, 0x80, 0, 0, 0, 0, 0, 0]], dtype=np.uint8)
    got = extract(payload_bits(payloads), FieldSpec(0, 16, True, True))
    assert list(got) == [-1, -32768]

  def test_extract_many_matches_extract(self):
    rng = np.random.default_rng(1)
    payloads = rng.integers(0, 256, size=(64, 16), dtype=np.uint8)
    bits = payload_bits(payloads)
    specs = [FieldSpec(s, 16, le, False) for s in range(0, 100, 4) for le in (True, False)]
    many = extract_many(bits, specs)
    for j, spec in enumerate(specs):
      assert np.array_equal(many[:, j], extract(bits, spec))

  def test_counter_and_checksum_heuristics(self):
    counter = np.arange(500) % 16
    assert looks_like_counter(counter, 4)
    rng = np.random.default_rng(2)
    checksum = rng.integers(0, 256, size=500)
    assert looks_like_checksum(checksum, 8)
    smooth = (np.sin(np.linspace(0, 20, 500)) * 100 + 128).astype(int)
    assert not looks_like_counter(smooth, 8)
    assert not looks_like_checksum(smooth, 8)


def _synthetic_capture(seed: int = 3) -> tuple[Capture, int, int]:
  """A drive with a planted 16-bit LE signed 'battery power' field (50 W/count) on 0x2A0 bus 0, a pack voltage on
  0x2A1, and a distractor message that is speed. Returns (capture, power_addr, voltage_addr)."""
  rng = np.random.default_rng(seed)
  mass = 2000.0
  duration = 240.0
  t = np.arange(0, duration, 0.01)
  # speed profile: accel / cruise / regen cycles
  a = 2.0 * np.sin(2 * np.pi * t / 40.0)
  v = np.clip(np.cumsum(a) * 0.01 + 15.0, 0.0, None)
  a = np.gradient(v, 0.01)
  car = CarSeries(t, v, a, np.zeros_like(t, bool), np.zeros_like(t, bool), np.zeros_like(t, bool))

  p_true = mass * a * v + mass * 9.81 * 0.01 * v + 0.5 * 1.2 * 0.75 * v ** 3
  p_true += rng.normal(0, 500, size=len(t))

  def series(addr, rate, builder):
    ts = np.arange(0, duration, 1.0 / rate)
    idx = np.searchsorted(t, ts)
    idx = np.clip(idx, 0, len(t) - 1)
    payloads = np.zeros((len(ts), 8), dtype=np.uint8)
    for k, i in enumerate(idx):
      buf = bytearray(8)
      builder(i, k, buf)
      payloads[k] = np.frombuffer(bytes(buf), dtype=np.uint8)
    return MessageSeries(0, addr, ts, payloads, np.full(len(ts), 8, dtype=np.uint8))

  def power_msg(i, k, buf):
    _pack_le(k & 0xF, 0, 4, buf)                       # counter
    _pack_le(int(round(p_true[i] / 50.0)), 24, 16, buf)  # signed power, 50 W/count
    _pack_le(int(rng.integers(0, 256)), 56, 8, buf)         # checksum-ish noise

  def volt_msg(i, k, buf):
    _pack_le(int(round((650.0 - 0.0001 * p_true[i]) / 0.125)), 8, 16, buf)  # ~650 V, 0.125 V/count, sags with power
    _pack_le(int(rng.integers(0, 4)), 40, 2, buf)

  def speed_msg(i, k, buf):
    _pack_le(int(v[i] * 100), 0, 16, buf)

  msgs = {
    (0, 0x2A0): series(0x2A0, 50, power_msg),
    (0, 0x2A1): series(0x2A1, 10, volt_msg),
    (0, 0x2B0): series(0x2B0, 100, speed_msg),
  }
  cap = Capture(msgs, car, "SYNTH", "synth", mass, {}, duration)
  return cap, 0x2A0, 0x2A1


class TestScan:
  def test_planted_power_field_ranks_first(self):
    cap, power_addr, _ = _synthetic_capture()
    ref = build_reference(cap)
    cands = scan_capture(cap, ref, top=10)
    assert cands, "no candidates"
    best = cands[0]
    assert best.address == power_addr
    assert best.spec.start_bit == 24 and best.spec.size == 16 and best.spec.little_endian and best.spec.signed
    assert best.r_power > 0.95
    # slope is counts per watt -> factor ~ 50 W/count
    assert abs(1.0 / best.slope - 50.0) < 5.0

  def test_speed_distractor_not_top(self):
    cap, power_addr, _ = _synthetic_capture()
    ref = build_reference(cap)
    cands = scan_capture(cap, ref, top=5)
    assert all(c.address == power_addr for c in cands[:2])

  def test_voltage_candidate_found(self):
    cap, _, volt_addr = _synthetic_capture()
    volts = find_voltage_candidates(cap)
    hits = [c for c in volts if c.address == volt_addr and c.spec.start_bit == 8 and c.spec.size == 16]
    assert hits
    assert abs(hits[0].median_v - 650.0) < 5.0 and hits[0].scale == 0.125
