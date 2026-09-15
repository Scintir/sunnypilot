"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.can_definitions import CanData
from openpilot.sunnypilot.selfdrive.car.hyundai.bms_uds import (BMS_RX_ADDR, BMS_TX_ADDR, FLOW_CONTROL_FRAME, OBD_BUS,
                                                                SERVICE_READ_DATA_BY_ID, SERVICE_READ_DATA_BY_LOCAL_ID,
                                                                BmsUdsPoller, IsoTpReceiver, build_request, decode_bms_0101)


def make_payload(soc=62.0, chg_kw=45.5, dchg_kw=120.0, current_a=-83.4, volt_v=352.7, max_t=31, min_t=27,
                 max_cell=3.71, min_cell=3.68, aux=13.9, rpm1=2450, rpm2=-120, relay=True, charging=False) -> bytes:
  """Build a 0x0101 payload (after the DID echo) using the community HKMC byte layout."""
  d = bytearray(60)
  d[0:4] = b"\xFF\xF7\xE7\xFF"
  d[4] = int(soc * 2)
  d[5:7] = int(chg_kw * 100).to_bytes(2, "big")
  d[7:9] = int(dchg_kw * 100).to_bytes(2, "big")
  d[9] = (0x01 if relay else 0) | (0x80 if charging else 0)
  d[10:12] = int(round(current_a * 10)).to_bytes(2, "big", signed=True)
  d[12:14] = int(round(volt_v * 10)).to_bytes(2, "big")
  d[14] = max_t & 0xFF
  d[15] = min_t & 0xFF
  d[23] = int(round(max_cell * 50))
  d[25] = int(round(min_cell * 50))
  d[29] = int(round(aux * 10))
  d[53:55] = int(rpm1).to_bytes(2, "big", signed=True)
  d[55:57] = int(rpm2).to_bytes(2, "big", signed=True)
  return bytes(d)


def isotp_frames(payload: bytes) -> list[bytes]:
  """Split a response payload into padded 8-byte ISO-TP frames."""
  if len(payload) <= 7:
    f = bytes([len(payload)]) + payload
    return [f + bytes(8 - len(f))]
  frames = [bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6]]
  rest = payload[6:]
  seq = 1
  while rest:
    chunk, rest = rest[:7], rest[7:]
    f = bytes([0x20 | seq]) + chunk
    frames.append(f + bytes(8 - len(f)))
    seq = (seq + 1) & 0x0F
  return frames


class FakeBms:
  """Answers 0x22 0101 (or 0x21 01 when `local_id`) with a multi-frame response, honoring flow control."""

  def __init__(self, payload: bytes, local_id: bool = False, negative_for=()):
    self.payload = payload
    self.local_id = local_id
    self.negative_for = set(negative_for)
    self.pending: list[bytes] = []
    self.requests = 0

  def handle(self, tx: list[CanData]) -> list[list[CanData]]:
    out: list[CanData] = []
    for f in tx:
      assert f.address == BMS_TX_ADDR and f.src == OBD_BUS and len(f.dat) == 8
      if f.dat[0] == 0x30:
        out.extend(CanData(BMS_RX_ADDR, fr, OBD_BUS) for fr in self.pending)
        self.pending = []
      elif f.dat[0] & 0xF0 == 0x00:
        self.requests += 1
        service = f.dat[1]
        if service in self.negative_for:
          out.append(CanData(BMS_RX_ADDR, bytes([0x03, 0x7F, service, 0x31, 0, 0, 0, 0]), OBD_BUS))
          continue
        if (service == SERVICE_READ_DATA_BY_LOCAL_ID) != self.local_id:
          out.append(CanData(BMS_RX_ADDR, bytes([0x03, 0x7F, service, 0x11, 0, 0, 0, 0]), OBD_BUS))
          continue
        echo = bytes([service + 0x40, 0x01, 0x01]) if service == SERVICE_READ_DATA_BY_ID else bytes([service + 0x40, 0x01])
        frames = isotp_frames(echo + self.payload)
        out.append(CanData(BMS_RX_ADDR, frames[0], OBD_BUS))
        self.pending = frames[1:]
    return [out] if out else []


class TestDecode:
  def test_decode_roundtrip(self):
    d = decode_bms_0101(make_payload())
    assert d.plausible
    assert d.soc == 62.0
    assert abs(d.available_charge_power - 45.5) < 1e-6
    assert abs(d.available_discharge_power - 120.0) < 1e-6
    assert abs(d.pack_current - (-83.4)) < 1e-6
    assert abs(d.pack_voltage - 352.7) < 1e-6
    assert d.max_temp == 31 and d.min_temp == 27
    assert abs(d.max_cell_voltage - 3.72) < 0.011 and abs(d.min_cell_voltage - 3.68) < 0.011
    assert abs(d.aux_voltage - 13.9) < 1e-6
    assert d.motor_rpm1 == 2450 and d.motor_rpm2 == -120
    assert d.bms_main_relay and not d.charging
    assert abs(d.pack_power_kw - (352.7 * -83.4 / 1000)) < 1e-6

  def test_implausible_flagged(self):
    assert not decode_bms_0101(bytes(60)).plausible
    assert not decode_bms_0101(b"\x00" * 10).plausible
    assert not decode_bms_0101(make_payload(volt_v=20.0)).plausible


class TestIsoTp:
  def test_single_and_multi(self):
    r = IsoTpReceiver()
    r.push(b"\x03\x62\x01\x01\x00\x00\x00\x00")
    assert r.complete == b"\x62\x01\x01"
    payload = bytes(range(40))
    r = IsoTpReceiver()
    frames = isotp_frames(payload)
    r.push(frames[0])
    assert r.need_flow_control and r.complete is None
    for f in frames[1:]:
      r.push(f)
    assert r.complete == payload

  def test_sequence_error_resets(self):
    r = IsoTpReceiver()
    frames = isotp_frames(bytes(range(40)))
    r.push(frames[0])
    r.push(frames[2])  # skipped seq 1
    assert not r.active and r.complete is None


def run(poller: BmsUdsPoller, bms, ticks: int):
  rx: list[list[CanData]] = []
  for _ in range(ticks):
    poller.rx(rx)
    tx = poller.tx()
    rx = bms.handle(tx)


class TestPoller:
  def test_polls_and_decodes(self):
    poller = BmsUdsPoller()
    bms = FakeBms(make_payload(soc=55.0))
    run(poller, bms, 100)
    st = poller.state
    assert st.request_count >= 9 and st.response_count == st.request_count
    assert st.timeout_count == 0 and st.negative_response_count == 0
    assert st.decoded.plausible and st.decoded.soc == 55.0
    assert st.last_service == SERVICE_READ_DATA_BY_ID

    class Msg:
      pass
    m = Msg()
    poller.fill_msg(m)
    assert m.valid and m.decodeValid and m.soc == 55.0 and m.rawData == make_payload(soc=55.0)

  def test_falls_back_to_local_id(self):
    poller = BmsUdsPoller()
    bms = FakeBms(make_payload(), local_id=True)
    run(poller, bms, 200)
    st = poller.state
    assert st.negative_response_count >= 3
    assert st.last_service == SERVICE_READ_DATA_BY_LOCAL_ID
    assert st.response_count >= 5 and st.decoded.plausible

  def test_timeout_when_silent(self):
    poller = BmsUdsPoller()

    class Silent:
      def handle(self, tx):
        return []
    run(poller, Silent(), 200)
    st = poller.state
    assert st.timeout_count >= 4 and st.response_count == 0

    class Msg:
      pass
    m = Msg()
    poller.fill_msg(m)
    assert not m.valid and not m.decodeValid

  def test_only_requests_and_flow_control_are_sent(self):
    poller = BmsUdsPoller()
    bms = FakeBms(make_payload())
    sent: list[bytes] = []
    rx: list[list[CanData]] = []
    for _ in range(100):
      poller.rx(rx)
      tx = poller.tx()
      sent.extend(f.dat for f in tx)
      rx = bms.handle(tx)
    assert sent
    for f in sent:
      assert f in (build_request(SERVICE_READ_DATA_BY_ID), build_request(SERVICE_READ_DATA_BY_LOCAL_ID), FLOW_CONTROL_FRAME)
