"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Read-only UDS polling of the Hyundai/Kia HV battery BMS (0x7E4 -> 0x7EC) on the OBD-II port (panda bus 1).

card drives this at 100 Hz: `rx()` gets every CAN frame card already drains, `tx()` returns the frames to append
to the next sendcan batch. The panda safety model only lets through single-frame read requests (0x22 / 0x21)
and flow-control frames to 0x7E4, so nothing here can write to the car.

Decoding follows the community HKMC BMS layout for DID 0x0101 (JejuSoul/OBD-PIDs-for-HKMC-EVs); byte letters in
that table start after the "62 01 01" / "61 01" echo, which is what `data` holds here. The 2022 Santa Fe PHEV has
not been confirmed against that table, so every decoded value is plausibility checked and the raw payload is
published alongside for offline verification.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from opendbc.car.can_definitions import CanData

BMS_TX_ADDR = 0x7E4
BMS_RX_ADDR = 0x7EC
OBD_BUS = 1

SERVICE_READ_DATA_BY_ID = 0x22
SERVICE_READ_DATA_BY_LOCAL_ID = 0x21
DID_BMS_MAIN = 0x0101

# ISO-TP protocol control information
PCI_SINGLE = 0x0
PCI_FIRST = 0x1
PCI_CONSECUTIVE = 0x2
PCI_FLOW_CONTROL = 0x3

FLOW_CONTROL_FRAME = bytes([0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

DEFAULT_PERIOD_FRAMES = 10      # 100 ms between requests at card's 100 Hz
RESPONSE_TIMEOUT_FRAMES = 30    # 300 ms
FALLBACK_AFTER_FAILURES = 3     # switch 0x22 <-> 0x21 after this many consecutive failures


def build_request(service: int) -> bytes:
  if service == SERVICE_READ_DATA_BY_ID:
    payload = bytes([service, DID_BMS_MAIN >> 8, DID_BMS_MAIN & 0xFF])
  else:
    payload = bytes([service, DID_BMS_MAIN & 0xFF])
  frame = bytes([len(payload)]) + payload
  return frame + bytes(8 - len(frame))


def _s8(b: int) -> int:
  return b - 256 if b >= 128 else b


@dataclass
class BmsDecoded:
  pack_voltage: float = 0.0
  pack_current: float = 0.0
  soc: float = 0.0
  available_charge_power: float = 0.0
  available_discharge_power: float = 0.0
  max_temp: float = 0.0
  min_temp: float = 0.0
  max_cell_voltage: float = 0.0
  min_cell_voltage: float = 0.0
  aux_voltage: float = 0.0
  motor_rpm1: float = 0.0
  motor_rpm2: float = 0.0
  bms_main_relay: bool = False
  charging: bool = False
  plausible: bool = False

  @property
  def pack_power_kw(self) -> float:
    return self.pack_voltage * self.pack_current / 1000.0


def decode_bms_0101(data: bytes) -> BmsDecoded:
  """`data` is the payload after the DID echo. Needs at least 59 bytes for the full table; shorter payloads
  decode what they can and are marked implausible."""
  d = BmsDecoded()
  if len(data) < 30:
    return d
  d.soc = data[4] / 2.0
  d.available_charge_power = ((data[5] << 8) | data[6]) / 100.0
  d.available_discharge_power = ((data[7] << 8) | data[8]) / 100.0
  flags = data[9]
  d.bms_main_relay = bool(flags & 0x01)
  d.charging = bool(flags & 0x80)
  d.pack_current = ((_s8(data[10]) * 256) + data[11]) / 10.0
  d.pack_voltage = ((data[12] << 8) | data[13]) / 10.0
  d.max_temp = float(_s8(data[14]))
  d.min_temp = float(_s8(data[15]))
  d.max_cell_voltage = data[23] / 50.0
  d.min_cell_voltage = data[25] / 50.0
  d.aux_voltage = data[29] * 0.1
  if len(data) >= 57:
    d.motor_rpm1 = float((_s8(data[53]) * 256) + data[54])
    d.motor_rpm2 = float((_s8(data[55]) * 256) + data[56])

  d.plausible = (
    0.0 <= d.soc <= 100.0 and
    150.0 <= d.pack_voltage <= 900.0 and
    -600.0 <= d.pack_current <= 600.0 and
    2.0 <= d.min_cell_voltage <= d.max_cell_voltage <= 4.5 and
    -40.0 <= d.min_temp <= d.max_temp <= 100.0 and
    8.0 <= d.aux_voltage <= 16.0
  )
  return d


class IsoTpReceiver:
  """Reassembles one ISO-TP transfer from a single sender. Feed frames with `push`; `complete` holds the payload."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.buf = bytearray()
    self.expected_len = 0
    self.next_seq = 0
    self.active = False
    self.complete: bytes | None = None
    self.need_flow_control = False

  def push(self, dat: bytes) -> None:
    if len(dat) < 1:
      return
    pci = dat[0] >> 4
    if pci == PCI_SINGLE:
      n = dat[0] & 0x0F
      self.buf = bytearray(dat[1:1 + n])
      self.expected_len = n
      self.active = False
      self.complete = bytes(self.buf)
    elif pci == PCI_FIRST:
      self.expected_len = ((dat[0] & 0x0F) << 8) | dat[1]
      self.buf = bytearray(dat[2:])
      self.next_seq = 1
      self.active = True
      self.complete = None
      self.need_flow_control = True
    elif pci == PCI_CONSECUTIVE and self.active:
      seq = dat[0] & 0x0F
      if seq != self.next_seq:
        self.reset()
        return
      self.next_seq = (self.next_seq + 1) & 0x0F
      self.buf.extend(dat[1:])
      if len(self.buf) >= self.expected_len:
        self.complete = bytes(self.buf[:self.expected_len])
        self.active = False


@dataclass
class BmsPollerState:
  service: int = SERVICE_READ_DATA_BY_ID
  request_count: int = 0
  response_count: int = 0
  timeout_count: int = 0
  negative_response_count: int = 0
  consecutive_failures: int = 0
  last_response_frame: int | None = None
  last_raw: bytes = b""
  last_service: int = 0
  decoded: BmsDecoded = field(default_factory=BmsDecoded)


class BmsUdsPoller:
  def __init__(self, period_frames: int = DEFAULT_PERIOD_FRAMES, timeout_frames: int = RESPONSE_TIMEOUT_FRAMES):
    self.period_frames = period_frames
    self.timeout_frames = timeout_frames
    self.state = BmsPollerState()
    self.rx_buf = IsoTpReceiver()
    self.frame = 0
    self.request_sent_frame: int | None = None
    self.next_request_frame = 0

  # ---- card hooks -------------------------------------------------------------------------------------------

  def rx(self, can_packets) -> None:
    """Feed the CAN packets card drained this tick: `[(nanos, [(address, dat, src), ...]), ...]` as returned by
    can_capnp_to_list. Frames are plain tuples, not CanData, so unpack positionally."""
    for _, frames in can_packets:
      for address, dat, src in frames:
        if src == OBD_BUS and address == BMS_RX_ADDR:
          self._on_bms_frame(bytes(dat))

  def tx(self) -> list[CanData]:
    """Frames to append to this tick's sendcan. Call once per card tick after rx()."""
    self.frame += 1
    out: list[CanData] = []
    st = self.state

    if self.rx_buf.need_flow_control:
      self.rx_buf.need_flow_control = False
      out.append(CanData(BMS_TX_ADDR, FLOW_CONTROL_FRAME, OBD_BUS))

    if self.request_sent_frame is not None and self.frame - self.request_sent_frame > self.timeout_frames:
      st.timeout_count += 1
      self._failure()
      self.request_sent_frame = None
      self.rx_buf.reset()

    if self.request_sent_frame is None and self.frame >= self.next_request_frame:
      out.append(CanData(BMS_TX_ADDR, build_request(st.service), OBD_BUS))
      st.request_count += 1
      self.request_sent_frame = self.frame
      self.next_request_frame = self.frame + self.period_frames
    return out

  # ---- internals --------------------------------------------------------------------------------------------

  def _failure(self) -> None:
    st = self.state
    st.consecutive_failures += 1
    if st.consecutive_failures >= FALLBACK_AFTER_FAILURES:
      st.service = SERVICE_READ_DATA_BY_LOCAL_ID if st.service == SERVICE_READ_DATA_BY_ID else SERVICE_READ_DATA_BY_ID
      st.consecutive_failures = 0

  def _on_bms_frame(self, dat: bytes) -> None:
    if self.request_sent_frame is None:
      return  # unsolicited / stale
    self.rx_buf.push(dat)
    payload = self.rx_buf.complete
    if payload is None:
      return
    self.rx_buf.complete = None
    st = self.state
    self.request_sent_frame = None

    if len(payload) >= 3 and payload[0] == 0x7F:
      st.negative_response_count += 1
      self._failure()
      return

    expected = st.service + 0x40
    if len(payload) < 3 or payload[0] != expected:
      self._failure()
      return
    echo_len = 3 if st.service == SERVICE_READ_DATA_BY_ID else 2
    data = payload[echo_len:]
    st.response_count += 1
    st.consecutive_failures = 0
    st.last_response_frame = self.frame
    st.last_raw = data
    st.last_service = st.service
    st.decoded = decode_bms_0101(data)

  # ---- publishing -------------------------------------------------------------------------------------------

  def fill_msg(self, msg) -> None:
    """Populate a cereal EvBatteryStateSP builder."""
    st = self.state
    d = st.decoded
    age_frames = (self.frame - st.last_response_frame) if st.last_response_frame is not None else None
    msg.valid = age_frames is not None and age_frames <= 5 * self.period_frames
    msg.decodeValid = bool(msg.valid and d.plausible)
    msg.service = st.last_service
    msg.responseAgeMs = int(age_frames * 10) if age_frames is not None else 0
    msg.requestCount = st.request_count
    msg.responseCount = st.response_count
    msg.timeoutCount = st.timeout_count
    msg.negativeResponseCount = st.negative_response_count
    msg.packVoltage = d.pack_voltage
    msg.packCurrent = d.pack_current
    msg.packPower = d.pack_power_kw
    msg.soc = d.soc
    msg.availableChargePower = d.available_charge_power
    msg.availableDischargePower = d.available_discharge_power
    msg.maxTemp = d.max_temp
    msg.minTemp = d.min_temp
    msg.maxCellVoltage = d.max_cell_voltage
    msg.minCellVoltage = d.min_cell_voltage
    msg.auxVoltage = d.aux_voltage
    msg.motorRpm1 = d.motor_rpm1
    msg.motorRpm2 = d.motor_rpm2
    msg.bmsMainRelay = d.bms_main_relay
    msg.charging = d.charging
    msg.rawData = st.last_raw
