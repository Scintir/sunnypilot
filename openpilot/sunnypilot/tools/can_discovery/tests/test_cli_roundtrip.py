"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

End-to-end: build a tiny rlog with can/carState/carParams/pandaStates events, then run the inventory and
power_scan CLIs on it.
"""
import os

import numpy as np

from openpilot.cereal import messaging
from openpilot.sunnypilot.tools.can_discovery import inventory, power_scan
from openpilot.sunnypilot.tools.can_discovery.capture import load_capture
from openpilot.tools.lib.logreader import save_log


def _write_rlog(path: str, seconds: float = 30.0):
  msgs = []
  t_ns = 1_000_000_000
  cp = messaging.new_message("carParams")
  cp.logMonoTime = t_ns
  cp.carParams.carFingerprint = "HYUNDAI_IONIQ_5"
  cp.carParams.brand = "hyundai"
  cp.carParams.mass = 2100.0
  msgs.append(cp.as_reader())

  ps = messaging.new_message("pandaStates", 1)
  ps.logMonoTime = t_ns
  ps.pandaStates[0].canState0.totalRxCnt = 1234
  ps.pandaStates[0].canState0.canSpeed = 5000
  ps.pandaStates[0].canState0.canDataSpeed = 20000
  ps.pandaStates[0].canState0.canfdEnabled = True
  msgs.append(ps.as_reader())

  n = int(seconds * 100)
  for i in range(n):
    t = t_ns + i * 10_000_000
    v = 10.0 + 5.0 * np.sin(i / 300.0)
    a = 5.0 * np.cos(i / 300.0) / 3.0
    cs = messaging.new_message("carState")
    cs.logMonoTime = t
    cs.carState.vEgo = float(v)
    cs.carState.aEgo = float(a)
    msgs.append(cs.as_reader())

    p = 2100.0 * a * v
    can = messaging.new_message("can", 3)
    can.logMonoTime = t
    can.can[0].address = 0x2A0
    can.can[0].src = 0
    can.can[0].dat = int(round(p / 50.0)).to_bytes(2, "little", signed=True) + bytes(6)
    can.can[1].address = 0x123
    can.can[1].src = 1
    can.can[1].dat = bytes([i & 0xFF] * 8)
    can.can[2].address = 0x1A0
    can.can[2].src = 2
    can.can[2].dat = bytes(64)  # FD sized
    msgs.append(can.as_reader())
  save_log(path, msgs, compress=True)


def test_cli_roundtrip(tmp_path, capsys):
  rlog = str(tmp_path / "rlog.zst")
  _write_rlog(rlog)

  cap = load_capture(rlog)
  assert cap.car_fingerprint == "HYUNDAI_IONIQ_5" and cap.mass_kg == 2100.0
  assert set(cap.buses()) == {0, 1, 2}
  assert cap.messages[(2, 0x1A0)].is_fd
  assert cap.bus_health[0].canfd_enabled and cap.bus_health[0].can_speed_kbps == 500

  out_inv = str(tmp_path / "inv.md")
  assert inventory.main([rlog, "--out", out_inv]) == 0
  inv = open(out_inv).read()
  assert "## Bus 0" in inv and "## Bus 1" in inv and "## Bus 2" in inv
  assert "0x2A0" in inv and "0x1A0" in inv

  out_scan = str(tmp_path / "scan.md")
  out_csv = str(tmp_path / "scan.csv")
  assert power_scan.main([rlog, "--top", "5", "--out", out_scan, "--csv", out_csv]) == 0
  scan = open(out_scan).read()
  first_row = [ln for ln in scan.splitlines() if ln.startswith("| 1 |")][0]
  assert "0x2A0" in first_row and "`0|16@1-`" in first_row
  assert os.path.getsize(out_csv) > 0
