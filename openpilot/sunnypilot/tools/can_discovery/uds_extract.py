#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Pull the BMS UDS responses (0x7EC on bus 1) out of a route's rlogs, reassemble the ISO-TP transfers, and dump
the raw payload plus the tentative decode as CSV. Use this to confirm the byte layout on a new platform before
trusting the on-device decode.

  python -m openpilot.sunnypilot.tools.can_discovery.uds_extract "<dongle>|<route>" --csv bms.csv
"""
from __future__ import annotations

import argparse
import csv
import sys

from openpilot.sunnypilot.selfdrive.car.hyundai.bms_uds import BMS_RX_ADDR, OBD_BUS, IsoTpReceiver, decode_bms_0101


def extract(identifier: str, rx_addr: int = BMS_RX_ADDR, bus: int = OBD_BUS):
  from openpilot.tools.lib.logreader import LogReader, ReadMode

  lr = LogReader(identifier, default_mode=ReadMode.RLOG, sort_by_time=True)
  rx = IsoTpReceiver()
  t0 = None
  rows = []
  for msg in lr:
    if msg.which() != "can":
      continue
    t = msg.logMonoTime * 1e-9
    t0 = t if t0 is None else t0
    for c in msg.can:
      if c.src != bus or c.address != rx_addr:
        continue
      rx.push(bytes(c.dat))
      if rx.complete is None:
        continue
      payload = rx.complete
      rx.complete = None
      if not payload or payload[0] == 0x7F:
        rows.append((t - t0, "negative", payload.hex(), None))
        continue
      service = payload[0] - 0x40
      echo_len = 3 if service == 0x22 else 2
      data = payload[echo_len:]
      rows.append((t - t0, f"0x{service:02X}", data.hex(), decode_bms_0101(data)))
  return rows


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route")
  ap.add_argument("--csv", help="write rows here (default stdout)")
  ap.add_argument("--rx-addr", type=lambda x: int(x, 0), default=BMS_RX_ADDR)
  ap.add_argument("--bus", type=int, default=OBD_BUS)
  args = ap.parse_args(argv)

  rows = extract(args.route, args.rx_addr, args.bus)
  out = open(args.csv, "w", newline="") if args.csv else sys.stdout
  w = csv.writer(out)
  w.writerow(["t_s", "service", "raw_hex", "plausible", "soc_pct", "pack_v", "pack_a", "pack_kw", "chg_limit_kw", "dchg_limit_kw",
              "max_t", "min_t", "max_cell_v", "min_cell_v", "aux_v", "rpm1", "rpm2"])
  for t, svc, raw, d in rows:
    if d is None:
      w.writerow([f"{t:.3f}", svc, raw] + [""] * 14)
      continue
    w.writerow([f"{t:.3f}", svc, raw, d.plausible, d.soc, d.pack_voltage, d.pack_current, f"{d.pack_power_kw:.2f}",
                d.available_charge_power, d.available_discharge_power, d.max_temp, d.min_temp,
                f"{d.max_cell_voltage:.2f}", f"{d.min_cell_voltage:.2f}", f"{d.aux_voltage:.1f}", d.motor_rpm1, d.motor_rpm2])
  if args.csv:
    out.close()
    print(f"wrote {len(rows)} responses to {args.csv}", file=sys.stderr)
  return 0


if __name__ == "__main__":
  sys.exit(main())
