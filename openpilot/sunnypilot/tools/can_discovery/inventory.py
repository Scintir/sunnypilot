#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Inventory every CAN message seen on every panda bus in a route, and mark which ones the car's DBC already
knows about. Run this first after a discovery drive to confirm all buses captured cleanly and to see how
much traffic is undocumented.

  python -m openpilot.sunnypilot.tools.can_discovery.inventory "<dongle>|<route>" [--bus 1] [--out inventory.md]
"""
from __future__ import annotations

import argparse
import sys

from openpilot.sunnypilot.tools.can_discovery.capture import Capture, load_capture


def dbc_lookup(car_fingerprint: str) -> dict[int, str]:
  """address -> 'dbc_name:MSG_NAME' for every DBC the platform uses. Empty if the platform is unknown."""
  if not car_fingerprint:
    return {}
  try:
    from opendbc.can.dbc import DBC
    from opendbc.car.interfaces import get_interface_attr
  except ImportError:
    return {}
  dbc_dicts = get_interface_attr("DBC", combine_brands=True)
  dbc_dict = dbc_dicts.get(car_fingerprint)
  if not dbc_dict:
    return {}
  known: dict[int, str] = {}
  for name in set(dbc_dict.values()):
    if not name:
      continue
    try:
      dbc = DBC(name)
    except FileNotFoundError:
      continue
    for addr, msg in dbc.msgs.items():
      known.setdefault(addr, f"{name}:{msg.name}")
  return known


def render(cap: Capture, known: dict[int, str], buses: list[int] | None = None) -> str:
  lines: list[str] = []
  lines.append("# CAN inventory\n")
  lines.append(f"- platform: `{cap.car_fingerprint or 'unknown'}` (brand `{cap.brand or '?'}`), mass {cap.mass_kg:.0f} kg")
  lines.append(f"- duration: {cap.duration_s:.0f} s, carState samples: {len(cap.car.times)}\n")

  lines.append("## Bus health (last pandaStates)\n")
  lines.append("| bus | rx total | rx lost | bus-off cnt | nominal kbps | data kbps | FD |")
  lines.append("|---|---|---|---|---|---|---|")
  for bus in range(3):
    h = cap.bus_health.get(bus)
    if h is None:
      lines.append(f"| {bus} | - | - | - | - | - | - |")
      continue
    fd = "yes" if h.canfd_enabled else "no"
    lines.append(f"| {bus} | {h.total_rx} | {h.rx_lost} | {h.bus_off_cnt} | {h.can_speed_kbps:.0f} | {h.can_data_speed_kbps:.0f} | {fd} |")
  lines.append("")

  for bus in cap.buses():
    if buses is not None and bus not in buses:
      continue
    msgs = cap.by_bus(bus)
    n_known = sum(1 for m in msgs if m.address in known)
    lines.append(f"## Bus {bus}: {len(msgs)} addresses ({n_known} in DBC, {len(msgs) - n_known} undocumented)\n")
    lines.append("| addr | dbc | count | Hz | len | FD | changing bytes |")
    lines.append("|---|---|---|---|---|---|---|")
    for m in msgs:
      lens = sorted({int(x) for x in m.lengths})
      len_s = str(lens[0]) if len(lens) == 1 else f"{lens[0]}-{lens[-1]}"
      cb = m.changing_bytes
      cb_s = ",".join(str(int(i)) for i in cb) if len(cb) <= 12 else f"{len(cb)} of {m.payloads.shape[1]}"
      fd = "y" if m.is_fd else ""
      lines.append(f"| 0x{m.address:X} ({m.address}) | {known.get(m.address, '')} | {m.count} | {m.rate_hz:.1f} | {len_s} | {fd} | {cb_s} |")
    lines.append("")
  return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", help="route, segment, local rlog path or directory, or URL (LogReader identifier)")
  ap.add_argument("--bus", type=int, action="append", help="restrict to this bus (repeatable)")
  ap.add_argument("--out", help="write markdown here instead of stdout")
  args = ap.parse_args(argv)

  cap = load_capture(args.route, buses=args.bus)
  known = dbc_lookup(cap.car_fingerprint)
  text = render(cap, known, args.bus)
  if args.out:
    with open(args.out, "w") as f:
      f.write(text)
    print(f"wrote {args.out}", file=sys.stderr)
  else:
    print(text)
  return 0


if __name__ == "__main__":
  sys.exit(main())
