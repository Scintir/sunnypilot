#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Rank every candidate bit-field in a route by correlation with estimated tractive power, and list fields that
look like a pack voltage. The top rows are the addresses to open in cabana next.

  python -m openpilot.sunnypilot.tools.can_discovery.power_scan "<dongle>|<route>" [--bus 0] [--top 40] [--mass 2100]
"""
from __future__ import annotations

import argparse
import csv
import sys

from openpilot.sunnypilot.tools.can_discovery.capture import load_capture
from openpilot.sunnypilot.tools.can_discovery.inventory import dbc_lookup
from openpilot.sunnypilot.tools.can_discovery.reference import build_reference
from openpilot.sunnypilot.tools.can_discovery.scan import Candidate, VoltageCandidate, find_voltage_candidates, scan_capture


def render(cands: list[Candidate], volts: list[VoltageCandidate], known: dict[int, str], ref_n: int) -> str:
  legend = "\n".join([
    "`slope` is raw counts per watt; if a field really is pack power in W its DBC factor is roughly 1/slope.",
    "Negative r with power means the sign convention is charge-positive.",
    "High r_speed with low r_power means the field is just speed.",
    "",
  ])
  lines = ["# Power-correlated field scan\n", f"reference samples used: {ref_n}\n",
           "## Top fields by |r| vs estimated tractive power\n", legend,
           "| # | bus | addr | dbc | field (start\\|size@endian sign) | r_power | r_inertial | r_speed | slope | raw range | unique |",
           "|---|---|---|---|---|---|---|---|---|---|---|"]
  for i, c in enumerate(cands, 1):
    corr = f"{c.r_power:+.3f} | {c.r_inertial:+.3f} | {c.r_speed:+.3f}"
    where = f"{c.bus} | 0x{c.address:X} | {known.get(c.address, '')} | `{c.spec.dbc}`"
    lines.append(f"| {i} | {where} | {corr} | {c.slope:.4g} | {c.raw_min}..{c.raw_max} | {c.unique} |")
  lines.append("")
  lines.append("## Pack-voltage-like fields (nearly constant, 200-1000 V after a plausible scale)\n")
  lines.append("| bus | addr | dbc | field | scale | median V | CV | unique |")
  lines.append("|---|---|---|---|---|---|---|---|")
  for v in volts[:30]:
    lines.append(f"| {v.bus} | 0x{v.address:X} | {known.get(v.address, '')} | `{v.spec.dbc}` | {v.scale} | {v.median_v:.1f} | {v.cv:.3f} | {v.unique} |")
  lines.append("")
  return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route")
  ap.add_argument("--bus", type=int, action="append", help="restrict to this bus (repeatable)")
  ap.add_argument("--top", type=int, default=40)
  ap.add_argument("--min-rate", type=float, default=2.0, help="ignore messages slower than this (Hz)")
  ap.add_argument("--mass", type=float, help="vehicle mass in kg (defaults to carParams.mass)")
  ap.add_argument("--cda", type=float, default=0.75, help="drag area Cd*A in m^2")
  ap.add_argument("--crr", type=float, default=0.010, help="rolling resistance coefficient")
  ap.add_argument("--keep-friction-braking", action="store_true", help="don't mask samples with brakePressed")
  ap.add_argument("--out", help="write markdown here instead of stdout")
  ap.add_argument("--csv", help="also dump all ranked candidates to this CSV")
  args = ap.parse_args(argv)

  cap = load_capture(args.route, buses=args.bus)
  ref = build_reference(cap, crr=args.crr, cda=args.cda, mass_kg=args.mass, mask_friction_brake=not args.keep_friction_braking)
  cands = scan_capture(cap, ref, buses=args.bus, min_rate_hz=args.min_rate, top=args.top)
  volts = find_voltage_candidates(cap, buses=args.bus)
  known = dbc_lookup(cap.car_fingerprint)

  text = render(cands, volts, known, int(ref.valid.sum()))
  if args.out:
    with open(args.out, "w") as f:
      f.write(text)
    print(f"wrote {args.out}", file=sys.stderr)
  else:
    print(text)

  if args.csv:
    with open(args.csv, "w", newline="") as f:
      w = csv.writer(f)
      w.writerow(["bus", "address", "start_bit", "size", "little_endian", "signed", "r_power", "r_inertial", "r_speed",
                  "slope", "raw_min", "raw_max", "unique", "n"])
      for c in cands:
        w.writerow([c.bus, c.address, c.spec.start_bit, c.spec.size, c.spec.little_endian, c.spec.signed,
                    f"{c.r_power:.4f}", f"{c.r_inertial:.4f}", f"{c.r_speed:.4f}", f"{c.slope:.6g}", c.raw_min, c.raw_max, c.unique, c.n])
  return 0


if __name__ == "__main__":
  sys.exit(main())
