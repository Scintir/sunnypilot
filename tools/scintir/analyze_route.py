#!/usr/bin/env python3
"""Scintir off-device route analyzer.

Reads every rlog.zst in a route directory, produces:

  report.md   -- human-readable summary of the drive
  report.json -- machine-readable inventory + event timelines

Intended to be run on an operator's workstation against routes that the
Comma 4's Scintir rsync uploader has pushed to the user's server. Depends on
openpilot.tools.lib.logreader, so run it from a sunnypilot / openpilot
checkout:

  cd /path/to/sunnypilot
  python3 tools/scintir/analyze_route.py /path/to/route_dir
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader  # type: ignore


def _format_ts(t: float) -> str:
  minutes = int(t // 60)
  seconds = t - minutes * 60
  return f"{minutes:02d}:{seconds:05.2f}"


@dataclass
class MessageStats:
  frame_count: int = 0
  first_ts: float = float("inf")
  last_ts: float = 0.0
  byte_seen: list = field(default_factory=lambda: [set() for _ in range(8)])

  def update(self, ts: float, data: bytes) -> None:
    self.frame_count += 1
    if ts < self.first_ts:
      self.first_ts = ts
    if ts > self.last_ts:
      self.last_ts = ts
    for i, byte_val in enumerate(data[:8]):
      self.byte_seen[i].add(byte_val)

  def as_dict(self) -> dict:
    duration = max(0.001, self.last_ts - self.first_ts)
    return {
      "frame_count": self.frame_count,
      "duration_s": round(self.last_ts - self.first_ts, 3) if self.last_ts else 0.0,
      "frequency_hz": round(self.frame_count / duration, 2),
      "byte_cardinality": [len(s) for s in self.byte_seen],
    }


@dataclass
class TimelineEntry:
  t: float
  value: object


def analyze_route(route_dir: Path) -> dict:
  messages: dict = defaultdict(MessageStats)
  ego_speed: list = []
  set_speed: list = []
  limiter_active: list = []
  limiter_offset: list = []
  soc: list = []
  current: list = []
  hcu1: list = []
  hcu5: list = []

  rlog_paths = sorted(route_dir.glob("*/rlog.zst"))
  if not rlog_paths:
    rlog_paths = sorted(route_dir.glob("rlog.zst"))
  if not rlog_paths:
    raise FileNotFoundError(f"no rlog.zst under {route_dir}")

  t0: float | None = None
  for rlog_path in rlog_paths:
    lr = LogReader(str(rlog_path))
    for msg in lr:
      ts = msg.logMonoTime * 1e-9
      if t0 is None:
        t0 = ts
      t = ts - t0
      typ = msg.which()
      if typ == "can":
        for can in msg.can:
          messages[(can.address, can.src)].update(t, bytes(can.dat))
      elif typ == "carState":
        cs = msg.carState
        ego_speed.append(TimelineEntry(t, float(cs.vEgo)))
        set_speed.append(TimelineEntry(t, float(cs.cruiseState.speed)))
      elif typ == "carStateSP":
        sp = msg.carStateSP
        limiter_active.append(TimelineEntry(t, bool(getattr(sp, "evLimiterActive", False))))
        limiter_offset.append(TimelineEntry(t, float(getattr(sp, "evLimiterSetSpeedOffset", 0.0))))
        soc.append(TimelineEntry(t, float(getattr(sp, "scintirBatterySoc", 0.0))))
        current.append(TimelineEntry(t, float(getattr(sp, "scintirBatteryCurrent", 0.0))))
        hcu1.append(TimelineEntry(t, int(getattr(sp, "scintirHcu1Status", 0))))
        hcu5.append(TimelineEntry(t, int(getattr(sp, "scintirHcu5Status", 0))))

  events = _build_limiter_events(limiter_active, limiter_offset)
  hcu1_transitions = _transitions(hcu1)
  hcu5_transitions = _transitions(hcu5)

  dbc_path = _find_dbc(route_dir)
  known_ids = _load_known_ids(dbc_path) if dbc_path else set()

  inventory = []
  for (addr, bus), stats in sorted(messages.items()):
    inventory.append({
      "address": addr,
      "hex": f"0x{addr:X}",
      "bus": bus,
      "known": addr in known_ids,
      **stats.as_dict(),
    })

  duration_s = 0.0
  for timeline in (limiter_active, ego_speed, soc):
    if timeline:
      duration_s = max(duration_s, timeline[-1].t)

  return {
    "route": str(route_dir),
    "segments": len(rlog_paths),
    "duration_s": round(duration_s, 2),
    "inventory": inventory,
    "limiter_events": events,
    "hcu1_transitions": hcu1_transitions,
    "hcu5_transitions": hcu5_transitions,
    "battery_soc_range": [min(e.value for e in soc), max(e.value for e in soc)] if soc else None,
    "battery_current_range": [min(e.value for e in current), max(e.value for e in current)] if current else None,
    "ego_speed_range": [min(e.value for e in ego_speed), max(e.value for e in ego_speed)] if ego_speed else None,
    "dbc_path": str(dbc_path) if dbc_path else None,
  }


def _build_limiter_events(active: list, offset: list) -> list:
  """Collapse per-frame limiter_active flag into start/end windows.

  Uses a running index into the offset timeline (no exact-timestamp join,
  which is fragile when active and offset streams are not sample-aligned).
  Also flushes a final event if the route ends while the limiter is still
  active.
  """
  events: list = []
  prev = False
  start_t = 0.0
  peak_offset = 0.0
  offset_idx = 0

  def _offset_at(t: float) -> float:
    nonlocal offset_idx
    while offset_idx + 1 < len(offset) and offset[offset_idx + 1].t <= t:
      offset_idx += 1
    return float(offset[offset_idx].value) if offset else 0.0

  for entry in active:
    cur_off = _offset_at(entry.t)
    if entry.value and not prev:
      start_t = entry.t
      peak_offset = cur_off
    elif entry.value and prev:
      if cur_off > peak_offset:
        peak_offset = cur_off
    elif not entry.value and prev:
      events.append({
        "start": _format_ts(start_t),
        "end": _format_ts(entry.t),
        "duration_s": round(entry.t - start_t, 2),
        "peak_offset": round(peak_offset, 1),
      })
    prev = entry.value

  # Flush tail if the route ended while still active
  if prev:
    end_t = active[-1].t if active else start_t
    events.append({
      "start": _format_ts(start_t),
      "end": _format_ts(end_t),
      "duration_s": round(end_t - start_t, 2),
      "peak_offset": round(peak_offset, 1),
      "truncated": True,
    })
  return events


def _transitions(timeline: list) -> list:
  out: list = []
  last = None
  for e in timeline:
    if last is None or e.value != last:
      out.append({"t": _format_ts(e.t), "value": e.value})
    last = e.value
  return out


def _find_dbc(route_dir: Path) -> Path | None:
  p = route_dir.resolve()
  for parent in [p, *p.parents]:
    candidate = parent / "opendbc_repo" / "opendbc" / "dbc" / "hyundai_kia_generic.dbc"
    if candidate.exists():
      return candidate
  return None


def _load_known_ids(dbc_path: Path) -> set:
  ids: set = set()
  with dbc_path.open() as f:
    for line in f:
      if line.startswith("BO_ "):
        parts = line.split()
        if len(parts) >= 2:
          try:
            ids.add(int(parts[1]))
          except ValueError:
            pass
  return ids


def write_reports(report: dict, route_dir: Path, write_md: bool = True) -> tuple[Path, Path | None]:
  json_path = route_dir / "report.json"
  with json_path.open("w") as f:
    json.dump(report, f, indent=2, default=str)

  if not write_md:
    return json_path, None

  lines: list = []
  lines.append(f"# Scintir route report: `{Path(report['route']).name}`")
  lines.append("")
  lines.append(f"- Segments: **{report['segments']}**")
  lines.append(f"- Duration: **{report['duration_s']} s**")
  if report.get("battery_soc_range"):
    lo, hi = report["battery_soc_range"]
    lines.append(f"- Battery SOC: **{lo:.0f}% → {hi:.0f}%**")
  if report.get("battery_current_range"):
    lo, hi = report["battery_current_range"]
    lines.append(f"- Battery current: **{lo:.1f} A → {hi:.1f} A** (+ discharge / - regen)")
  if report.get("ego_speed_range"):
    lo, hi = report["ego_speed_range"]
    lines.append(f"- Ego speed: **{lo:.1f} m/s → {hi:.1f} m/s**")
  lines.append(f"- DBC used: `{report['dbc_path'] or '(not found)'}`")
  lines.append("")

  lines.append("## Limiter activations")
  if report["limiter_events"]:
    for ev in report["limiter_events"]:
      lines.append(f"- **{ev['start']} → {ev['end']}** ({ev['duration_s']} s, peak offset {ev['peak_offset']})")
  else:
    lines.append("- _(none observed)_")
  lines.append("")

  lines.append("## HCU1_STS transitions")
  for t in report["hcu1_transitions"]:
    lines.append(f"- `{t['t']}`: {t['value']}")
  lines.append("")

  lines.append("## HCU5_STS transitions")
  for t in report["hcu5_transitions"]:
    lines.append(f"- `{t['t']}`: {t['value']}")
  lines.append("")

  unknowns = [m for m in report["inventory"] if not m["known"]]
  unknowns.sort(key=lambda m: -m["frame_count"])
  lines.append(f"## Top un-decoded messages ({len(unknowns)} total)")
  lines.append("| arbID | bus | freq (Hz) | frames | byte cardinalities |")
  lines.append("|---|---|---|---|---|")
  for m in unknowns[:30]:
    card = "/".join(str(c) for c in m["byte_cardinality"])
    lines.append(f"| {m['hex']} | {m['bus']} | {m['frequency_hz']} | {m['frame_count']} | {card} |")
  lines.append("")

  lines.append(f"## Inventory total")
  lines.append(f"- Unique arbID/bus combos: **{len(report['inventory'])}**")
  lines.append("")

  md_path = route_dir / "report.md"
  with md_path.open("w") as f:
    f.write("\n".join(lines))
  return json_path, md_path


def main() -> int:
  ap = argparse.ArgumentParser(description="Analyze a Scintir-uploaded route")
  ap.add_argument("route_dir", type=Path, help="Path to route directory")
  ap.add_argument("--json-only", action="store_true", help="Only write report.json")
  args = ap.parse_args()

  if not args.route_dir.exists():
    print(f"route dir not found: {args.route_dir}", file=sys.stderr)
    return 1

  report = analyze_route(args.route_dir)
  json_p, md_p = write_reports(report, args.route_dir, write_md=not args.json_only)
  print(f"wrote {json_p}")
  if md_p is not None:
    print(f"wrote {md_p}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
