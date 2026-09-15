# CAN discovery tooling

Finds undocumented high-voltage signals (pack voltage, current, power, power limits) in a route's rlogs.
Companion to the `CanDiscoveryMode` param; see `docs/ev-power-governor/README.md` for the full procedure.

## What it does

- `inventory.py` lists every message on every bus with rate, length, CAN-FD flag, and the payload bytes that
  change, and marks which addresses the platform's DBC already covers. It also prints the panda's per-bus
  RX counters so a bad capture (lost frames, bus-off, wrong bit rate) is obvious before any analysis.
- `power_scan.py` enumerates every nibble-aligned 8/12/16-bit little- and big-endian field in every message,
  drops counters and checksums, resamples onto a 20 Hz grid, and ranks fields by Pearson correlation with a
  tractive-power estimate built from the car's own `vEgo`/`aEgo` and curb mass. It also lists fields that sit
  in the 200-1000 V band with low variance, which is what a pack voltage looks like.

- `uds_extract.py` pulls the BMS UDS responses (`0x7EC` on bus 1, from `EvBmsUdsPolling`) out of a route,
  reassembles the ISO-TP transfers, and writes the raw payload plus the tentative decode per response to CSV.

Field descriptors are printed in DBC syntax (`start|size@endian sign`) so a hit can be pasted into a DBC.

## Usage

```bash
# on a PC with the route pulled (or any LogReader identifier: route name, segment, local dir, URL)
python -m openpilot.sunnypilot.tools.can_discovery.inventory  "<dongle>|<route>" --out inventory.md
python -m openpilot.sunnypilot.tools.can_discovery.power_scan "<dongle>|<route>" --top 40 --out scan.md --csv scan.csv

# restrict to one bus, override the mass if carParams wasn't logged
python -m openpilot.sunnypilot.tools.can_discovery.power_scan /data/media/0/realdata/<route> --bus 1 --mass 2100
```

## Reading the results

- A real pack-power field has `r_power` above roughly 0.9 and a low `r_speed`. Its `slope` is counts per watt,
  so `1/slope` is the DBC factor if the field is in watts (50 for a 50 W/count field).
- A pack-current field looks the same but with `1/slope` near the pack voltage divided by the DBC factor.
- A field with high `r_speed` and `r_power` close to it is just vehicle speed. Ignore it.
- A negative `r_power` usually means the sign convention is charge-positive.
- Pack voltage sags a little under load, so it may show up in the power table with a small negative `r_power`
  and in the voltage table with a plausible scale.

Open the top addresses in cabana (`tools/cabana`) to confirm bit boundaries and scaling before adding them to
a DBC.
