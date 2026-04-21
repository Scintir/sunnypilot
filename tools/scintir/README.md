# Scintir off-device analysis tools

Run these on your workstation (**not** on the Comma 4) against routes that the
device's Scintir rsync uploader has pushed to your server. They consume
`rlog.zst` segments and produce a human-readable drive report plus a
machine-readable JSON inventory.

## Prerequisites

- A checkout of this repo with its Python deps installed (for
  `openpilot.tools.lib.logreader`).
- Passwordless SSH to the server hosting the uploaded routes.
- The environment variable `SCINTIR_REMOTE` pointing at the remote path the
  Comma 4 is rsyncing into — e.g. `user@host:/srv/scintir/`.

## Typical workflow after a drive

```bash
# 1. pull the latest route from the server
SCINTIR_REMOTE=user@host:/srv/scintir/ \
  ./tools/scintir/fetch_route.sh 2026-04-20--14-00-00--abc123 ~/scintir_routes

# 2. analyze it locally
python3 tools/scintir/analyze_route.py ~/scintir_routes/2026-04-20--14-00-00--abc123

# 3. read the report and hand it to Claude in this container, narrate the drive
#    ("merged onto highway around 12min, heard ICE kick in around 18min"),
#    and decide what to tune.
```

## What's in the report

- Route duration and battery SOC / current range for the drive
- Every CAN arbID seen on each bus, with frame count, frequency, and per-byte
  cardinality (how many distinct values appeared over the drive — useful for
  guessing which bytes are flags vs counters vs scaled quantities)
- Limiter activation windows — each time the limiter commanded a set-speed
  offset, the start/end times
- HCU1_STS and HCU5_STS transitions — candidate indicators of hybrid-state
  changes (ICE start/stop, EV-only, regen mode)
- List of un-decoded messages sorted by frame count — good starting points for
  reverse engineering

Known messages are identified by looking up the arbID in
`opendbc_repo/opendbc/dbc/hyundai_kia_generic.dbc`. The script searches up from
the route directory for that file, so running from the sunnypilot checkout
just works.
