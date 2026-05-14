# EV Limiter forensic scripts

Pinned from `/tmp/` per iter15 Phase 8 (avoid losing them on container restart).

Each script consumes one or more comma route segments (e.g. `00000085--6763c2caa0--*`)
and emits compact per-drive metrics relevant to the EV limiter / power estimator.

## Running

These scripts depend on the openpilot LogReader. Run from the repo root with:

```bash
PYTHONPATH=/home/alex.smith/git/sunnypilot/.venv/lib/python3.12/site-packages:/usr/lib/python3/dist-packages:/home/alex.smith/git/sunnypilot \
  /usr/bin/python3 tools/log_uploader/forensics/<script>.py <route_segment_dir> [args]
```

If your venv differs, adjust the leading `PYTHONPATH=` accordingly.

## Scripts

| Script | Purpose |
|---|---|
| `iter13_drive_forensics.py` | iter13 baseline — block reasons, SCC cancel suspicion |
| `iter14_drive_forensics.py` | iter14 v2 — state distribution, guard yield events, RECOVERY-while-capped runs (Bug #1 trace) |
| `iter14_verify_recovery_capped.py` | Print the first N frames where state==RECOVERY AND estPowerControlW>=cap (Bug #1 detection) |
| `iter14_estimator_validation.py` | Triple-output power estimator distributions (instant/control/HUD) per speed bucket |
| `iter14_power_ground_truth.py` | Compare estPowerInstantW peaks against real ICE-engagement threshold |
| `cruise_off_detail.py` | Per cruise-off event, lookback at prior-1s SET activity to attribute cancels |
| `standstill_telemetry_audit.py` | Verifies `_was_in_prelaunch` reset on standstill exit (iter14 carryover) |

## iter15 additions

iter15 adds new telemetry fields used by these scripts on next runs. None of the
existing scripts read these fields yet; extend in-place when post-deploy drive
data arrives:

- `evLimiterGuardForcedTransition` / `evLimiterGuardForcedTransitionEvents` (Section A)
- `evLimiterRecoveryYieldEpisodes` (Section A, edge-detected)
- `evLimiterGradePowerRawW` / `evLimiterGradePowerCappedFrames` (Section B)
- `evLimiterStandstillExitStateSnapshot` / `evLimiterStandstillExitTimeS` /
  `evLimiterStandstillExitToFirstResLatencyFrames` (Section C)
- `evLimiterLongStandstillResets` / `evLimiterLongStandstillPrelaunchBackoffCleared` /
  `evLimiterLongStandstillSoftcapReasonCleared` (Section C)
- `evLimiterPostResQuietActive` / `evLimiterSoftcapDecrementSuppressedFrames` /
  `evLimiterSoftcapDecrementSuppressedEvents` / `evLimiterPostResHardOverrideEvents` (Section D)
