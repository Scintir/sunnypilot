# Drive 2026-09-24 forensics — comm faults and power-estimate bias

Route `00000015--99c9d4c0d0` (33 segments, 16:26 MDT), build `staging-mici-2026.09` @ 824e63acef, device mici (comma 4).
Params: EVLimiterPowerThresholdKW=40, EvLimiterMotorCapKW=60, EvLimiterAssumeEvOnly=1, MaxDeficit=7, MaxGap=5.

## 1. Communication faults — hardwared freezes; cause not yet named

Three faults, all raised by selfdrived's freshness check on `deviceState` (the `gpsLocationExternal` entry in the first pass was a red herring: it never gapped; only the always-ignored `alertDebug` / `lateralManeuverPlan` share the list):

| t (route) | alert | deviceState gap |
|---|---|---|
| 10:44 | Low Communication Rate | 3.0 s |
| 16:26 | Low Communication Rate | 1.7 s + 2.0 s |
| 29:34 | Communication Issue | 8.0 s → forced disengage at 29:36.8 |

Tick spacing over the drive (3984 ticks): p50 0.503 s, p99 0.512 s, p99.9 2.02 s, max 8.04 s. Only 6 ticks exceeded 0.7 s. The loop is not generally jittery; it freezes outright a few times per drive.

What the rlog rules out:

- **Not disk.** System iowait stays at 0.0–0.3 % through every stall (6.6 % once, *after* the 16:26 stall), no process is ever in D state, and loggerd/encoderd keep their normal CPU share. Segment rotations do not line up with the stalls.
- **Not CPU starvation.** hardwared normally uses ~0.10 s CPU per 2 s procLog window. Across the 8 s stall it used **0.00 s in three consecutive windows**, i.e. the whole process (main loop, hw_state thread, touch thread) was parked, not runnable. Other cores were idle.
- **Not the panda / CAN.** pandaStates cadence, faultStatus, heartbeat, spiErrorCount and voltage are flat across every stall. The one SPI NACK burst (16:26.6–16:28.7, 64 NACKs) starts *after* hardwared resumes.
- **Not the network path.** networkType was `none` for the whole drive, so the wifi (`wpa_supplicant`, `sudo cat` of NM keyfiles) and modem branches in hardwared never ran.
- **Not the params writes.** `put(block=False)` hands the write to a C++ background thread (it did on the old base too, under the name `put_bool_nonblocking`; upstream #38016 only renamed it). The "only write on change" edit below is a tidy-up, not the fix.
- **Not a missing thermal zone.** All zones resolve; `exhaust` reads a constant −40 °C (open thermistor) and `pm8005_tz` a constant 37 °C, but those are values, not errors, and the same zone list was read on the old base.
- Nothing in the kernel/journal at the stall times. The journal does show an AGNOS-side `power_monitor.service` logging every 60 s and `udevadm` re-parsing rules a few times; neither aligns with the stalls.

What is left: a single syscall in one hardwared thread blocking for seconds **while holding the GIL** (that is what freezes the other two threads too). Candidates the old base did not touch per tick are the new USB/chestnut sysfs reads (`get_usb_topology` every 0.5 s, `get_usb_state` every 10 s, `typec_cc_orientation` on the charger PSY) and the hwmon/bms power reads; the OS update changed the drivers behind all of them. Per Alex this did not happen on the previous OS + 2026-01-000 base.

**Instrumentation added (uncommitted) to name it on the next drive**, `openpilot/system/hardware/hardwared.py` → `StallWatchdog`:

- per-phase wall time inside the tick (thermal zones, statvfs/proc/gpu, screen brightness, usb/chestnut, params startup conditions, build metadata, kmsg, hwmon/bms, send, statlog, status packet);
- `faulthandler.dump_traceback_later(1.5 s)` re-armed every tick. faulthandler's watchdog is a C thread that does not need the GIL, so 1.5 s into a freeze it writes **every thread's Python stack** to `/dev/shm/hardwared_stall_stacks.txt`; the next tick ships that text to cloudlog as `hardwared slow tick` (`period`, `slowest_phases`, `stacks`). Verified off-device against a stall that holds the GIL (ctypes `PyDLL` sleep).

After the next drive: `grep "hardwared slow tick"` in the rlog (`tools/log_uploader/forensics/hardwared_stall_scan.py` prints them with the gap table). The `stacks` field names the file/line each thread was blocked on.

If it needs to be caught live instead: `py-spy dump --pid $(pgrep -f hardwared)` or `strace -f -T -p $(pgrep -f hardwared) 2>&1 | awk -F'<' '$NF+0 > 0.5'` on the device during a drive.

**Limiter itself:** cleared. canErrorCounter moved once at boot; carState/can/sendcan/modelV2 kept cadence throughout. Alex reports never entering ICE activation on this drive despite many throttle-ins, i.e. the limiter held power demand under the engine-start threshold for the whole route.

## 2. Power estimate — what the telemetry shows

No real-power decode was available (`evLimiterRealPowerSource` = 0 on all 200 k frames); everything below is model-based.

Estimator = `m·v·max(0, aBasis) + clamp(m·v·max(0, grade−0.2), 10 kW) + road_load(v)`, then a fast-rise (150 ms) / slow-fall (2 s) LP for the HUD.

### 2a. High speed / grades over-read: aBasis already contains the grade term, and it is jittery

Steady cruise > 60 mph with |aEgo| < 0.05 m/s², bucketed by Kalman grade accel:

| grade (m/s²) | aBasis p50 | inst kW | HUD kW |
|---|---|---|---|
| −0.4 | −0.44 | 20.7 | 21.6 |
| −0.2 | −0.19 | 21.1 | 25.4 |
| 0.0 | +0.01 | 22.4 | 30.1 |
| +0.2 | +0.21 | 34.3 | 44.8 |
| +0.3 | +0.22 | 39.5 | 46.9 |
| +0.4 | +0.33 | 30.4 | 38.0 |

- **aBasis tracks grade 1:1** at constant speed: TCS13.aBasis is the *net* accel demand the powertrain must deliver, gravity included. Adding a separate grade term on top double-counts uphill (partially masked by the 0.2 m/s² deadband and the 10 kW clamp).
- **The saturation detector misfires on real grades.** At +0.4 m/s² aBasis (0.33) diverges from aEgo (0) by > 0.3, so after 0.5 s the code substitutes filtered aEgo (≈0) and the reading *drops* to road load + 10 kW (30 kW) on the steepest climbs. So: over-read on moderate grades, under-read on steep ones.
- **aBasis jitter is rectified by the asymmetric filter.** At > 60 mph the frame-to-frame |ΔaBasis| p90 is 0.28 m/s². At 70 mph the term is 61 kW per m/s², so that is ±17 kW of noise per frame. The fast-rise/slow-fall HUD filter rides the peaks: on flat highway (grade bucket 0.0) inst p50 is 22 kW but HUD p50 is 30 kW, and the worst frames (10:35.7, 71 mph, aEgo 0.00) show inst swinging 74→46→36→57 kW within 100 ms with the HUD pinned at the 60 kW cap.
- Road-load constants (Crr 0.011, CdA 0.75 → 22 kW at 70 mph) are not the over-read source; if anything they are slightly low for this vehicle.

### 2b. Low speed under-read: wheel power only, no efficiency or aux

- Hard launches at 9 mph, aEgo 2.7 m/s²: inst/HUD ≈ 20–22 kW. That is wheel power (F·v is small at low v). Battery power is wheel power / η plus accessories; at low speed and high torque η is ~0.7–0.8, and HVAC + DC-DC is 2–4 kW. Expected battery draw ≈ 30 kW.
- 0–10 mph bucket HUD p50 is 1.7 kW; a PHEV creeping with climate on draws 3–5 kW.
- HUD p50 at 10–20 mph is 6–9 kW against road load of 1.6–3 kW — the accel term dominates and is fine, the constant offset is what is missing.

### 2c. Recommended estimator change (iter17 candidate)

1. Drop aBasis from the power term. Use `m·v·max(0, aEgo_f + grade_kalman)` with a ~0.2 s symmetric LP on aEgo. Measured accel plus calibrated pitch is what the motor is actually delivering; this removes the double count, the jitter, and the need for the saturation hack, the grade deadband, and the grade cap. Keep aBasis only as a leading-edge indicator for the state machine if wanted.
2. Divide wheel power by an efficiency map η(v): 0.72 at 0 mph → 0.90 at ≥ 45 mph.
3. Add an aux baseline (start 2.5 kW; make it a param so it can be tuned against the cluster gauge).
4. Make the HUD filter symmetric (~0.5 s). Keep the fast control filter for the state arbiter.

Replay of that estimator on this drive (no ground truth, but direction matches the complaint):

| mph | current HUD p50/p90/p99 | alt p50/p90/p99 |
|---|---|---|
| 10 | 6.1 / 23.6 / 31.1 | 6.7 / 29.7 / 39.4 |
| 20 | 8.6 / 31.1 / 38.1 | 8.3 / 37.5 / 46.7 |
| 40 | 14.5 / 28.7 / 45.0 | 14.7 / 30.1 / 50.3 |
| 70 | 31.1 / 48.2 / 58.7 | 29.9 / 47.9 / 53.0 |

Low-speed p90 rises ~6 kW; highway p99 falls ~6 kW and the phantom peaks disappear. To validate, log the cluster kW gauge (photo or dash cam) at a few steady highway points and a few launches on the next drive.

Scripts: `tools/log_uploader/forensics/comm_fault_scan.py`, `estimator_decomposition.py`, `iter15_drive_forensics.py`.

## 3. Fixes applied (2026-09-24, uncommitted in the working tree)

### Comm faults — `openpilot/system/hardware/hardwared.py`
- `StallWatchdog` (per-phase timing + GIL-independent faulthandler stack dump on any >1.5 s freeze, shipped to cloudlog on the next tick). See Section 1 for what it answers.
- `GithubRunnerSufficientVoltage` and `NetworkMetered` are written only when their value changes. Tidy-up only; the nonblocking writer was never the stall.

### Power estimate — `opendbc/sunnypilot/car/hyundai/carstate_ext.py` (iter17)
- Power term is `m·v·max(0, aEgo_f + grade)` with aEgo LP'd at 0.2 s and grade from the Kalman pitch (or LONG_ACCEL fallback). aBasis no longer enters the estimate; it is still published as accelDemand for the SCC-decel detector.
- Battery power = wheel power / η(v) + aux. η ramps 0.72 → 0.90 over 0 → 45 mph. Aux defaults to 2.5 kW and is tunable via the new `EvLimiterAuxPowerW` param (card.py plumbs it; falls back to the default if the prebuilt params lib does not know the key yet — rebuild on device to enable it).
- Retired: grade deadband, grade contribution cap (`EvLimiterGradeContribCapKW` no longer read), aBasis/aEgo saturation substitution. `estPowerSaturated` is always false; `evLimiterGradePowerCappedFrames` stays 0; `evLimiterGradePowerRawW` still publishes.
- HUD filter symmetric 0.5 s (was 150 ms rise / 2 s fall). Control filter unchanged.
- Tests: `test_iter15_grade_clamp.py` and the iter9 deadband class removed; `test_iter17_estimator.py` added (efficiency map, aBasis independence, uncapped grade, downhill cancel, launch/creep baselines, aux clamp, LONG_ACCEL fallback, filter symmetry). Suite: 265 passed, 2 skipped.

Replay of the implemented iter17 code over this drive against the logged HUD:

| mph | logged HUD p50/p90/p99 | iter17 HUD p50/p90/p99 |
|---|---|---|
| 10 | 6.1 / 23.6 / 31.1 | 6.4 / 29.7 / 39.4 |
| 20 | 8.6 / 31.1 / 38.1 | 7.6 / 37.3 / 46.6 |
| 40 | 14.5 / 28.7 / 45.0 | 14.1 / 29.7 / 49.8 |
| 70 | 31.1 / 48.2 / 58.7 | 25.1 / 46.2 / 51.3 |

Flat 70 mph now reads ~25 kW (physically plausible for this vehicle); launches read ~30 % higher. Validate against the cluster gauge on the next drive, then tune `EvLimiterAuxPowerW` and the η endpoints.

**Threshold note:** the 40 kW `EVLimiterPowerThresholdKW` was tuned against the old, biased estimate. With the aux + efficiency terms, expect the limiter to trip a little earlier on launches and later on flat highway. Re-check the threshold after one drive.
