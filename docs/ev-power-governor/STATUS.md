# EV power governor: status as of 2026-09-18

Handoff notes so work can continue in a fresh session. Vehicle: 2022 Hyundai Santa Fe PHEV
(`HYUNDAI_SANTA_FE_PHEV_2022`, Hyundai CAN, Hyundai L harness). Device: comma four on AGNOS 19.7.

## Where the code is

| repo | branch | HEAD | remote |
|---|---|---|---|
| sunnypilot | `ev-can-discovery` | `a5238c290b` | https://github.com/Scintir/sunnypilot (remote `scintir`) |
| opendbc (submodule) | `ev-bms-uds` | `eef850ed` | https://github.com/Scintir/opendbc (remote `scintir`) |

`.gitmodules` in the branch points opendbc at the Scintir fork. On the device the submodule's `origin` was
also pointed at that fork by hand (`git submodule sync` did not take on the first attempt).

Commits on `ev-can-discovery` beyond origin/master (`a5f44653d7`):
1. `120dcb2a7c` CAN discovery mode + Hyundai BMS UDS polling + offline tooling + docs
2. `9b032dc500` poller fix: unpack card's `(nanos, frames)` packet tuples
3. `a5238c290b` poller fix: frames are plain `(address, dat, src)` tuples, not `CanData`; test now round-trips
   through card's real `can_list_to_can_capnp` / `can_capnp_to_list`

## What was built

- `CanDiscoveryMode` param (int): 1 = passive ELM327 sniff, 2 = also pin bus 1 to the OBD-II port. Not used
  for the Santa Fe path; kept for other platforms.
- `EvBmsUdsPolling` param (bool): read-only UDS polling of the BMS at `0x7E4` on bus 1 (OBD-II port) while
  openpilot drives normally.
  - opendbc Hyundai safety allowlists `0x7E4` on bus 1 behind `HyundaiSafetyFlagsSP.BMS_UDS` (bit 16), limited
    to single-frame `0x22`/`0x21` reads with 1-2 byte identifiers and `30 00 00` flow control. Full opendbc
    safety suite passed (7450 tests).
  - pandad re-selects the OBD mux (panda cmd `0xdb`) after setting the car safety model. No panda firmware change.
  - card runs `BmsUdsPoller` (`openpilot/sunnypilot/selfdrive/car/hyundai/bms_uds.py`): `22 0101` at 10 Hz,
    ISO-TP reassembly, fallback to `21 01` after 3 failures. Publishes `evBatteryStateSP` (custom.capnp slot 10,
    log.capnp @136, services.py 10 Hz) with decoded fields + raw payload.
  - Decode follows the community HKMC BMS `0x0101` layout (JejuSoul/OBD-PIDs-for-HKMC-EVs). NOT yet confirmed
    on the Santa Fe PHEV; `decodeValid` is a plausibility flag only.
- Offline tooling in `openpilot/sunnypilot/tools/can_discovery/`: `inventory.py`, `power_scan.py`,
  `uds_extract.py` (dumps BMS responses from a route to CSV with the tentative decode).

## What happened on the device (chronology)

- First branch install bricked boot: the dev branch expects AGNOS 19.7, the release had 18.4, the hidden OS
  update was interrupted by a power cycle. Recovered via tap-to-factory-reset. Lesson: never power cycle during
  the first boot of a branch with a different AGNOS; run the AGNOS update manually over SSH first if in doubt
  (`agnos.py --swap`).
- Reinstalled `install.sunnypilot.ai/master` (AGNOS 19.7), then switched to the fork over SSH. Build fine.
- Drive 1 and drive 2 both showed "Unknown Vehicle Variant": card crashed on its first tick in the poller's
  receive path (wrong assumptions about the CAN packet shape). Both fixed; see commits 2 and 3. Fingerprinting
  itself worked (source `fixed`, 14 FW versions, 7 ECUs answered, VIN read), so harness + OBD cable are fine.
- `CarPlatformBundle` is pinned to the Santa Fe PHEV on the device (set via sunnylink; it synced once the
  device was back on Wi-Fi). Not strictly needed since FW fingerprinting works.
- Device params: `EvBmsUdsPolling=True`, `CanDiscoveryMode` unset/0.

## Drive 3 (2026-09-18, route 00000007--24310e2c0a, 26 segments)

- card did not crash. Fingerprint OK, radar tracks enabled, pandad logged "routing bus 1 to the OBD-II port".
- Poller ran: 4744 requests, 0 responses, 4743 timeouts, 0 negative responses. Only 5 TX echoes (src 129) on
  bus 1 right after the safety switch, none after; no safety rejects (src 193); safetyTxBlocked stayed at 2.
  Bus 1 RX counter frozen at 39900 (the fingerprint-time count) for the entire drive.
- Diagnosis: the `0xdb` OBD mux command re-routes pins without re-initializing the FDCAN core; the firmware's
  ELM327 path does `can_init_all` after the mux. Bus 1 came up dead. Fix: pandad now calls
  `set_can_speed_kbps(1, 500)` right after `set_obd(true)` (triggers `can_init` for bus 1). Commit on
  `ev-can-discovery` after a5238c290b. Requires a pandad rebuild on the device (C++), so the next boot
  rebuilds.
- The alert Alex saw was `commIssue` ("Communication Issue between Processes"), present in every segment, with
  empty not_alive / not_freq_ok lists, i.e. some service was marked *invalid*. Which one is not yet known; the
  `invalid` list from the commIssue log events is the next thing to read. openpilot longitudinal is OFF in
  carParams, so radar errors were never evaluated.

- Root cause of the commIssue alert (found 2026-09-18): the Hyundai radar track parser reads **bus 1**. Re-routing
  bus 1 to the OBD-II port for BMS polling removes the tracks; the radar interface then never publishes again
  (last radarTracks at 12.3 s == the moment the safety model + OBD mux were set). Fix in opendbc `ev-bms-uds`
  (after eef850ed): with stock longitudinal, fall back to empty RadarData at 20 Hz after 1 s without tracks.
  Consequence: no radar-derived lead data while EvBmsUdsPolling is on and openpilot longitudinal is off.
  Bus 1 cannot be both the harness CAN2 pair and the OBD-II port; if radar tracks matter, the BMS has to be
  reached another way (bus 0 answered UDS from 7 ECUs during fingerprinting; the BMS was not among them).

## Drive 4 (2026-09-18, route 00000008--4902dfe79f) and the bus change

- Drive 4 ran the OLD pandad binary (scons cache; device clock was wrong at first build) and the old opendbc, so
  it tested nothing new. Same result: bus 1 error-passive, REC 127, tx frozen, 0 responses.
- Later analysis: the ELM327/OBD fingerprint phase never produced any UDS reply on bus 1 either (all 7 ECU
  responses were on bus 0). The OBD-II CAN path on this install looks dead electrically; the bus 1 traffic seen
  is the harness CAN2 pair in NORMAL mode (radar tracks etc.).
- Change: `EvBmsUdsBus` param (int, default 0). Default polls the BMS on **bus 0 (C-CAN)**: safety allowlists
  0x7E4 on bus 0 too (opendbc 47ed2a20 -> next commit), pandad only switches the OBD mux when the param is 1,
  radar tracks stay intact. Whether the BMS answers 0x7E4 on C-CAN is unknown; the FW query got no 0x7EC reply
  there, but it asked different DIDs.
- Device gotcha: `git submodule update` did not move opendbc; pin it by hand (`cd opendbc_repo && git fetch
  origin ev-bms-uds && git checkout <sha>`). pandad rebuilds at boot only if scons decides so; force with
  `rm -f openpilot/selfdrive/pandad/panda_safety.o openpilot/selfdrive/pandad/pandad && scons openpilot/selfdrive/pandad`.

## Open items, in order

1. (done 2026-09-18, drive 3 happened) Verify the install on the bench before driving. The bench script in the last chat turn had a
   bug of its own (called `tx()` twice inside the request interval, then indexed an empty list). Corrected
   script: fresh poller for the send-path check. Expected output `card hook paths OK on device`.
2. Clear `CarParamsCache` / `CarParamsSPCache`, reboot, plug in, drive 15-20 min with firm accelerations and
   lift-off regen.
3. Pull segments from `/data/media/0/realdata/` to the Mac, then to `/home/alex.smith/<dir>` on ctc-gitrunner so
   the container can read them. Run `uds_extract.py` and compare `soc_pct` / `pack_v` to the cluster.
4. If negative responses for both `0x22` and `0x21`: the gateway isn't answering; inspect raw bus 1 frames.
5. If decode is off: refit byte offsets from the CSV.
6. Phase 2 (not started): feed `availableDischargePower` into `get_cruise_accel` in
   `openpilot/selfdrive/controls/lib/longitudinal_planner.py`, backstop in `get_pid_accel_limits`. Goal to
   confirm with Alex: keep demand under the BMS discharge limit so the PHEV stays in EV mode.

## Lessons for the next session

- Test card hooks against card's real serializers (`can_list_to_can_capnp` / `can_capnp_to_list`), never a
  hand-written fake. That is what the round-trip test now does.
- Run any "bench check" script locally first; two bad scripts cost trust.
- Device access: SSH `comma@<ip>` (192.168.2.2 on Mac Internet Sharing), key auth via GitHub user `Scintir`.
  Container pushes to GitHub over SSH with the forwarded ctc-gitrunner key.
- The comma installer only clones sunnypilot's own repo; forks are installed by SSH checkout after a stock install.
