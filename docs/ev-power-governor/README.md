# EV power governor: phase 1, CAN discovery

Goal: keep sunnypilot's longitudinal control inside the high-voltage system's real power limits instead of
inferring them. That needs the car to tell us pack voltage, current, power, or ideally the BMS's allowed
discharge/regen power. Phase 1 captures every CAN frame the comma 4 can see and finds those signals.

## How CAN reaches the logs today

- The panda firmware forwards every received frame on all three buses to the host. There are no acceptance
  filters and the safety mode cannot drop RX frames; safety only gates TX, the relay, and the bus-1 mux
  (`panda/board/stm32h7/llfdcan.h`, `panda/board/drivers/fdcan.h`).
- pandad publishes every frame on the `can` service and loggerd writes every `can` event to the rlog.
  Only the qlog is decimated. No param can turn this off on a real car.
- Bus map on the comma 4 (cuatro): bus 0 is the vehicle side of the harness, bus 2 is the camera side, bus 1
  is either the harness's CAN2 pair or the OBD-II port depending on the mux. Bus 0/2 swap automatically with
  harness orientation so the numbers stay stable.
- Frames with `src >= 128` are echoes of frames we sent (`+128`) or frames the safety model rejected (`+192`).
  They are not vehicle traffic.
- CAN-FD is enabled on all buses. The firmware defaults are 500 kbit/s nominal and 2 Mbit/s data.

So the raw capture already exists in every rlog. What was missing is a way to keep the panda fully passive
for a whole drive, to point bus 1 at the OBD-II port during that drive, and tooling to sift the result.

## What this branch adds

1. **`CanDiscoveryMode` param** (int, persistent).
   - `0` off (default).
   - `1` passive sniff. pandad stays in ELM327 with the relay closed and bus 1 on the harness CAN2 pair.
     The car safety model is never set, so nothing is transmitted and openpilot cannot engage.
   - `2` passive sniff with bus 1 pinned to the OBD-II port for the whole drive. Needs the OBD-C cable
     (comma power) plugged into the car's OBD-II port.
   - selfdrived suppresses the `controlsMismatch` alert for ELM327 while the mode is active.
2. **Discovery tooling** in `openpilot/sunnypilot/tools/can_discovery/` (inventory and power-correlation scan).
   See its README.

## Procedure

1. Enable the mode on the device, then reboot so pandad picks it up on the next onroad transition:
   ```bash
   echo -n 2 > /data/params/d/CanDiscoveryMode   # or 1 for harness CAN2 on bus 1
   ```
2. Drive manually for 20 to 40 minutes. openpilot will not engage in this mode. Make the power demand easy to
   correlate: several hard accelerations from a stop, long steady cruises, lift-off regen from highway speed,
   a few full stops, and a couple of minutes parked with HVAC on then off. Avoid friction braking during the
   regen samples where you safely can; the scan masks `brakePressed` samples by default.
3. Set the param back to `0` and reboot.
4. Pull the rlogs (`/data/media/0/realdata/<route>`) to a PC, or let them upload and use the route name.
5. Run the inventory first and check the bus-health table: `rx lost` and `bus-off cnt` should be zero and FD
   should be `yes` on FD buses. If bus 1 has few or no addresses in mode 2, the OBD gateway is not
   broadcasting and UDS polling is the next step (phase 2 below).
6. Run the power scan. Confirm the top hits in cabana, then add them to the platform's DBC in opendbc.

## What is already known per brand (from opendbc)

- Ford: `BattTrac2_Pw_LimDchrg` / `BattTrac2_Pw_LimChrg` in watts plus instantaneous power. Directly usable.
- VW MEB: `MO_HVEM_MaxLeistung` (max drive power, W) and `MEB_HVEM_01.Engine_Power` (kW). Directly usable.
- GM: `HVBatteryVoltage` and `HVBatteryCurrent` in the powertrain DBC, so P = V*I. No limit signal.
- Rivian: front/rear max/min torque envelopes. A torque bound, not a power bound.
- Hyundai/Kia/Genesis (CAN and CAN-FD), Tesla, Toyota, Honda: nothing for the traction pack. HKG BMS data
  is available over UDS on the OBD port (`0x7E4`, DID `0x0101`) but not broadcast, so phase 2 there means
  polling rather than sniffing.

## BMS UDS polling (Hyundai CAN, added for the 2022 Santa Fe PHEV)

Alex's earlier full-bus scan of the Santa Fe PHEV found no broadcast HV messages, so the pack data has to be
polled from the BMS over UDS on the OBD-II port. This works while openpilot drives normally.

- **`EvBmsUdsPolling` param** (bool, persistent). When set on a Hyundai CAN platform (not CAN-FD), card sets
  `HyundaiFlagsSP.BMS_UDS_POLLING` and the `HyundaiSafetyFlagsSP.BMS_UDS` safety bit in `CarParamsSP`.
- **Safety (opendbc)**: the Hyundai safety model allowlists `0x7E4` on bus 1, but only ISO-TP single frames
  carrying `0x22` or `0x21` read requests with a 1 or 2 byte identifier, plus `30 00 00` flow control frames.
  Session control, writes, security access, tester present, and anything on bus 0 or 2 stay blocked. Covered by
  `test_bms_uds_tx` in `opendbc/safety/tests/test_hyundai.py`.
- **pandad**: after it sets the car safety model it re-selects the OBD mux (panda command `0xdb`) when the
  safety bit is set, so bus 1 stays on the OBD-II port for the whole drive. No panda firmware change is needed.
- **Poller** (`openpilot/sunnypilot/selfdrive/car/hyundai/bms_uds.py`, driven by card at 100 Hz): requests
  `22 0101` at 10 Hz, answers the first frame with flow control, reassembles the multi-frame response, and
  falls back to `21 01` after three consecutive negative responses or timeouts (older HKG BMS firmware).
- **Logging**: every request and response frame is already in the rlog on bus 1 (TX echoes show as bus 129).
  The decoded state is published and logged as `evBatteryStateSP` at 10 Hz with the raw payload attached.
- **Decode**: uses the community HKMC BMS 0x0101 table (SOC, charge/discharge power limits, pack current and
  voltage, temps, cell min/max, 12 V, motor rpm). The Santa Fe PHEV layout is not yet confirmed, so
  `decodeValid` only goes true when the values pass plausibility checks. Verify with
  `python -m openpilot.sunnypilot.tools.can_discovery.uds_extract <route> --csv bms.csv` and compare SOC and
  pack voltage against the cluster before trusting it.

Setup on the device:

```bash
echo -n 1 > /data/params/d/EvBmsUdsPolling   # requires the OBD-C cable in the car's OBD-II port
```

Then reboot and drive. `CanDiscoveryMode` must be `0` for this, since polling needs the car safety model.

## Phase 2 plan (not started)

- Carry the new signals in `CarStateSP` (`openpilot/cereal/custom.capnp` and `opendbc/car/structs.py`), parsed in
  a `carstate_ext.py` for the brand.
- Apply the limit in `get_cruise_accel` in `openpilot/selfdrive/controls/lib/longitudinal_planner.py` as
  `a_max = min(a_max, P_limit / (m * v))`, and mirror it in `get_pid_accel_limits` as a hard backstop.
- The UDS poller above already provides `availableDischargePower` and `availableChargePower` for Hyundai CAN
  platforms; once the decode is confirmed, feed those into the planner clamp.
