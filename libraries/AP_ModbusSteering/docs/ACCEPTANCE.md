# CL57R steering acceptance checklist

## Firmware

- Build: `./waf rover` (board DAKEFPVH743 or SITL)
- Tag in GCS: `CL57R: using SERIALn v14.4-run`

## Bring-up sequence

1. SERIAL* protocol = 101 (ModbusSteer), 115200
2. Power CL57R + motor; wait for `CL57R: init OK`
3. Disarm. Start cal: `OB_STR_CAL_TRIG=1` or `CAL_CH` rising edge
4. Expect: `cal start` → `seek 1 M17/M18` → `seek 2` → `mid target` → `calibrated at mid`
5. Arming must pass: calibrated, not homing, no alarm, link_ok
6. MANUAL: stick / QGC MANUAL_CONTROL moves rudder; named float `STR_IN` tracks input
7. STEERING/AUTO: motors steering (±4500) maps to absolute target
8. Pull RS485: `Modbus link lost, STOP`; restore: `link restored`

## Failure paths

- OT / stall during seek: backoff then reseek; after 2 tries → abort → idle
- `CAL_TRIG=2` / `RST_CH`: alarm clear (disarmed)
- Cal while armed: ignored

## SITL

```sh
./waf configure --board sitl && ./waf rover
python3 outb_steer_sim.py &
Tools/scripts/sitl_modbus_steering_test.py
```
