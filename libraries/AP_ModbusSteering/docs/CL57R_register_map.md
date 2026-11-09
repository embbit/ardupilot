# CL57R Modbus register map (steering subset)

Source: CNC Technology CL57R NEMA23 Modbus manual (Feb 2026).
Full text extract: `CL57R_Modbus_NEMA23.txt`. PDF is local-only (too large for git hooks).

## Protocol

- Modbus RTU over RS-485
- FC03 read holding, FC06 write single, FC16 write multiple
- Half-duplex: one outstanding request; wait echo/CRC before next TX

## Status / feedback (RO)

| Addr | Name | Notes |
|------|------|-------|
| 0x0003 | Status word | Bit0 in-pos, Bit1 home-done, Bit2 running, Bit3 alarm, Bit4 enabled |
| 0x0005 | DI status | Bit1=X1 (default P-OT), Bit2=X2 (default N-OT) |
| 0x0007/08 | Encoder abs | int32, high then low |

## Motion parameters (RW)

| Addr | Name | Range / notes |
|------|------|---------------|
| 0x0030 | Start speed | 2..300 RPM |
| 0x0031 | Accel time | 1..2000 ms |
| 0x0032 | Decel time | 1..2000 ms |
| 0x0033 | Max speed | -3000..3000 RPM (sign = direction in speed mode) |
| 0x0034/35 | Target position | int32 pulses, high then low |
| 0x0036 | Motion control | Bit0 pos start, Bit1 abs(1)/rel(0), Bit2 irq, Bit3 speed, Bit4 home, Bit5 stop, Bit6 estop |
| 0x0037 | Aux control | 4=alarm clear, 8=pos zero |
| 0x0038 | Motor enable | 0=off, 1=on |
| 0x003A | Abs/rel mode | 0=relative, 1=absolute |

## Homing (RW/S)

| Addr | Name | Notes |
|------|------|-------|
| 0x0040 | Home method | 17=N-OT approach, 18=P-OT approach |
| 0x0041 | Home seek RPM | 5..3000 |
| 0x0042 | Home crawl RPM | 5..300 |
| 0x0043 | Home accel | 1..2000 ms |

## Vendor write sequence (positioning)

1. Enable `0x0038 = 1`
2. Write start/accel/decel/maxspd and target `0x0034/35` (FC16 OK)
3. Separate frame: `0x0036 = 0x0001` (rel) or `0x0003` (abs start)
4. Stop: `0x0036 = 0x0020`; estop: `0x0040`

Target and motion word stay as **two frames** (parameter, then command).
