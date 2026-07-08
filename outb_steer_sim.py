#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""UDP Modbus RTU simulator for CL57R + NEMA23 steering (SERIAL5 protocol 101)."""

import argparse
import socket
import time

HOST = "0.0.0.0"
MODBUS_UDP_PORT = 14555

CL57R_STEPS_PER_REV = 4000
DEFAULT_MAX_RPM = 120
DEFAULT_ACCEL_MS = 400
DEFAULT_DECEL_MS = 400

# NEMA23 + 25:1 gearbox + rudder load (reflected inertia at motor shaft)
NEMA23_INERTIA_FACTOR = 1.35
NEMA23_COAST_DECEL_PPS2 = 180.0


class Nema23Motor:
    """CL57R position-mode motor with accel/decel ramps and mechanical inertia."""

    def __init__(self):
        self.position = 0.0
        self.velocity = 0.0
        self.driver_target = 0
        self.max_rpm = DEFAULT_MAX_RPM
        self.accel_ms = DEFAULT_ACCEL_MS
        self.decel_ms = DEFAULT_DECEL_MS
        self.motor_enabled = False
        self.inertia_factor = NEMA23_INERTIA_FACTOR
        self.coast_decel = NEMA23_COAST_DECEL_PPS2
        self.status_word = 0
        self.error_code = 0
        self.track_err_limit = 8000
        self.encoder_lag_s = 0.0
        self.reported_position = 0.0

    @property
    def max_vel(self):
        return self.max_rpm * CL57R_STEPS_PER_REV / 60.0

    @property
    def max_accel(self):
        if self.accel_ms <= 0:
            return self.max_vel
        return self.max_vel / (self.accel_ms / 1000.0)

    @property
    def max_decel(self):
        if self.decel_ms <= 0:
            return self.max_vel
        return self.max_vel / (self.decel_ms / 1000.0)

    @property
    def actual_pos(self):
        return int(round(self.position))

    @property
    def encoder_pos(self):
        return int(round(self.reported_position))

    def set_driver_target(self, pos):
        self.driver_target = pos

    def _update_tracking_error(self):
        following = abs(self.driver_target - self.position)
        if self.motor_enabled and following > self.track_err_limit:
            self.error_code = 4
            self.status_word |= (1 << 3)
        else:
            self.error_code = 0
            self.status_word &= ~(1 << 3)

    def update(self, dt_s, link_active):
        if dt_s <= 0:
            return

        if not self.motor_enabled:
            self._decay_velocity(dt_s, self.max_decel * 2)
            self.position += self.velocity * dt_s
            self._update_tracking_error()
            lag = max(self.encoder_lag_s, 0.0)
            if lag > 0.0:
                alpha = min(dt_s / lag, 1.0)
                self.reported_position += (self.position - self.reported_position) * alpha
            else:
                self.reported_position = self.position
            return

        if link_active:
            self._track_target(dt_s)
        else:
            self._coast(dt_s)

        self.position += self.velocity * dt_s
        self._update_tracking_error()
        lag = max(self.encoder_lag_s, 0.0)
        if lag > 0.0:
            alpha = min(dt_s / lag, 1.0)
            self.reported_position += (self.position - self.reported_position) * alpha
        else:
            self.reported_position = self.position

    def _decay_velocity(self, dt_s, decel):
        if abs(self.velocity) < 0.5:
            self.velocity = 0.0
            return
        step = decel * dt_s
        if self.velocity > 0:
            self.velocity = max(0.0, self.velocity - step)
        else:
            self.velocity = min(0.0, self.velocity + step)

    def _coast(self, dt_s):
        """Link lost: NEMA23 rotor + gearbox coast with friction."""
        self._decay_velocity(dt_s, self.coast_decel)

    def _track_target(self, dt_s):
        error = self.driver_target - self.position
        if abs(error) < 0.5 and abs(self.velocity) < 1.0:
            self.velocity = 0.0
            self.position = float(self.driver_target)
            return

        direction = 1.0 if error > 0 else -1.0
        stop_dist = 0.5 * self.velocity * self.velocity / self.max_decel

        if direction * self.velocity < 0:
            # Reversing: decelerate first
            desired_accel = -direction * self.max_decel * self.inertia_factor
        elif abs(error) <= abs(stop_dist) + abs(self.velocity) * dt_s:
            # Brake for target
            desired_accel = -direction * self.max_decel
        else:
            # Accelerate toward target
            if abs(self.velocity) < self.max_vel:
                desired_accel = direction * self.max_accel
            else:
                desired_accel = 0.0

        self.velocity += desired_accel * dt_s
        self.velocity = max(-self.max_vel, min(self.max_vel, self.velocity))

        # Inertia overshoot: if braking hard but still moving past target
        if direction * (self.driver_target - self.position) < 0:
            if abs(self.driver_target - self.position) < abs(self.velocity) * dt_s * self.inertia_factor:
                self.velocity *= 0.85


motor = Nema23Motor()
sim_state = {
    "link_active": True,
    "link_drop_at": None,
    "link_restore_at": None,
    "last_rx_hex": "WAITING...",
    "init_count": 0,
    "packets_rx": 0,
    "packets_dropped": 0,
    "telemetry_file": None,
    "last_cmd_target": 0,
}


def modbus_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


def decode_position_write(req):
    """Match AP_ModbusSteering: values[0]=high word, values[1]=low word."""
    high_word = (req[7] << 8) | req[8]
    low_word = (req[9] << 8) | req[10]
    if high_word & 0x8000:
        high_word -= 0x10000
    return (high_word << 16) | low_word


def encode_position(pos):
    pos_u = pos & 0xFFFFFFFF
    high_word = (pos_u >> 16) & 0xFFFF
    low_word = pos_u & 0xFFFF
    return high_word, low_word


def _log_telemetry(now, cmd_target):
    path = sim_state.get("telemetry_file")
    if path is None:
        return
    with open(path, "a", encoding="ascii") as fh:
        fh.write(f"{now:.3f},{cmd_target},{motor.actual_pos},{motor.encoder_pos},"
                 f"{motor.velocity:.1f},{abs(motor.driver_target - motor.position):.0f},"
                 f"{motor.error_code}\n")


def check_link_schedule(now):
    if sim_state["link_drop_at"] is not None and now >= sim_state["link_drop_at"]:
        if sim_state["link_active"]:
            sim_state["link_active"] = False
            print(f"[CL57R Modbus Sim] LINK DOWN (inertia coast v={motor.velocity:+.0f} pps)")
        sim_state["link_drop_at"] = None

    if sim_state["link_restore_at"] is not None and now >= sim_state["link_restore_at"]:
        if not sim_state["link_active"]:
            sim_state["link_active"] = True
            print(f"[CL57R Modbus Sim] LINK UP (pos={motor.actual_pos} v={motor.velocity:+.0f} pps)")
        sim_state["link_restore_at"] = None


def handle_write_single(reg_addr, val):
    if reg_addr == 0x0030:
        pass  # start speed
    elif reg_addr == 0x0033:
        motor.max_rpm = val
    elif reg_addr == 0x0031:
        motor.accel_ms = val
    elif reg_addr == 0x0032:
        motor.decel_ms = val
    elif reg_addr == 0x0038:
        motor.motor_enabled = (val == 0x0001)
        if not motor.motor_enabled:
            motor.velocity = 0.0
    elif reg_addr == 0x0037 and val == 0x0004:
        motor.error_code = 0
        motor.status_word &= ~(1 << 3)
    elif reg_addr == 0x0052:
        motor.track_err_limit = val
    elif reg_addr == 0x0036:
        pass  # motion trigger handled via 0x10 position write


def run_modbus_simulator(link_drop_delay=None, link_down_duration=None, encoder_lag_ms=0.0,
                         telemetry_file=None):
    motor.encoder_lag_s = encoder_lag_ms / 1000.0
    motor.reported_position = motor.position
    sim_state["telemetry_file"] = telemetry_file
    if telemetry_file:
        with open(telemetry_file, "w", encoding="ascii") as fh:
            fh.write("t,cmd_target,actual,encoder,vel,follow,err\n")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, MODBUS_UDP_PORT))
    print(f"[CL57R Modbus Sim] NEMA23 inertia model, UDP {MODBUS_UDP_PORT}")
    if link_drop_delay is not None and link_down_duration is not None:
        sim_state["link_drop_at"] = time.time() + link_drop_delay
        sim_state["link_restore_at"] = time.time() + link_drop_delay + link_down_duration
        print(f"[CL57R Modbus Sim] Scheduled link drop in {link_drop_delay}s for {link_down_duration}s")

    raw_buffer = bytearray()
    last_log_time = time.time()
    last_physics_time = time.time()
    saw_enable = False
    enable_writes = 0

    while True:
        try:
            now = time.time()
            check_link_schedule(now)

            dt_s = now - last_physics_time
            motor.update(dt_s, sim_state["link_active"])
            last_physics_time = now

            sock.settimeout(0.02)
            try:
                data, addr = sock.recvfrom(1024)
                raw_buffer.extend(data)
            except socket.timeout:
                addr = None

            if not sim_state["link_active"]:
                raw_buffer.clear()
            else:
                while len(raw_buffer) >= 4:
                    slave_id = raw_buffer[0]
                    func_code = raw_buffer[1]

                    if func_code == 0x06:
                        frame_len = 8
                    elif func_code == 0x10:
                        if len(raw_buffer) < 7:
                            break
                        bytes_count = raw_buffer[6]
                        frame_len = 7 + bytes_count + 2
                    elif func_code == 0x03:
                        frame_len = 8
                    else:
                        raw_buffer.pop(0)
                        continue

                    if len(raw_buffer) < frame_len:
                        break

                    req = raw_buffer[:frame_len]
                    del raw_buffer[:frame_len]

                    received_crc = (req[-1] << 8) | req[-2]
                    if modbus_crc(req[:-2]) != received_crc:
                        continue

                    sim_state["packets_rx"] += 1
                    sim_state["last_rx_hex"] = req.hex().upper()
                    reg_addr = (req[2] << 8) | req[3]
                    response = bytearray()

                    if func_code == 0x06:
                        val = (req[4] << 8) | req[5]
                        handle_write_single(reg_addr, val)
                        if reg_addr == 0x0038 and val == 0x0001:
                            enable_writes += 1
                            sim_state["init_count"] = enable_writes
                            print(f"[CL57R Modbus Sim] INIT #{enable_writes} motor enable")
                            saw_enable = True
                        elif reg_addr == 0x0038 and val == 0x0000:
                            saw_enable = False
                        response.extend(req[:6])

                    elif func_code == 0x10:
                        if reg_addr == 0x0034 and len(req) >= 13:
                            cmd = decode_position_write(req)
                            motor.set_driver_target(cmd)
                            sim_state["last_cmd_target"] = cmd
                            _log_telemetry(now, cmd)
                        response.append(slave_id)
                        response.append(0x10)
                        response.extend(req[2:6])

                    elif func_code == 0x03:
                        if reg_addr == 0x0007:
                            high_word, low_word = encode_position(motor.encoder_pos)
                            response.append(slave_id)
                            response.append(0x03)
                            response.append(0x04)
                            response.append((high_word >> 8) & 0xFF)
                            response.append(high_word & 0xFF)
                            response.append((low_word >> 8) & 0xFF)
                            response.append(low_word & 0xFF)
                        elif reg_addr == 0x0003:
                            response.append(slave_id)
                            response.append(0x03)
                            response.append(0x04)
                            response.append(0)
                            response.append(motor.status_word & 0xFF)
                            response.append(0)
                            response.append(motor.error_code & 0xFF)

                    if len(response) > 0 and addr is not None:
                        crc = modbus_crc(response)
                        response.append(crc & 0xFF)
                        response.append((crc >> 8) & 0xFF)
                        sock.sendto(response, addr)
                    elif len(response) > 0:
                        sim_state["packets_dropped"] += 1

            if now - last_log_time >= 2.0:
                link = "UP" if sim_state["link_active"] else "DOWN"
                follow = int(abs(motor.driver_target - motor.position))
                enc_lag = motor.actual_pos - motor.encoder_pos
                print(f"[STEER] link={link} pos={motor.actual_pos:7d} enc={motor.encoder_pos:7d} "
                      f"target={motor.driver_target:7d} vel={motor.velocity:+7.0f}pps "
                      f"follow={follow:6d} enc_lag={enc_lag:+5d} err={motor.error_code} "
                      f"inits={sim_state['init_count']}")
                last_log_time = now

        except Exception as e:
            print(f"[Modbus Error] {e}")
            raw_buffer.clear()
            time.sleep(0.1)


def parse_args():
    parser = argparse.ArgumentParser(description="CL57R/NEMA23 Modbus UDP simulator")
    parser.add_argument("--link-drop-delay", type=float, default=None,
                        help="Seconds after start to drop Modbus link")
    parser.add_argument("--link-down-duration", type=float, default=None,
                        help="Seconds to keep link down before restore")
    parser.add_argument("--encoder-lag-ms", type=float, default=0.0,
                        help="Encoder read lag in milliseconds (simulates stale feedback)")
    parser.add_argument("--telemetry-file", type=str, default=None,
                        help="CSV log of position commands and motor state")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    drop_delay = args.link_drop_delay
    down_duration = args.link_down_duration
    if (drop_delay is None) != (down_duration is None):
        raise SystemExit("Both --link-drop-delay and --link-down-duration are required together")
    run_modbus_simulator(drop_delay, down_duration, args.encoder_lag_ms, args.telemetry_file)
