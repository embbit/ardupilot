#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""UDP Modbus RTU simulator for CL57R steering (SERIAL5 protocol 101)."""

import socket
import time

HOST = "0.0.0.0"
MODBUS_UDP_PORT = 14555

CL57R_STEPS_PER_REV = 4000
DEFAULT_MAX_RPM = 120

stepper_state = {
    "target_pos": 0,
    "actual_pos": 0,
    "max_rpm": DEFAULT_MAX_RPM,
    "status_word": 0,
    "error_code": 0,
    "last_rx_hex": "WAITING...",
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
    """Match decode_encoder_position() in AP_ModbusSteering.cpp."""
    pos_u = pos & 0xFFFFFFFF
    high_word = (pos_u >> 16) & 0xFFFF
    low_word = pos_u & 0xFFFF
    return high_word, low_word


def move_toward_target(dt_s):
    max_step = max(int(stepper_state["max_rpm"] * CL57R_STEPS_PER_REV / 60 * dt_s), 1)
    delta = stepper_state["target_pos"] - stepper_state["actual_pos"]
    if abs(delta) <= max_step:
        stepper_state["actual_pos"] = stepper_state["target_pos"]
    elif delta > 0:
        stepper_state["actual_pos"] += max_step
    else:
        stepper_state["actual_pos"] -= max_step


def run_modbus_simulator():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, MODBUS_UDP_PORT))
    print(f"[CL57R Modbus Sim] Listening on UDP {MODBUS_UDP_PORT}...")

    raw_buffer = bytearray()
    last_log_time = time.time()
    last_move_time = time.time()

    while True:
        try:
            sock.settimeout(0.05)
            try:
                data, addr = sock.recvfrom(1024)
                raw_buffer.extend(data)
            except socket.timeout:
                pass

            now = time.time()
            move_toward_target(now - last_move_time)
            last_move_time = now

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

                stepper_state["last_rx_hex"] = req.hex().upper()
                reg_addr = (req[2] << 8) | req[3]
                response = bytearray()

                if func_code == 0x06:
                    val = (req[4] << 8) | req[5]
                    if reg_addr == 0x0033:
                        stepper_state["max_rpm"] = val
                    elif reg_addr == 0x0038 and val == 0:
                        stepper_state["error_code"] = 0
                    response.extend(req[:6])

                elif func_code == 0x10:
                    if reg_addr == 0x0034 and len(req) >= 13:
                        stepper_state["target_pos"] = decode_position_write(req)
                    response.append(slave_id)
                    response.append(0x10)
                    response.extend(req[2:6])

                elif func_code == 0x03:
                    if reg_addr == 0x0007:
                        high_word, low_word = encode_position(stepper_state["actual_pos"])
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
                        response.append(stepper_state["status_word"] & 0xFF)
                        response.append(0)
                        response.append(stepper_state["error_code"] & 0xFF)

                if len(response) > 0:
                    crc = modbus_crc(response)
                    response.append(crc & 0xFF)
                    response.append((crc >> 8) & 0xFF)
                    sock.sendto(response, addr)

            if now - last_log_time >= 2.0:
                err = stepper_state["actual_pos"] - stepper_state["target_pos"]
                print(f"[STEER] actual={stepper_state['actual_pos']:7d} "
                      f"target={stepper_state['target_pos']:7d} "
                      f"err={err:+6d} rpm={stepper_state['max_rpm']}")
                last_log_time = now

        except Exception as e:
            print(f"[Modbus Error] {e}")
            raw_buffer.clear()
            time.sleep(0.1)


if __name__ == "__main__":
    run_modbus_simulator()
