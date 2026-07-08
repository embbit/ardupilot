#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run Rover SITL with CL57R Modbus steering simulator and basic checks."""

import os
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(ROOT, "modules/mavlink"))

from pymavlink import mavutil  # noqa: E402


def wait_for_tcp_port(host, port, timeout_s=30):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            sock = socket.create_connection((host, port), timeout=0.5)
            sock.close()
            return True
        except OSError:
            time.sleep(0.2)
    return False


def start_process(cmd, cwd=ROOT):
    print(f"START: {' '.join(cmd)}")
    return subprocess.Popen(
        cmd,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )


def stream_output(proc, label, store):
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            print(f"[{label}] {line}")
            store.append(line)


def main():
    sitl_lines = []
    sim_lines = []

    sim_proc = start_process([sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py")])
    sim_thread = threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True)
    sim_thread.start()
    time.sleep(0.5)

    rover_bin = os.path.join(ROOT, "build/sitl/bin/ardurover")
    defaults = os.path.join(ROOT, "ports.parm")
    rover_cmd = [
        rover_bin,
        "--model", "rover",
        "--speedup", "1",
        "--defaults", defaults,
        "-I0",
        "--serial5=udpclient:127.0.0.1:14555",
    ]
    rover_proc = start_process(rover_cmd)
    sitl_thread = threading.Thread(target=stream_output, args=(rover_proc, "SITL", sitl_lines), daemon=True)
    sitl_thread.start()

    if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
        print("FAIL: SITL did not open TCP port 5760")
        return 1

    mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
    mavlink.wait_heartbeat(timeout=30)

    driver_ready = False
    link_lost = False
    deadline = time.time() + 60

    while time.time() < deadline:
        msg = mavlink.recv_match(blocking=False)
        while msg is not None:
            if msg.get_type() == "STATUSTEXT":
                text = msg.text
                print(f"[MAV] {text}")
                if "CL57R: Modbus Driver READY" in text or "Modbus link restored" in text:
                    driver_ready = True
                if "CL57R: Modbus link lost" in text:
                    link_lost = True
            msg = mavlink.recv_match(blocking=False)
        if driver_ready:
            break
        time.sleep(0.1)

    if not driver_ready:
        print("FAIL: CL57R driver did not reach READY within timeout")
        return 1

    if link_lost:
        print("WARN: link lost observed during init")

    print("PASS: init sequence completed")

    mavlink.mav.command_long_send(
        mavlink.target_system,
        mavlink.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1,  0, 0, 0, 0, 0, 0,
    )
    time.sleep(0.5)
    mavlink.set_mode_apm("MANUAL")
    time.sleep(0.5)

    def send_stick(pwm, label):
        print(f"STICK {label} pwm={pwm}")
        mavlink.mav.rc_channels_override_send(
            mavlink.target_system,
            mavlink.target_component,
            pwm, 0, 1500, 1500, 1500, 1500, 1500, 1500,
        )

    send_stick(1900, "RIGHT")
    time.sleep(4)
    send_stick(1100, "LEFT")
    time.sleep(4)
    send_stick(1500, "CENTER")
    time.sleep(2)

    moved = any("target=" in line and "target=      0" not in line for line in sim_lines[-20:])
    if moved:
        print("PASS: simulator received non-zero steering targets")
    else:
        print("WARN: simulator did not log non-zero targets (check SIM output)")

    for proc in (rover_proc, sim_proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
