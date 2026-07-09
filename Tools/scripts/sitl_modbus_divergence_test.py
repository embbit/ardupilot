#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""SITL test: driver recovers from tracking-error alarm (position divergence)."""

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


def start_process(cmd):
    print(f"START: {' '.join(cmd)}")
    return subprocess.Popen(cmd, cwd=ROOT, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)


def stream_output(proc, label, store):
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            print(f"[{label}] {line}")
            store.append(line)


def sim_field(sim_lines, key):
    """Parse latest value of key=NNN from [STEER] sim log lines."""
    for line in reversed(sim_lines):
        if "[STEER]" in line and f"{key}=" in line:
            try:
                seg = line.split(f"{key}=")[1].split()[0]
                return int(seg)
            except (IndexError, ValueError):
                continue
    return None


def main():
    sim_lines = []
    # Alarm at -203210 (past +/-200000 soft limit) at t=12s while stick centered.
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--alarm-events", "12:-203210",
    ]
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)

    rover_proc = start_process([
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover", "--speedup", "1",
        "--defaults", os.path.join(ROOT, "ports.parm"),
        "-I0", "--serial5=udpclient:127.0.0.1:14555",
    ])
    threading.Thread(target=stream_output, args=(rover_proc, "SITL", []), daemon=True).start()

    rc = 1
    try:
        if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
            print("FAIL: no TCP 5760")
            return rc
        m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
        m.wait_heartbeat(timeout=30)

        ready = False
        deadline = time.time() + 60
        while time.time() < deadline:
            msg = m.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "STATUSTEXT":
                    print(f"[MAV] {msg.text}")
                    if "Driver READY" in msg.text or "link restored" in msg.text:
                        ready = True
                msg = m.recv_match(blocking=False)
            if ready:
                break
            time.sleep(0.05)
        if not ready:
            print("FAIL: driver not ready")
            return rc
        print("PASS: init done")

        m.set_mode_apm("MANUAL")
        time.sleep(0.5)
        # Keep stick centered so commanded target is ~0.
        m.mav.rc_channels_override_send(
            m.target_system, m.target_component,
            1500, 0, 1500, 1500, 1500, 1500, 1500, 1500,
        )

        # Wait for alarm to be raised in sim.
        deadline = time.time() + 20
        alarm_seen = False
        while time.time() < deadline:
            if any("ALARM raised" in ln for ln in sim_lines):
                alarm_seen = True
                break
            time.sleep(0.1)
        if not alarm_seen:
            print("FAIL: sim did not raise alarm")
            return rc
        print("PASS: tracking-error alarm raised at -203210")

        # Driver should send alarm-clear and motor should return toward 0.
        deadline = time.time() + 45
        cleared = False
        recovered = False
        while time.time() < deadline:
            if not cleared and any("ALARM CLEARED by driver" in ln for ln in sim_lines):
                cleared = True
                print("PASS: driver cleared the alarm")
            pos = sim_field(sim_lines, "pos")
            if pos is not None and abs(pos) < 20000:
                recovered = True
                break
            time.sleep(0.2)

        if not cleared:
            print("FAIL: driver did not clear the alarm")
            return rc
        if not recovered:
            pos = sim_field(sim_lines, "pos")
            print(f"FAIL: motor did not return to center (pos={pos})")
            return rc
        print(f"PASS: motor recovered to center (pos={sim_field(sim_lines, 'pos')})")
        rc = 0
        return rc
    finally:
        for proc in (rover_proc, sim_proc):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
