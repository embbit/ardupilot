#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Run Rover SITL with CL57R Modbus steering simulator and basic checks."""

import argparse
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

MODBUS_LINK_TIMEOUT_S = 3.5


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


def collect_mavlink_events(mavlink, timeout_s, events, stop_when=None):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mavlink.recv_match(blocking=False)
        while msg is not None:
            if msg.get_type() == "STATUSTEXT":
                text = msg.text
                print(f"[MAV] {text}")
                events.append(text)
                if stop_when is not None and stop_when(text):
                    return
            msg = mavlink.recv_match(blocking=False)
        time.sleep(0.05)


def wait_for_status(events, needle, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        for text in events:
            if needle in text:
                return True
        time.sleep(0.05)
    return False


def run_sitl(link_drop_delay=None, link_down_duration=None):
    sitl_lines = []
    sim_lines = []

    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py")]
    if link_drop_delay is not None:
        sim_cmd.extend([
            "--link-drop-delay", str(link_drop_delay),
            "--link-down-duration", str(link_down_duration),
        ])

    sim_proc = start_process(sim_cmd)
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

    procs = (rover_proc, sim_proc)
    try:
        if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
            print("FAIL: SITL did not open TCP port 5760")
            return 1

        mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
        mavlink.wait_heartbeat(timeout=30)

        events = []
        ready = lambda t: "Modbus link restored" in t or "Modbus Driver READY" in t
        collect_mavlink_events(mavlink, 60, events, stop_when=ready)
        if not any(ready(t) for t in events):
            print("FAIL: CL57R driver did not reach READY")
            return 1
        print("PASS: init sequence completed")

        mavlink.mav.command_long_send(
            mavlink.target_system,
            mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.5)
        mavlink.set_mode_apm("MANUAL")
        time.sleep(0.5)

        def send_stick(pwm, label, hold_s=0.0, expect_abs=None):
            print(f"STICK {label} pwm={pwm}")
            deadline = time.time() + hold_s
            seen_in = False
            while True:
                mavlink.mav.rc_channels_override_send(
                    mavlink.target_system,
                    mavlink.target_component,
                    pwm, 0, 1500, 1500, 1500, 1500, 1500, 1500,
                )
                msg = mavlink.recv_match(
                    type=["NAMED_VALUE_FLOAT", "DEBUG_VECT", "STATUSTEXT"],
                    blocking=False,
                )
                while msg is not None:
                    mtype = msg.get_type()
                    if mtype == "STATUSTEXT":
                        print(f"[MAV] {msg.text}")
                        events.append(msg.text)
                    else:
                        name = getattr(msg, "name", "")
                        if isinstance(name, bytes):
                            name = name.split(b"\x00", 1)[0].decode("ascii", "ignore")
                        name = str(name).rstrip("\x00")
                        if mtype == "NAMED_VALUE_FLOAT" and name.startswith("STR_IN"):
                            print(f"[MAV] STR_IN={msg.value:.3f}")
                            if expect_abs is not None and abs(msg.value) >= expect_abs:
                                seen_in = True
                        if mtype == "DEBUG_VECT" and name.startswith("STEER"):
                            print(f"[MAV] STEER y={msg.y:.0f}")
                            if expect_abs is not None and abs(msg.y) >= 1000:
                                seen_in = True
                    msg = mavlink.recv_match(
                        type=["NAMED_VALUE_FLOAT", "DEBUG_VECT", "STATUSTEXT"],
                        blocking=False,
                    )
                if time.time() >= deadline:
                    break
                time.sleep(0.2)
            return seen_in

        if link_drop_delay is None:
            right_in = send_stick(1900, "RIGHT", hold_s=4, expect_abs=0.2)
            left_in = send_stick(1100, "LEFT", hold_s=4, expect_abs=0.2)
            send_stick(1500, "CENTER", hold_s=3)

            if any("Re-initializing" in t for t in events):
                print("FAIL: unexpected Modbus Timeout while the link was up")
                return 1
            print("PASS: no telemetry timeout while stick was centered")

            moved = any("target=" in line and "target=      0" not in line for line in sim_lines)
            inertia = any("vel=" in line and "vel=     +0" not in line and "vel=     -0" not in line
                          for line in sim_lines)
            if moved:
                print("PASS: simulator received non-zero steering targets")
            else:
                print("FAIL: simulator did not log non-zero targets")
                return 1
            if right_in or left_in:
                print("PASS: GCS stick telemetry followed RC override")
            else:
                print("WARN: STR_IN/DEBUG_VECT not observed (motor still moved)")
            if inertia:
                print("PASS: NEMA23 inertia model shows non-zero velocity")
            else:
                print("WARN: no velocity observed in sim logs")
            return 0

        # Link-loss test: build inertia, center stick, then drop link
        send_stick(1900, "RIGHT", hold_s=2)
        send_stick(1500, "CENTER", hold_s=2)

        events.clear()
        deadline = time.time() + link_drop_delay + link_down_duration + 45
        link_lost = False
        link_restored = False

        while time.time() < deadline:
            msg = mavlink.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "STATUSTEXT":
                    text = msg.text
                    print(f"[MAV] {text}")
                    events.append(text)
                    if "Modbus link lost" in text or "Modbus Timeout" in text:
                        link_lost = True
                    if not link_restored and ("Modbus link restored" in text or
                                              "Modbus Driver READY" in text):
                        link_restored = True
                msg = mavlink.recv_match(blocking=False)

            if link_lost and link_restored:
                break
            time.sleep(0.05)

        init_after_loss = 0
        for line in sim_lines:
            if "INIT #" in line:
                try:
                    init_after_loss = max(init_after_loss, int(line.split("INIT #")[1].split()[0]))
                except (IndexError, ValueError):
                    pass

        send_stick(1500, "CENTER", hold_s=2)

        coast_seen = any("LINK DOWN" in line and "inertia coast" in line for line in sim_lines)
        if not link_lost:
            print("FAIL: did not observe Modbus link lost")
            return 1
        print("PASS: Modbus link lost detected")

        if link_down_duration < MODBUS_LINK_TIMEOUT_S:
            print(f"WARN: link down {link_down_duration}s < timeout {MODBUS_LINK_TIMEOUT_S}s")
        if not link_restored:
            print("FAIL: Modbus link not restored after drop")
            return 1
        print("PASS: Modbus link restored after drop")

        if init_after_loss < 2:
            print(f"FAIL: expected re-init (init_count>=2), got {init_after_loss}")
            return 1
        print(f"PASS: motor re-initialized (init_count={init_after_loss})")

        if any("encoder" in t and "latched" in t for t in events):
            print("WARN: encoder fault latched after link recovery (inertia overshoot)")
        elif any("overshoot" in t for t in events):
            print("WARN: overshoot correction during link recovery test")

        if coast_seen:
            print("PASS: motor coasted with inertia during link down")
        else:
            print("WARN: inertia coast not logged (motor may have been stopped)")

        # Verify steering works again after re-init
        events.clear()
        n_before = len(sim_lines)
        send_stick(1100, "LEFT", hold_s=3)
        send_stick(1500, "CENTER", hold_s=1)

        if not any("target=" in line and "target=      0" not in line for line in sim_lines[n_before:]):
            print("FAIL: no steering commands after re-init")
            return 1
        print("PASS: steering works after link recovery")
        return 0

    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


def main():
    parser = argparse.ArgumentParser(description="SITL test for CL57R Modbus steering")
    parser.add_argument("--test", choices=("basic", "link-loss", "all"), default="all")
    args = parser.parse_args()

    if args.test in ("basic", "all"):
        print("=== BASIC STEERING + INERTIA TEST ===")
        rc = run_sitl()
        if rc != 0:
            return rc

    if args.test in ("link-loss", "all"):
        print("=== LINK LOSS + RE-INIT TEST ===")
        # Drop at 8s, down 5s (>3s MODBUS_LINK_TIMEOUT_MS)
        rc = run_sitl(link_drop_delay=8.0, link_down_duration=5.0)
        if rc != 0:
            return rc

    print("ALL TESTS PASSED")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
