#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Extended SITL tests: FAULT_LATCHED, OB_STR_INV, repeated link loss."""

import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(ROOT, "modules/mavlink"))

from pymavlink import mavutil  # noqa: E402

FAULT_LIMIT = 300000
FAULT_INJECT_POS = -310000


def build_fault_inject_schedule():
    """Ramp encoder past fault_limit in steps below spike-filter max_jump (~3200 @ 120 RPM)."""
    specs = []
    pos = 0
    t0 = 10.0
    step = -2500
    while pos > FAULT_INJECT_POS:
        pos += step
        if pos < FAULT_INJECT_POS:
            pos = FAULT_INJECT_POS
        specs.append(f"{t0:.1f}:{pos}")
        t0 += 0.35
    return ",".join(specs)


LINK_DOWN_S = 5.0
LINK_GAP_S = 15.0


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
        cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )


def stream_output(proc, label, store):
    for line in proc.stdout:
        line = line.rstrip()
        if line:
            print(f"[{label}] {line}")
            store.append(line)


def make_parm_file(overrides):
    fd = tempfile.NamedTemporaryFile(mode="w", prefix="ob_str_ext_", suffix=".parm", delete=False)
    with open(os.path.join(ROOT, "ports.parm"), encoding="ascii") as src:
        fd.write(src.read())
    for key, val in overrides.items():
        fd.write(f"\n{key} {val}\n")
    fd.close()
    return fd.name


def parse_targets_from_sim(sim_lines):
    targets = []
    for line in sim_lines:
        if "target=" not in line:
            continue
        try:
            part = line.split("target=")[1].split()[0]
            targets.append(int(part))
        except (IndexError, ValueError):
            pass
    return targets


def wait_driver_ready(mavlink, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mavlink.recv_match(blocking=False)
        while msg is not None:
            if msg.get_type() == "STATUSTEXT":
                text = msg.text
                print(f"[MAV] {text}")
                if "Modbus link restored" in text or "Modbus Driver READY" in text:
                    return True
            msg = mavlink.recv_match(blocking=False)
        time.sleep(0.05)
    return False


def run_session(sim_extra_args, parm_overrides, test_fn, timeout_s=90):
    sim_lines = []
    parm_path = make_parm_file(parm_overrides)
    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py")] + sim_extra_args
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)
    if sim_proc.poll() is not None:
        os.unlink(parm_path)
        print("FAIL: simulator exited early")
        return 1, sim_lines, []

    rover_proc = start_process([
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover", "--speedup", "1",
        "--defaults", parm_path,
        "-I0", "--serial5=udpclient:127.0.0.1:14555",
    ])
    threading.Thread(target=stream_output, args=(rover_proc, "SITL", []), daemon=True).start()

    events = []
    rc = 1
    try:
        if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
            print("FAIL: SITL TCP 5760 unavailable")
            return rc, sim_lines, events

        mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
        mavlink.wait_heartbeat(timeout=30)
        if not wait_driver_ready(mavlink, timeout_s=60):
            print("FAIL: driver not ready")
            return rc, sim_lines, events

        mavlink.set_mode_apm("MANUAL")
        time.sleep(0.5)
        rc = test_fn(mavlink, sim_lines, events, timeout_s)
        return rc, sim_lines, events
    finally:
        for proc in (rover_proc, sim_proc):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
        time.sleep(2)
        os.unlink(parm_path)


def test_fault_latched(mavlink, sim_lines, events, timeout_s):
    print("Stick centered, waiting for encoder fault latch...")
    mavlink.mav.rc_channels_override_send(
        mavlink.target_system, mavlink.target_component,
        1500, 0, 1500, 1500, 1500, 1500, 1500, 1500,
    )

    def pump(deadline):
        while time.time() < deadline:
            msg = mavlink.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "STATUSTEXT":
                    text = msg.text
                    print(f"[MAV] {text}")
                    events.append(text)
                msg = mavlink.recv_match(blocking=False)
            time.sleep(0.05)

    pump(time.time() + timeout_s)

    latched = any(("latched" in t or "out of range" in t) and "encoder" in t for t in events)
    motor_disabled = any("MOTOR DISABLE" in line for line in sim_lines)
    if not latched and not motor_disabled:
        print("FAIL: encoder fault latch not detected (no MAVLink, no motor disable)")
        return 1
    if latched:
        print("PASS: encoder fault latched (MAVLink)")
    else:
        print("PASS: encoder fault latched (motor disable in sim)")

    injected = any("INJECT position=-310000" in line for line in sim_lines)
    if not injected:
        print("FAIL: simulator did not reach injected fault position")
        return 1
    print("PASS: out-of-range position reached")

    # Снимок состояния на момент latch
    inits_at_latch = sum(1 for line in sim_lines if "INIT #" in line)
    events_at_latch = len(events)

    mavlink.mav.rc_channels_override_send(
        mavlink.target_system, mavlink.target_component,
        1900, 0, 1500, 1500, 1500, 1500, 1500, 1500,
    )
    time.sleep(3)
    pump(time.time() + 3)

    inits_after = sum(1 for line in sim_lines if "INIT #" in line) - inits_at_latch
    if inits_after > 0:
        print(f"FAIL: {inits_after} unexpected re-init(s) after fault latch")
        return 1
    print("PASS: no re-init after fault latch")

    # Только события ПОСЛЕ latch
    events_post = events[events_at_latch:]
    restored_after = [t for t in events_post if "Modbus link restored" in t]
    if restored_after:
        print(f"FAIL: unexpected link restored after fault latch: {restored_after[-1]}")
        return 1
    print("PASS: driver stays latched (no auto recovery)")
    return 0


def test_invert(mavlink, sim_lines, events, timeout_s, inv_value, expect_sign):
    print(f"Testing OB_STR_INV={inv_value}, expect sign {'+' if expect_sign > 0 else '-'}")
    mavlink.mav.rc_channels_override_send(
        mavlink.target_system, mavlink.target_component,
        1900, 0, 1500, 1500, 1500, 1500, 1500, 1500,
    )
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = mavlink.recv_match(blocking=False)
        while msg is not None:
            if msg.get_type() == "STATUSTEXT":
                print(f"[MAV] {msg.text}")
            msg = mavlink.recv_match(blocking=False)
        time.sleep(0.05)

    targets = [t for t in parse_targets_from_sim(sim_lines) if abs(t) > 5000]
    if not targets:
        print("FAIL: no significant steering targets observed")
        return 1
    peak = max(targets, key=abs)
    print(f"  peak target={peak}")
    if expect_sign > 0 and peak < 0:
        print("FAIL: expected positive target with OB_STR_INV=0")
        return 1
    if expect_sign < 0 and peak > 0:
        print("FAIL: expected negative target with OB_STR_INV=1")
        return 1
    print(f"PASS: OB_STR_INV={inv_value} direction correct")
    return 0


def test_multi_link_loss(mavlink, sim_lines, events, timeout_s):
    print("Stick centered during triple link loss cycle...")
    mavlink.mav.rc_channels_override_send(
        mavlink.target_system, mavlink.target_component,
        1500, 0, 1500, 1500, 1500, 1500, 1500, 1500,
    )
    link_lost = 0
    link_restored = 0
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        msg = mavlink.recv_match(blocking=False)
        while msg is not None:
            if msg.get_type() == "STATUSTEXT":
                text = msg.text
                print(f"[MAV] {text}")
                events.append(text)
                if "Modbus link lost" in text:
                    link_lost += 1
                if "Modbus link restored" in text:
                    link_restored += 1
            msg = mavlink.recv_match(blocking=False)
        if link_lost >= 3 and link_restored >= 3:
            break
        time.sleep(0.05)

    init_count = 0
    for line in sim_lines:
        if "INIT #" in line:
            try:
                init_count = max(init_count, int(line.split("INIT #")[1].split()[0]))
            except (IndexError, ValueError):
                pass

    link_downs = sum(1 for line in sim_lines if "LINK DOWN" in line)
    link_ups = sum(1 for line in sim_lines if "LINK UP" in line)

    print(f"  link_lost={link_lost} link_restored={link_restored} inits={init_count}")
    print(f"  sim link_down={link_downs} link_up={link_ups}")

    if link_lost < 3:
        print(f"FAIL: expected 3 link lost events, got {link_lost}")
        return 1
    print("PASS: 3x Modbus link lost")

    if link_restored < 3:
        print(f"FAIL: expected 3 link restored events, got {link_restored}")
        return 1
    print("PASS: 3x Modbus link restored")

    if init_count < 4:
        print(f"FAIL: expected init_count>=4 (1 + 3 re-inits), got {init_count}")
        return 1
    print(f"PASS: motor re-initialized {init_count - 1} times")

    mavlink.mav.rc_channels_override_send(
        mavlink.target_system, mavlink.target_component,
        1100, 0, 1500, 1500, 1500, 1500, 1500, 1500,
    )
    time.sleep(3)
    if not any("target=" in line and "target=      0" not in line for line in sim_lines[-12:]):
        print("FAIL: steering dead after triple link recovery")
        return 1
    print("PASS: steering works after triple link recovery")
    return 0


def run_fault_test():
    print("\n=== FAULT_LATCHED TEST ===")
    rc, _, _ = run_session(
        ["--inject-positions", build_fault_inject_schedule()],
        {"OB_STR_MAX_SPD": 1200},
        test_fault_latched,
        timeout_s=100,
    )
    return rc


def run_invert_tests():
    print("\n=== OB_STR_INV TEST (INV=1) ===")
    rc1, _, _ = run_session(
        [], {"OB_STR_INV": 1},
        lambda m, s, e, t: test_invert(m, s, e, t, 1, -1),
        timeout_s=15,
    )
    if rc1 != 0:
        return rc1

    print("\n=== OB_STR_INV TEST (INV=0) ===")
    rc0, _, _ = run_session(
        [], {"OB_STR_INV": 0},
        lambda m, s, e, t: test_invert(m, s, e, t, 0, 1),
        timeout_s=15,
    )
    return rc0


def run_multi_link_test():
    print("\n=== TRIPLE LINK LOSS TEST ===")
    drops = []
    t = 10.0
    for _ in range(3):
        drops.append(f"{t:.0f}:{int(LINK_DOWN_S)}")
        t += LINK_DOWN_S + LINK_GAP_S
    rc, _, _ = run_session(
        ["--link-drops", ",".join(drops)],
        {},
        test_multi_link_loss,
        timeout_s=t + 20,
    )
    return rc


def main():
    parser = __import__("argparse").ArgumentParser(description="Extended CL57R SITL tests")
    parser.add_argument(
        "--test",
        choices=("fault", "invert", "multi-link", "all"),
        default="all",
    )
    args = parser.parse_args()

    tests = []
    if args.test in ("fault", "all"):
        tests.append(("fault", run_fault_test))
    if args.test in ("invert", "all"):
        tests.append(("invert", run_invert_tests))
    if args.test in ("multi-link", "all"):
        tests.append(("multi-link", run_multi_link_test))

    for name, fn in tests:
        if fn() != 0:
            print(f"\n{name.upper()} TEST(S) FAILED")
            return 1
        time.sleep(3)

    print("\nALL EXTENDED TESTS PASSED")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
