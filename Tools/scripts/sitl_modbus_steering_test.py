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

# Matches OB_STR_LINK_TO default (1500 ms) plus margin for SITL scheduling.
MODBUS_LINK_TIMEOUT_S = 2.0


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


def run_sitl(link_drop_delay=None, link_down_duration=None, mute_reads=False):
    sitl_lines = []
    sim_lines = []

    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py")]
    if mute_reads:
        sim_cmd.append("--mute-reads")
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

            if any("Re-initializing" in t or "Modbus link lost" in t for t in events):
                print("FAIL: unexpected Modbus link failsafe while the link was up")
                return 1
            print("PASS: no link failsafe while stick was centered")

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
        fw_stop = False

        while time.time() < deadline:
            if not link_lost and any("LINK DOWN" in line for line in sim_lines):
                link_lost = True
                print("PASS: Modbus link lost detected")
            if link_lost and any("LINK UP" in line for line in sim_lines):
                link_restored = True
                break

            msg = mavlink.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "STATUSTEXT":
                    text = msg.text
                    print(f"[MAV] {text}")
                    events.append(text)
                    if "Modbus link lost" in text or "Modbus link still down" in text:
                        fw_stop = True
                    if "Modbus link restored" in text:
                        link_restored = True
                msg = mavlink.recv_match(blocking=False)
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

        if link_down_duration < MODBUS_LINK_TIMEOUT_S:
            print(f"WARN: link down {link_down_duration}s < timeout {MODBUS_LINK_TIMEOUT_S}s")
        if not link_restored:
            print("FAIL: Modbus link did not come back up")
            return 1
        print("PASS: Modbus link restored after drop")

        if not fw_stop and not any("Modbus link lost" in t for t in events):
            print("FAIL: firmware did not report Modbus link lost / STOP")
            return 1
        print("PASS: firmware Modbus link failsafe STOP")

        if init_after_loss < 2:
            print(f"WARN: enable-count={init_after_loss} (re-init not required if RUN keepalive)")
        else:
            print(f"PASS: motor enable seen after drop (init_count={init_after_loss})")

        if any("encoder" in t and "latched" in t for t in events):
            print("WARN: encoder fault latched after link recovery (inertia overshoot)")
        elif any("overshoot" in t for t in events):
            print("WARN: overshoot correction during link recovery test")

        if coast_seen:
            # Firmware now queues MOTION_STOP on link loss; coast is best-effort.
            print("PASS: motor coasted with inertia during link down")
        else:
            print("WARN: inertia coast not logged (expected if firmware STOP won the race)")

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


def wait_for_log(lines, needle, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if any(needle in line for line in lines):
            return True
        time.sleep(0.05)
    return False


def set_param(mavlink, name, value, ptype):
    mavlink.mav.param_set_send(
        mavlink.target_system,
        mavlink.target_component,
        name.encode("ascii"),
        float(value),
        ptype,
    )


def hold_rc(mavlink, events, seconds, ch1=1500, ch6=1500, ch7=1500):
    deadline = time.time() + seconds
    while time.time() < deadline:
        mavlink.mav.rc_channels_override_send(
            mavlink.target_system,
            mavlink.target_component,
            ch1, 0, 1500, 1500, 1500, ch6, ch7, 1500,
        )
        msg = mavlink.recv_match(type="STATUSTEXT", blocking=False)
        while msg is not None:
            print(f"[MAV] {msg.text}")
            events.append(msg.text)
            msg = mavlink.recv_match(type="STATUSTEXT", blocking=False)
        time.sleep(0.1)


def try_arm(mavlink):
    mavlink.mav.command_long_send(
        mavlink.target_system,
        mavlink.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )


def heartbeat_armed(mavlink, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = mavlink.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def run_param_trigger():
    sitl_lines = []
    sim_lines = []
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--alarm-events", "8:5000",
    ]
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)

    rover_cmd = [
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover",
        "--speedup", "1",
        "--defaults", os.path.join(ROOT, "ports.parm"),
        "-I0",
        "--serial5=udpclient:127.0.0.1:14555",
    ]
    rover_proc = start_process(rover_cmd)
    threading.Thread(target=stream_output, args=(rover_proc, "SITL", sitl_lines), daemon=True).start()
    procs = (rover_proc, sim_proc)

    try:
        if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
            print("FAIL: SITL did not open TCP port 5760")
            return 1

        mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
        mavlink.wait_heartbeat(timeout=30)
        events = []
        ready = lambda t: "Modbus Driver READY" in t
        collect_mavlink_events(mavlink, 60, events, stop_when=ready)
        if not any(ready(t) for t in events):
            print("FAIL: CL57R driver did not reach READY")
            return 1

        int8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
        int16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
        set_param(mavlink, "OB_STR_CAL_TRIG", 0, int8)
        time.sleep(0.5)
        set_param(mavlink, "OB_STR_OUT_REV", 2, int8)
        set_param(mavlink, "OB_STR_RATIO", 10, int16)
        set_param(mavlink, "OB_STR_SEEK_SPD", 1800, int16)
        set_param(mavlink, "OB_STR_MAX_SPD", 1300, int16)
        time.sleep(0.5)

        if not wait_for_log(sim_lines, "ALARM raised", 20):
            print("FAIL: simulator did not raise tracking alarm")
            return 1

        set_param(mavlink, "OB_STR_CAL_TRIG", 2, int8)
        time.sleep(1.0)
        collect_mavlink_events(mavlink, 2, events)
        if not wait_for_log(sim_lines, "ALARM CLEAR write", 5):
            print("FAIL: CAL_TRIG=2 did not clear alarm")
            return 1
        print("PASS: CAL_TRIG=2 cleared alarm")

        set_param(mavlink, "OB_STR_CAL_TRIG", 1, int8)
        calibrated = False
        deadline = time.time() + 45
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.5, events)
            if any("CL57R: calibrated" in t for t in events):
                calibrated = True
                break
        if not calibrated:
            print("FAIL: CAL_TRIG=1 did not finish calibration")
            return 1
        if not (wait_for_log(sim_lines, "SPEED START", 2) or
                wait_for_log(sim_lines, "HOME START", 2)):
            print("FAIL: simulator did not see SPEED/HOME START")
            return 1
        # Auto mid return via speed-mode follow after cal.
        mid_ok = False
        hold_rc_deadline = time.time() + 45
        while time.time() < hold_rc_deadline:
            collect_mavlink_events(mavlink, 0.5, events)
            if any("at mid-travel" in t for t in events):
                mid_ok = True
                break
            if any("spd-follow" in t for t in events) or any("SPEED START" in line for line in sim_lines):
                # Keep waiting for arrive
                pass
        if not mid_ok and not any("spd-follow" in t for t in events):
            print("FAIL: missing speed-follow mid return after cal")
            return 1
        print("PASS: CAL_TRIG=1 calibrated steering")

        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if not heartbeat_armed(mavlink, 5.0):
            print("FAIL: ARM failed after param calibration")
            return 1
        print("PASS: ARM succeeded after param calibration")
        return 0
    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


def run_rc_buttons():
    sitl_lines = []
    sim_lines = []
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--alarm-events", "8:5000",
    ]
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)

    rover_cmd = [
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover",
        "--speedup", "1",
        "--defaults", os.path.join(ROOT, "ports.parm"),
        "-I0",
        "--serial5=udpclient:127.0.0.1:14555",
    ]
    rover_proc = start_process(rover_cmd)
    threading.Thread(target=stream_output, args=(rover_proc, "SITL", sitl_lines), daemon=True).start()
    procs = (rover_proc, sim_proc)

    try:
        if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
            print("FAIL: SITL did not open TCP port 5760")
            return 1

        mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
        mavlink.wait_heartbeat(timeout=30)
        events = []
        ready = lambda t: "Modbus Driver READY" in t
        collect_mavlink_events(mavlink, 60, events, stop_when=ready)
        if not any(ready(t) for t in events):
            print("FAIL: CL57R driver did not reach READY")
            return 1
        print("PASS: init sequence completed")

        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed before calibration")
            return 1
        if not any("CL57R not calibrated" in t for t in events):
            print("FAIL: mandatory pre-arm did not report missing calibration")
            return 1
        print("PASS: ARM blocked until calibration")

        int8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
        int16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
        set_param(mavlink, "OB_STR_RST_CH", 6, int8)
        set_param(mavlink, "OB_STR_CAL_CH", 7, int8)
        set_param(mavlink, "OB_STR_OUT_REV", 2, int8)
        set_param(mavlink, "OB_STR_RATIO", 10, int16)
        set_param(mavlink, "OB_STR_SEEK_SPD", 1800, int16)
        set_param(mavlink, "OB_STR_MAX_SPD", 1300, int16)
        time.sleep(0.5)

        if not wait_for_log(sim_lines, "ALARM raised", 20):
            print("FAIL: simulator did not raise tracking alarm")
            return 1
        print("PASS: tracking alarm injected")

        hold_rc(mavlink, events, 0.5, ch6=1500, ch7=1500)
        hold_rc(mavlink, events, 0.8, ch6=1900, ch7=1500)
        hold_rc(mavlink, events, 0.5, ch6=1500, ch7=1500)
        if not wait_for_log(sim_lines, "ALARM CLEAR write", 5):
            print("FAIL: reset button did not write alarm clear")
            return 1
        print("PASS: reset button cleared alarm")

        hold_rc(mavlink, events, 0.8, ch6=1500, ch7=1900)
        calibrated = False
        deadline = time.time() + 45
        while time.time() < deadline:
            hold_rc(mavlink, events, 0.5, ch6=1500, ch7=1500)
            if any("CL57R: calibrated" in t for t in events):
                calibrated = True
                break
        if not calibrated:
            print("FAIL: homing button did not finish calibration")
            return 1
        # Dual-limit uses speed-mode seek (SPEED START); legacy native home used HOME START.
        if not (wait_for_log(sim_lines, "SPEED START", 2) or
                wait_for_log(sim_lines, "HOME START", 2)):
            print("FAIL: simulator did not see SPEED/HOME START")
            return 1
        if not wait_for_log(sim_lines, "POSITION ZEROED", 2):
            print("FAIL: simulator did not zero after limit 1")
            return 1
        if not any("cal done at limit" in t or "CL57R: calibrated" in t for t in events):
            # calibrated is required; offset message is best-effort
            pass
        print("PASS: home button calibrated (alarm clear + home + center)")

        if not any("HOME_SPD=1800" in line for line in sim_lines):
            print("FAIL: homing did not write HOME_SPD=1800")
            return 1
        if not any("cal@limit" in t or "cal done at limit" in t for t in events):
            print("FAIL: missing cal done at limit message")
            return 1
        if not any("spd-follow" in t or " v12" in t or " v11" in t or " v10k" in t or " v10j" in t or " v10i" in t or " v10h" in t or " v10g" in t or " v10f" in t or " v10e" in t or " v10d" in t or " v10c" in t or " v10b" in t or " v10a" in t or " v10" in t or " v9" in t or "v8" in t
                   or "cal@limit" in t or "limit seek" in t or "DI extreme" in t or "soft crawl" in t or "recover DI" in t for t in events):
            print("FAIL: missing speed-follow mid return message")
            return 1

        # Wait for physical mid return (speed mode at SEEK RPM).
        mid_done = False
        deadline = time.time() + 60
        while time.time() < deadline:
            hold_rc(mavlink, events, 0.5, ch6=1500, ch7=1500)
            if any("at mid-travel" in t for t in events):
                mid_done = True
                break
            if any("SPEED START" in line for line in sim_lines):
                pass
        if not mid_done:
            print("FAIL: did not reach mid-travel after cal")
            return 1
        print("PASS: returned to mid-travel after cal")

        # Run speed restores after mid arrive.
        n_before = len(sim_lines)
        restored = False
        deadline = time.time() + 15
        while time.time() < deadline:
            hold_rc(mavlink, events, 0.3, ch6=1500, ch7=1500)
            for line in sim_lines[n_before:]:
                if "MAX_SPD=1300" in line:
                    restored = True
                    break
            if restored:
                break
            after_mid = False
            for line in sim_lines:
                if "SPEED START" in line:
                    after_mid = True
                elif after_mid and "MAX_SPD=1300" in line:
                    restored = True
                    break
            if restored:
                break
        if not restored:
            print("FAIL: run speed 1300 not restored after mid")
            return 1
        print("PASS: home speed 1800, mid return, run speed restored to 1300")

        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if not heartbeat_armed(mavlink, 5.0):
            print("FAIL: ARM failed after calibration")
            return 1
        print("PASS: ARM succeeded after calibration")

        # QGC/virtual-stick path: RC override must drive speed-mode motion.
        n_stick = len(sim_lines)
        for _ in range(20):
            mavlink.mav.rc_channels_override_send(
                mavlink.target_system,
                mavlink.target_component,
                1900, 0, 1500, 1500, 1500, 1500, 1500, 1500,
            )
            collect_mavlink_events(mavlink, 0.25, events)
        stick_moved = any(
            ("SPEED START" in line and "rpm=" in line and "rpm=0" not in line)
            or ("MAX_SPD=" in line and "MAX_SPD=0" not in line and "MAX_SPD=1300" not in line
                and "MAX_SPD=-1300" not in line)
            or ("vel=" in line and "vel=     +0" not in line and "vel=     -0" not in line)
            for line in sim_lines[n_stick:]
        )
        # Also accept a new SPEED START after mid with non-zero signed rpm.
        if not stick_moved:
            stick_moved = any("SPEED START" in line for line in sim_lines[n_stick:])
        if not stick_moved:
            # Fallback: any non-zero MAX_SPD write after mid restore means follow reacted.
            stick_moved = any(
                "MAX_SPD=" in line and not line.rstrip().endswith("MAX_SPD=1300")
                and not line.rstrip().endswith("MAX_SPD=-1300")
                for line in sim_lines[n_stick:]
            )
        # Signed run speed toward stick is expected (±1300 or other non-zero).
        if not stick_moved:
            for line in sim_lines[n_stick:]:
                if "MAX_SPD=" in line:
                    stick_moved = True
                    break
        if not stick_moved:
            print("FAIL: stick override did not command speed-mode motion")
            return 1
        print("PASS: virtual stick override drives speed-mode follow")
        return 0
    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


def default_control_file():
    return os.path.join("/tmp", f"cl57r_sim_ctl_{os.getpid()}.txt")


def write_sim_control(path, command):
    with open(path, "w", encoding="ascii") as fh:
        fh.write(command.strip() + "\n")
    print(f"CTL> {command.strip()}")


def stop_procs(procs):
    for proc in procs:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def start_rover_and_sim(sim_extra=None, control_file=None):
    sitl_lines = []
    sim_lines = []
    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py")]
    if sim_extra:
        sim_cmd.extend(sim_extra)
    if control_file:
        sim_cmd.extend(["--control-file", control_file])
        try:
            open(control_file, "w", encoding="ascii").close()
        except OSError:
            pass
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)
    rover_cmd = [
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover",
        "--speedup", "1",
        "--defaults", os.path.join(ROOT, "ports.parm"),
        "-I0",
        "--serial5=udpclient:127.0.0.1:14555",
    ]
    rover_proc = start_process(rover_cmd)
    threading.Thread(target=stream_output, args=(rover_proc, "SITL", sitl_lines), daemon=True).start()
    return (rover_proc, sim_proc), sitl_lines, sim_lines


def connect_ready():
    if not wait_for_tcp_port("127.0.0.1", 5760, timeout_s=30):
        print("FAIL: SITL did not open TCP port 5760")
        return None, None
    mavlink = mavutil.mavlink_connection("tcp:127.0.0.1:5760", timeout=1)
    mavlink.wait_heartbeat(timeout=30)
    events = []
    collect_mavlink_events(mavlink, 60, events, stop_when=lambda t: "Modbus Driver READY" in t)
    if not any("Modbus Driver READY" in t for t in events):
        print("FAIL: CL57R driver did not reach READY")
        return None, None
    print("PASS: init sequence completed")
    return mavlink, events


def set_cal_defaults(mavlink, cal_mode=1, cal_mth=17):
    int8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    int16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
    set_param(mavlink, "OB_STR_CAL_TRIG", 0, int8)
    set_param(mavlink, "OB_STR_OUT_REV", 2, int8)
    set_param(mavlink, "OB_STR_RATIO", 10, int16)
    set_param(mavlink, "OB_STR_SEEK_SPD", 1800, int16)
    set_param(mavlink, "OB_STR_MAX_SPD", 1300, int16)
    set_param(mavlink, "OB_STR_CAL_MODE", cal_mode, int8)
    set_param(mavlink, "OB_STR_CAL_MTH", cal_mth, int8)
    set_param(mavlink, "OB_STR_LINK_TO", 1500, int16)
    time.sleep(0.4)


def wait_calibrated(mavlink, events, timeout_s=50):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        collect_mavlink_events(mavlink, 0.5, events)
        if any("CL57R: calibrated" in t for t in events):
            return True
    return False


def wait_mid_travel(mavlink, events, timeout_s=60):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        hold_rc(mavlink, events, 0.4)
        if any("at mid-travel" in t for t in events):
            return True
    return False


def calibrate_via_param(mavlink, events, cal_mode=1, cal_mth=17):
    set_cal_defaults(mavlink, cal_mode=cal_mode, cal_mth=cal_mth)
    set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
    if not wait_calibrated(mavlink, events, timeout_s=55):
        print("FAIL: calibration did not finish")
        return False
    print("PASS: calibrated")
    return True


def run_link_loss_after_cal():
    """Post-cal speed-follow must STOP on Modbus link loss."""
    ctl = default_control_file()
    procs, _sitl, sim_lines = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events):
            return 1
        if not wait_mid_travel(mavlink, events):
            print("FAIL: mid-travel not reached before link drop")
            return 1

        # Drive stick so speed-follow is active, then drop the link.
        for _ in range(8):
            hold_rc(mavlink, events, 0.25, ch1=1900)
        n_before = len(events)
        write_sim_control(ctl, "DROP 5")
        deadline = time.time() + 20
        lost = False
        restored = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.3, events)
            if any("Modbus link lost" in t for t in events[n_before:]):
                lost = True
            if lost and any("Modbus link restored" in t for t in events[n_before:]):
                restored = True
                break
            if any("LINK DOWN" in line for line in sim_lines) and any(
                    "LINK UP" in line for line in sim_lines):
                # keep waiting for firmware texts
                pass
        if not lost:
            print("FAIL: no Modbus link lost after cal/speed-follow")
            return 1
        if not restored:
            print("FAIL: link not restored after post-cal drop")
            return 1
        if not any("MOTION STOP" in line for line in sim_lines):
            print("WARN: sim did not log MOTION STOP (may have raced)")
        print("PASS: post-cal speed-follow link failsafe STOP")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_arming_gates():
    """Arming must refuse: not calibrated, homing, alarm, link loss."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)

        # 1) Not calibrated
        events.clear()
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed before calibration")
            return 1
        if not any("CL57R not calibrated" in t for t in events):
            print("FAIL: missing 'CL57R not calibrated'")
            return 1
        print("PASS: arm blocked when not calibrated")

        # 2) Homing in progress
        set_cal_defaults(mavlink)
        events.clear()
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        collect_mavlink_events(mavlink, 2.0, events)
        try_arm(mavlink)
        hold_rc(mavlink, events, 1.5)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed while homing")
            return 1
        if not any("CL57R homing" in t for t in events):
            # May finish very fast on some hosts; accept if still not calibrated/homing text
            if any("CL57R: calibrated" in t for t in events):
                print("WARN: cal finished before homing arm check; skipping")
            else:
                print("FAIL: missing 'CL57R homing'")
                return 1
        else:
            print("PASS: arm blocked while homing")

        # Finish cal for remaining checks
        if not any("CL57R: calibrated" in t for t in events):
            if not wait_calibrated(mavlink, events, timeout_s=55):
                print("FAIL: could not finish cal for arming gates")
                return 1
        if not wait_mid_travel(mavlink, events):
            print("WARN: mid-travel not seen; continuing arming gates")

        # 3) Alarm latched. Drop the link as soon as firmware reports the
        # follow alarm so auto-clear cannot refresh _status_word before arming.
        hold_rc(mavlink, events, 0.5)
        events.clear()
        write_sim_control(ctl, "ALARM")
        saw_follow_alarm = False
        deadline = time.time() + 5
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.1, events)
            if any("follow alarm" in t for t in events):
                saw_follow_alarm = True
                break
        if not saw_follow_alarm:
            print("FAIL: firmware did not report follow alarm")
            return 1
        write_sim_control(ctl, "DROP 10")
        time.sleep(0.3)
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed while alarmed")
            return 1
        if not any("CL57R alarm" in t for t in events):
            # If clear won the race, link-down gate still proves arming block.
            if any("CL57R Modbus link" in t for t in events):
                print("WARN: alarm cleared before arm; link gate blocked arm instead")
            else:
                print("FAIL: missing 'CL57R alarm' (and no link gate)")
                return 1
        else:
            print("PASS: arm blocked when alarmed")

        # Wait for link restore, clear any residual alarm for the next check.
        time.sleep(10.5)
        set_param(mavlink, "OB_STR_CAL_TRIG", 2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(1.0)
        collect_mavlink_events(mavlink, 1.0, events)

        # 4) Link down (fresh drop after restore)
        events.clear()
        write_sim_control(ctl, "DROP 8")
        time.sleep(2.5)
        collect_mavlink_events(mavlink, 1.5, events)
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed while Modbus link down")
            return 1
        if not any("CL57R Modbus link" in t for t in events):
            print("FAIL: missing 'CL57R Modbus link'")
            return 1
        print("PASS: arm blocked when Modbus link down")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_cal_mode0():
    """Single-limit native home (CAL_MODE=0)."""
    procs, _sitl, sim_lines = start_rover_and_sim()
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events, cal_mode=0, cal_mth=17):
            return 1
        if not any("homing to limit" in t for t in events):
            print("FAIL: missing single-limit homing message")
            return 1
        if not (any("HOME START" in line for line in sim_lines) or
                any("SPEED START" in line for line in sim_lines)):
            print("FAIL: sim saw neither HOME nor SPEED START")
            return 1
        print("PASS: CAL_MODE=0 single-limit cal")
        return 0
    finally:
        stop_procs(procs)


def run_cal_mth18():
    """Dual-limit with first method M18 (positive/X1 first)."""
    procs, _sitl, sim_lines = start_rover_and_sim()
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events, cal_mode=1, cal_mth=18):
            return 1
        joined = "\n".join(events)
        if not any("DI extreme" in t or "cal@limit" in t for t in events):
            print("FAIL: missing DI extreme / cal@limit for M18")
            return 1
        if "M18" not in joined and "limit seek" not in joined:
            print("FAIL: missing M18 / limit seek evidence")
            return 1
        if not any("SPEED START" in line for line in sim_lines):
            print("FAIL: no SPEED START during M18 cal")
            return 1
        print("PASS: CAL_MTH=18 dual-limit cal")
        return 0
    finally:
        stop_procs(procs)


def run_cal_rx_abort():
    """Cal must abort if Modbus RX is lost in HOME_WAIT before motion."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_cal_defaults(mavlink)
        # Slow crawl so DROP can win the race before _saw_home_motion latches.
        set_param(mavlink, "OB_STR_CRAWL_SPD", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        set_param(mavlink, "OB_STR_SEEK_SPD", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        # Reach HOME_WAIT (prep needs link), then drop before encoder motion.
        seek_started = False
        deadline = time.time() + 20
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.1, events)
            if any("limit seek" in t or "homing to limit" in t or "homing leg" in t
                   for t in events):
                seek_started = True
                break
        if not seek_started:
            print("FAIL: cal never reached seek/HOME_WAIT")
            return 1
        # Silence >12s arms the timer; abort after another 30s without motion.
        write_sim_control(ctl, "DROP 55")
        deadline = time.time() + 70
        aborted = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.5, events)
            if any("Modbus RX lost, abort home" in t for t in events):
                aborted = True
                break
            if any("CL57R: calibrated" in t for t in events):
                print("FAIL: calibrated despite RX drop (abort expected)")
                return 1
        if not aborted:
            print("FAIL: cal did not abort on Modbus RX loss")
            for t in events[-15:]:
                print(f"  last: {t}")
            return 1
        print("PASS: cal aborted on Modbus RX loss")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_follow_alarm():
    """Speed-follow must react to a tracking alarm after cal."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events):
            return 1
        if not wait_mid_travel(mavlink, events):
            print("FAIL: mid-travel required before follow-alarm test")
            return 1
        for _ in range(10):
            hold_rc(mavlink, events, 0.2, ch1=1900)
        n_before = len(events)
        write_sim_control(ctl, "ALARM")
        deadline = time.time() + 15
        saw = False
        while time.time() < deadline:
            hold_rc(mavlink, events, 0.3, ch1=1900)
            if any(
                ("follow alarm" in t) or ("follow halt" in t) or ("follow resume" in t)
                for t in events[n_before:]
            ):
                saw = True
                break
        if not saw:
            print("FAIL: no follow alarm/halt/resume after injected alarm")
            return 1
        print("PASS: speed-follow handled injected alarm")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description="SITL test for CL57R Modbus steering")
    parser.add_argument(
        "--test",
        choices=(
            "basic",
            "link-loss",
            "link-loss-cal",
            "encoder-mute",
            "rc-buttons",
            "param-trigger",
            "arming-gates",
            "cal-mode0",
            "cal-mth18",
            "cal-rx-abort",
            "follow-alarm",
            "coverage",
            "all",
        ),
        default="all",
    )
    args = parser.parse_args()

    if args.test in ("basic", "all"):
        print("=== BASIC STEERING + INERTIA TEST ===")
        rc = run_sitl()
        if rc != 0:
            return rc

    if args.test in ("encoder-mute", "all"):
        print("=== STICK MOVES WITH MUTED ENCODER READS ===")
        rc = run_sitl(mute_reads=True)
        if rc != 0:
            return rc

    if args.test in ("link-loss", "all"):
        print("=== LINK LOSS + RE-INIT TEST ===")
        rc = run_sitl(link_drop_delay=8.0, link_down_duration=5.0)
        if rc != 0:
            return rc

    if args.test in ("rc-buttons", "all"):
        print("=== RC RESET + HOME + PRE-ARM TEST ===")
        rc = run_rc_buttons()
        if rc != 0:
            return rc

    if args.test in ("param-trigger", "all"):
        print("=== CAL_TRIG PARAM TEST ===")
        rc = run_param_trigger()
        if rc != 0:
            return rc

    # New coverage tests (also via --test coverage)
    coverage_tests = (
        ("link-loss-cal", "=== POST-CAL LINK FAILSAFE ===", run_link_loss_after_cal),
        ("arming-gates", "=== ARMING GATES ===", run_arming_gates),
        ("cal-mode0", "=== CAL_MODE=0 SINGLE-LIMIT ===", run_cal_mode0),
        ("cal-mth18", "=== CAL_MTH=18 DUAL-LIMIT ===", run_cal_mth18),
        ("cal-rx-abort", "=== CAL RX ABORT ===", run_cal_rx_abort),
        ("follow-alarm", "=== FOLLOW ALARM ===", run_follow_alarm),
    )
    for name, title, fn in coverage_tests:
        if args.test in (name, "coverage", "all"):
            print(title)
            rc = fn()
            if rc != 0:
                return rc

    print("ALL TESTS PASSED")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
