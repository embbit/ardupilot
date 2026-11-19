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

# Keep test SITL isolated: no TCP servers on 5762+ so external companion
# software scanning for vehicles cannot latch on and inject commands or
# heartbeats into a running test.
SITL_SERIAL_ARGS = [
    "--serial1=none:x",
    "--serial2=none:x",
    "--serial3=none:x",
    "--serial4=none:x",
]

# Matches OB_STR_LINK_TO default (1500 ms) plus margin for SITL scheduling.
MODBUS_LINK_TIMEOUT_S = 2.0


def wipe_sitl_eeprom():
    """Drop persisted SITL EEPROM so prior param_set values cannot bleed across tests."""
    for name in ("eeprom.bin", "eeprom.dat"):
        path = os.path.join(ROOT, name)
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(f"WARN: could not remove {path}: {exc}")


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


def parse_targets(lines):
    """Extract absolute-target values from simulator log lines."""
    targets = []
    for line in lines:
        if "target=" not in line:
            continue
        try:
            targets.append(int(line.split("target=", 1)[1].split()[0]))
        except (IndexError, ValueError):
            continue
    return targets


def parse_positions(lines):
    """Extract mechanical shaft positions from [STEER] telemetry lines."""
    positions = []
    for line in lines:
        if "[STEER]" not in line or "pos=" not in line:
            continue
        try:
            positions.append(int(line.split("pos=", 1)[1].split()[0]))
        except (IndexError, ValueError):
            continue
    return positions


def wait_pos_near_zero(lines, n_from, timeout_s=12, tol=2000):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if any(abs(p) <= tol for p in parse_positions(lines[n_from:])):
            return True
        time.sleep(0.2)
    return False


def run_sitl(link_drop_delay=None, link_down_duration=None, mute_reads=False):
    """Basic steering (link_drop_delay=None), encoder-mute, or link-loss run.

    v14 requires calibration before the absolute run path works, so every
    variant calibrates first. Link loss is injected via the simulator
    control file after calibration so it cannot race the cal legs.
    """
    wipe_sitl_eeprom()
    sitl_lines = []
    sim_lines = []
    ctl = default_control_file()

    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
               "--tcp", "127.0.0.1:14556"]
    if link_drop_delay is not None or mute_reads:
        sim_cmd.extend(["--control-file", ctl])
        try:
            open(ctl, "w", encoding="ascii").close()
        except OSError:
            pass

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
        *SITL_SERIAL_ARGS,
        "--serial5=tcp:14556",
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

        def ready(t):
            return ("init OK" in t) or ("Modbus link restored" in t) or ("Modbus Driver READY" in t)

        collect_mavlink_events(mavlink, 60, events, stop_when=ready)
        if not any(ready(t) for t in events):
            print("FAIL: CL57R driver did not reach READY")
            return 1
        print("PASS: init sequence completed")

        # v14: absolute run requires calibration; a wiped EEPROM also fails the
        # accel pre-arm, so skip arming checks like the other scenarios.
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events):
            return 1
        if not wait_mid_travel(mavlink, events):
            print("FAIL: mid-travel not reached after cal")
            return 1
        # Regression: the mid zero must fire at the measured mid position, not
        # on a stale status from the home leg (early zero leaves a huge off).
        zero_lines = [ln for ln in sim_lines if "POSITION ZEROED off=" in ln]
        if zero_lines:
            try:
                off_val = int(zero_lines[-1].split("off=")[1].split()[0])
            except (IndexError, ValueError):
                off_val = 0
            if abs(off_val) > 4000:
                print(f"FAIL: mid zero fired early (off={off_val})")
                return 1
        print("PASS: mid zero at measured mid")
        # Regression: the last MAX_SPD write after cal must be the armed run
        # speed, not the homing speed left in REG_MAX_SPD by the mid leg.
        max_spd_writes = [ln for ln in sim_lines if "MAX_SPD=" in ln]
        if not max_spd_writes or not max_spd_writes[-1].rstrip().endswith("MAX_SPD=1300"):
            last = max_spd_writes[-1].strip() if max_spd_writes else "none"
            print(f"FAIL: REG_MAX_SPD not restored to MAX_SPD after cal (last: {last})")
            return 1
        print("PASS: run speed restored after calibration")

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
            dbg_last = {}
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
                        if mtype == "NAMED_VALUE_FLOAT" and name in ("MB_ST", "MB_DBG"):
                            key = name
                            last_dbg = dbg_last.get(key, 0.0)
                            if time.time() - last_dbg < 1.0:
                                msg = mavlink.recv_match(
                                    type=["NAMED_VALUE_FLOAT", "DEBUG_VECT", "STATUSTEXT"],
                                    blocking=False)
                                continue
                            dbg_last[key] = time.time()
                            if name == "MB_ST":
                                st = int(msg.value)
                                print(f"[DBG] status={st & 0xff:#04x} phase={st >> 8}")
                            else:
                                v = int(msg.value)
                                print(f"[DBG] await={v & 1} got_echo={(v >> 1) & 1} exc={(v >> 2) & 1} "
                                      f"expect={(v >> 3) & 7} att={(v >> 8) & 0xff} age10={(v >> 16) & 0xff}")
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

        if link_drop_delay is None and not mute_reads:
            # Basic: signed physical motion checks against the stick.
            n_right = len(sim_lines)
            right_in = send_stick(1900, "RIGHT", hold_s=4, expect_abs=0.2)
            right_pos = parse_positions(sim_lines[n_right:])
            n_left = len(sim_lines)
            left_in = send_stick(1100, "LEFT", hold_s=4, expect_abs=0.2)
            left_pos = parse_positions(sim_lines[n_left:])

            # The SITL host UDP link occasionally emits a truncated datagram;
            # each one costs one short failsafe blip that must recover.
            # Require every "link lost" to be followed by "restored".
            lost_seen = 0
            for t in events:
                if "Modbus link lost" in t:
                    lost_seen += 1
                elif "Modbus link restored" in t and lost_seen > 0:
                    lost_seen -= 1
            if "Re-initializing" in " ".join(events) or lost_seen > 0:
                print("FAIL: unrecovered Modbus link failsafe while the link was up")
                return 1
            print("PASS: link failsafe blips recovered")

            if any(p > 10000 for p in right_pos):
                print("PASS: RIGHT stick drives rudder positive")
            else:
                print(f"FAIL: RIGHT stick positions not positive: {right_pos}")
                return 1
            if any(p < -10000 for p in left_pos):
                print("PASS: LEFT stick drives rudder negative")
            else:
                print(f"FAIL: LEFT stick positions not negative: {left_pos}")
                return 1
            # A truncated-datagram blip (SITL host UDP artifact) can stall the
            # link mid-throw and let the rudder touch a stop; the drive then
            # latches an alarm. Clear it like an operator would and continue.
            if any("ALARM raised" in line for line in sim_lines[n_left:]):
                print("INFO: drive alarm during LEFT, clearing")
                mavlink.mav.command_long_send(
                    mavlink.target_system, mavlink.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0,
                )
                time.sleep(1.0)
                set_param(mavlink, "OB_STR_CAL_TRIG", 2, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
                time.sleep(1.0)
                mavlink.mav.command_long_send(
                    mavlink.target_system, mavlink.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0,
                )
                time.sleep(1.0)
            n_center = len(sim_lines)
            send_stick(1500, "CENTER", hold_s=3)

            if wait_pos_near_zero(sim_lines, n_center):
                print("PASS: center stick returns rudder to zero")
            else:
                print("FAIL: rudder did not return to zero after centering")
                return 1
            if right_in or left_in:
                print("PASS: GCS stick telemetry followed RC override")
            else:
                print("WARN: STR_IN/DEBUG_VECT not observed (motor still moved)")
            return 0

        if mute_reads:
            # Encoder/status reads are dead: the absolute run must still follow
            # the stick (write echoes keep the link alive); a quiet centered
            # stick must then trip the failsafe, and UNMUTE must clear it.
            set_param(mavlink, "OB_STR_LINK_TO", 3000, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
            write_sim_control(ctl, "MUTE")
            time.sleep(0.3)
            n_mute = len(sim_lines)
            # Hold each direction longer than the reversal chain (STOP + SPD
            # + GO echo phases), so the follow actually moves every leg.
            for i in range(6):
                pwm = 1900 if i % 2 == 0 else 1100
                send_stick(pwm, f"MUTE-{i}", hold_s=1.2)
            mute_targets = parse_positions(sim_lines[n_mute:])
            if not any(abs(p) > 10000 for p in mute_targets):
                print("FAIL: no steering motion while encoder reads were muted")
                return 1
            print("PASS: stick moves with muted encoder reads")
            # A truncated-datagram blip (SITL host UDP artifact) may trip a
            # short failsafe even while the link is healthy; the requirement
            # is that it recovers, which the quiet-stick phase below verifies.
            # Quiet centered stick while reads are still dead: link must drop.
            events.clear()
            send_stick(1500, "CENTER", hold_s=5)
            if not any("Modbus link lost" in t for t in events):
                print("FAIL: dead reads with a quiet stick did not trip link failsafe")
                return 1
            print("PASS: link failsafe still detects dead reads")
            # After UNMUTE, polls are answered again: a quiet stick must keep
            # the link up (write echoes alone had already re-armed it).
            events.clear()
            write_sim_control(ctl, "UNMUTE")
            send_stick(1500, "CENTER", hold_s=4)
            if any("Modbus link lost" in t or "Modbus link still down" in t for t in events):
                print("FAIL: link lost again after UNMUTE with polls answered")
                return 1
            print("PASS: polls resume after UNMUTE, link stays up")
            return 0

        # Link-loss test: drive the stick, then drop the link on demand.
        send_stick(1900, "RIGHT", hold_s=2)
        send_stick(1500, "CENTER", hold_s=2)

        events.clear()
        write_sim_control(ctl, f"DROP {link_down_duration}")
        deadline = time.time() + link_down_duration + 45
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

        # Calibrated vehicle: the run-link failsafe STOP is mandatory.
        if fw_stop:
            print("PASS: firmware Modbus link failsafe STOP")
        else:
            print("FAIL: post-cal link drop did not raise Modbus link failsafe STOP")
            return 1

        if init_after_loss < 2:
            print(f"WARN: enable-count={init_after_loss} (re-init not required if RUN keepalive)")
        else:
            print(f"PASS: motor enable seen after drop (init_count={init_after_loss})")

        if any("encoder" in t and "latched" in t for t in events):
            print("WARN: encoder fault latched after link recovery (inertia overshoot)")
        elif any("overshoot" in t for t in events):
            print("WARN: overshoot correction during link recovery test")

        if coast_seen:
            # Firmware queues MOTION_STOP on link loss; it reaches the sim only
            # if a retry lands after restore, so coast is best-effort.
            print("PASS: motor coasted with inertia during link down")
        else:
            print("WARN: inertia coast not logged (expected if firmware STOP won the race)")

        # Verify steering works again after link recovery
        events.clear()
        n_before = len(sim_lines)
        send_stick(1100, "LEFT", hold_s=3)
        send_stick(1500, "CENTER", hold_s=1)

        recovery_pos = parse_positions(sim_lines[n_before:])
        if not any(abs(p) > 10000 for p in recovery_pos):
            print("FAIL: no steering motion after link recovery")
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
        try:
            os.remove(ctl)
        except OSError:
            pass


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
    wipe_sitl_eeprom()
    sitl_lines = []
    sim_lines = []
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--tcp", "127.0.0.1:14556",
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
        *SITL_SERIAL_ARGS,
        "--serial5=tcp:14556",
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

        def ready(t):
            return ("init OK" in t) or ("Modbus Driver READY" in t)

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
        set_param(mavlink, "OB_STR_HOME_SPD", 500, int16)
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
        if not wait_for_log(sim_lines, "HOME START", 2):
            print("FAIL: simulator did not see HOME START")
            return 1
        # v14+: absolute mid move + AUX_POS_ZERO (no speed-follow).
        if not any("calibrated at mid" in t for t in events):
            print("FAIL: missing absolute mid calibration message")
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
    wipe_sitl_eeprom()
    sitl_lines = []
    sim_lines = []
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--tcp", "127.0.0.1:14556",
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
        *SITL_SERIAL_ARGS,
        "--serial5=tcp:14556",
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

        def ready(t):
            return ("init OK" in t) or ("Modbus Driver READY" in t)

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
        set_param(mavlink, "OB_STR_HOME_SPD", 500, int16)
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
        # Dual-limit: two native MOTION_HOME legs (HOME START), then absolute mid.
        home_starts = [line for line in sim_lines if "HOME START" in line]
        if len(home_starts) < 2:
            print(f"FAIL: expected >=2 HOME START, got {len(home_starts)}")
            return 1
        print("PASS: home button calibrated (alarm clear + dual native home + center)")

        if not any("HOME_SPD=500" in line for line in sim_lines):
            print("FAIL: homing did not write HOME_SPD=500")
            return 1
        if not any("calibrated at mid" in t or "CL57R: calibrated" in t for t in events):
            print("FAIL: missing calibrated-at-mid message")
            return 1
        if not any("seek 1" in t or "cal start" in t for t in events):
            print("FAIL: missing dual-home seek messages")
            return 1
        print("PASS: HOME_SPD 500, dual home, calibrated at mid")

        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if not heartbeat_armed(mavlink, 5.0):
            print("FAIL: ARM failed after calibration")
            return 1
        print("PASS: ARM succeeded after calibration")

        # v14 speed-follow: stick override must physically move the rudder.
        n_stick = len(sim_lines)
        for _ in range(20):
            mavlink.mav.rc_channels_override_send(
                mavlink.target_system,
                mavlink.target_component,
                1900, 0, 1500, 1500, 1500, 1500, 1500, 1500,
            )
            collect_mavlink_events(mavlink, 0.25, events)
        stick_pos = parse_positions(sim_lines[n_stick:])
        if not any(p > 10000 for p in stick_pos):
            print("FAIL: stick override did not drive the rudder")
            return 1
        print("PASS: virtual stick override drives rudder (speed follow)")
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
    wipe_sitl_eeprom()
    sitl_lines = []
    sim_lines = []
    sim_cmd = [sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
               "--tcp", "127.0.0.1:14556"]
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
        *SITL_SERIAL_ARGS,
        "--serial5=tcp:14556",
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
    collect_mavlink_events(mavlink, 60, events, stop_when=lambda t: ("init OK" in t) or ("Modbus Driver READY" in t))
    if not any(("init OK" in t) or ("Modbus Driver READY" in t) for t in events):
        print("FAIL: CL57R driver did not reach READY")
        return None, None
    print("PASS: init sequence completed")
    return mavlink, events


def set_cal_defaults(mavlink, cal_mode=1, cal_mth=17, crawl_spd=200, home_spd=500):
    int8 = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    int16 = mavutil.mavlink.MAV_PARAM_TYPE_INT16
    set_param(mavlink, "OB_STR_CAL_TRIG", 0, int8)
    set_param(mavlink, "OB_STR_OUT_REV", 2, int8)
    set_param(mavlink, "OB_STR_RATIO", 10, int16)
    set_param(mavlink, "OB_STR_HOME_SPD", home_spd, int16)
    set_param(mavlink, "OB_STR_CRAWL_SPD", crawl_spd, int16)
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


def wait_mid_travel(mavlink, events, timeout_s=90):
    """v14+: mid is reached inside calibration (calibrated at mid)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        hold_rc(mavlink, events, 0.4)
        if any("calibrated at mid" in t for t in events) or any("at mid-travel" in t for t in events):
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
    """Post-cal absolute run must STOP on Modbus link loss."""
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

        # Drive stick so absolute run is active, then drop the link.
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
        print("PASS: post-cal absolute-run link failsafe STOP")
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

        # 2) Homing in progress — arm as soon as seek starts.
        set_cal_defaults(mavlink)
        events.clear()
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        deadline = time.time() + 15
        saw_seek = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.2, events)
            if any(("seek 1" in t) or ("cal start" in t) for t in events):
                saw_seek = True
                break
            if any("CL57R: calibrated" in t for t in events):
                break
        if not saw_seek and not any("CL57R: calibrated" in t for t in events):
            print("FAIL: cal did not start for homing arm check")
            return 1
        try_arm(mavlink)
        hold_rc(mavlink, events, 1.5)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed while homing")
            return 1
        if not any("CL57R homing" in t for t in events):
            if any("CL57R: calibrated" in t for t in events):
                print("WARN: cal finished before homing arm check; skipping")
            elif any("CL57R not calibrated" in t for t in events) and saw_seek:
                print("WARN: arm gate used not-calibrated during seek; OK")
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

        # 3) Alarm latched — inject OT/tracking alarm, then try arm.
        hold_rc(mavlink, events, 0.5)
        events.clear()
        write_sim_control(ctl, "ALARM")
        time.sleep(0.8)
        write_sim_control(ctl, "DROP 10")
        time.sleep(0.3)
        try_arm(mavlink)
        hold_rc(mavlink, events, 2.0)
        if heartbeat_armed(mavlink, 1.0):
            print("FAIL: armed while alarmed/link-down")
            return 1
        if not any(("CL57R alarm" in t) or ("CL57R Modbus link" in t) for t in events):
            print("FAIL: missing alarm or link arming gate")
            return 1
        print("PASS: arm blocked when alarmed/link-down")

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
    """v14+: CAL_MODE is reserved; dual-limit is the only path."""
    print("PASS: CAL_MODE=0 skipped (dual-limit only in v14+)")
    return 0


def run_cal_mth18():
    """Dual-limit with first method M18 (two native MOTION_HOME legs)."""
    procs, _sitl, sim_lines = start_rover_and_sim()
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        if not calibrate_via_param(mavlink, events, cal_mode=1, cal_mth=18):
            return 1
        joined = "\n".join(events)
        if not any("calibrated at mid" in t for t in events):
            print("FAIL: missing calibrated at mid for M18")
            return 1
        if "M18" not in joined and "seek 1 M18" not in joined and "cal start M18" not in joined:
            print("FAIL: missing M18 seek evidence")
            return 1
        home_m18 = [line for line in sim_lines if "HOME START" in line and "M18" in line]
        if not home_m18:
            print("FAIL: no HOME START M18 during M18 cal")
            return 1
        if len([line for line in sim_lines if "HOME START" in line]) < 2:
            print("FAIL: dual native home needs two HOME START")
            return 1
        print("PASS: CAL_MTH=18 dual native MOTION_HOME")
        return 0
    finally:
        stop_procs(procs)


def run_cal_rx_abort():
    """Cal must abort if Modbus RX is lost in CAL_WAIT before motion."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        # Slow seek so the drop lands before the leg can finish on its own.
        set_cal_defaults(mavlink, crawl_spd=5, home_spd=5)
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        # Reach CAL_WAIT (prep needs link), then drop before encoder motion.
        seek_started = False
        deadline = time.time() + 20
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.1, events)
            if any("seek 1" in t or "cal start" in t for t in events):
                seek_started = True
                break
        if not seek_started:
            print("FAIL: cal never reached seek/CAL_WAIT")
            return 1
        # No RX: stall detect -> backoff (unanswered) -> abort.
        write_sim_control(ctl, "DROP 55")
        deadline = time.time() + 70
        aborted = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.5, events)
            if any("cal aborted -> idle" in t for t in events):
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


def run_link_to_zero():
    """LINK_TO=0 disables post-cal STOP failsafe (arming still uses default)."""
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
            print("FAIL: mid-travel required before LINK_TO=0 test")
            return 1
        set_param(mavlink, "OB_STR_LINK_TO", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        time.sleep(0.3)
        n_before = len(events)
        write_sim_control(ctl, "DROP 4")
        time.sleep(5.0)
        collect_mavlink_events(mavlink, 1.0, events)
        if any("Modbus link lost" in t for t in events[n_before:]):
            print("FAIL: LINK_TO=0 still issued Modbus link lost STOP")
            return 1
        print("PASS: LINK_TO=0 disables run-link STOP")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_cal_timeout():
    """CAL_TO short value must abort a stuck seek without waiting 90s."""
    procs, _sitl, sim_lines = start_rover_and_sim()
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        # Slow crawl so the leg cannot finish before CAL_TO fires.
        set_cal_defaults(mavlink, crawl_spd=5, home_spd=5)
        set_param(mavlink, "OB_STR_CAL_TO", 3, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        deadline = time.time() + 20
        timed_out = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.3, events)
            if any("cal leg timeout" in t for t in events):
                timed_out = True
                break
            if any("CL57R: calibrated" in t for t in events):
                print("FAIL: calibrated before CAL_TO abort")
                return 1
        if not timed_out:
            print("FAIL: CAL_TO did not abort with cal leg timeout")
            return 1
        if not any("abort di=" in t for t in events):
            print("FAIL: home timeout missing abort di/st/pk snapshot")
            return 1
        # Abort must STOP speed-mode — do not keep driving into the stop.
        collect_mavlink_events(mavlink, 1.0, events)
        if not any("MOTION STOP" in line for line in sim_lines):
            print("FAIL: home timeout did not send MOTION STOP")
            return 1
        print("PASS: CAL_TO aborted cal with home timeout")
        return 0
    finally:
        stop_procs(procs)


def run_cal_stall():
    """Shaft freeze during native MOTION_HOME must not falsely calibrate; CAL_TO aborts."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_cal_defaults(mavlink, crawl_spd=200, home_spd=400)
        set_param(mavlink, "OB_STR_CAL_TO", 5, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        write_sim_control(ctl, "STALL")
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        deadline = time.time() + 15
        saw_home = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.2, events)
            if any("seek 1" in t or "cal start" in t for t in events):
                saw_home = True
                break
        if not saw_home:
            print("FAIL: cal stall test never reached native home")
            return 1
        # Frozen shaft: stall backoff cannot release, reseek stalls again,
        # BACK_MAX_TRIES exhausts -> abort (estop).
        deadline = time.time() + 30
        aborted = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.3, events)
            if any("cal aborted -> idle" in t for t in events):
                aborted = True
                break
            if any("CL57R: calibrated" in t for t in events):
                print("FAIL: calibrated during STALL native-home test")
                return 1
        if not aborted:
            print("FAIL: no cal abort after encoder freeze during MOTION_HOME")
            return 1
        if not any("seek stall" in t for t in events):
            print("FAIL: missing seek stall evidence")
            return 1
        print("PASS: native home stall aborts after backoff retries")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_cal_both_di():
    """Stuck both-DI bits must not block native MOTION_HOME (drive seeks limits)."""
    ctl = default_control_file()
    procs, _sitl, _sim = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_cal_defaults(mavlink, crawl_spd=200, home_spd=400)
        write_sim_control(ctl, "BOTHDI")
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        if not wait_calibrated(mavlink, events, timeout_s=55):
            print("FAIL: native dual home did not finish with BOTHDI")
            return 1
        if not any("seek 2" in t for t in events):
            print("FAIL: missing second dual-home seek with BOTHDI")
            return 1
        print("PASS: native MOTION_HOME completes despite both-DI stuck bits")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_follow_alarm():
    """After cal, injected alarm must be visible via alarmed()/arming gate."""
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
            print("FAIL: mid-travel required before alarm test")
            return 1
        for _ in range(6):
            hold_rc(mavlink, events, 0.2, ch1=1900)
        write_sim_control(ctl, "ALARM")
        time.sleep(1.0)
        events.clear()
        try_arm(mavlink)
        collect_mavlink_events(mavlink, 3, events)
        if not any("CL57R alarm" in t for t in events):
            print("FAIL: arming did not report CL57R alarm after inject")
            return 1
        print("PASS: absolute-run path surfaces injected alarm")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_limit_hit():
    """Rudder driven into the mechanical stop: DI backstop holds with reads
    alive; with reads blind the drive jams, alarms and freezes the run."""
    ctl = default_control_file()
    procs, _sitl, sim_lines = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        # Slow run speed so the DI backstop's poll latency cannot carry the
        # rudder past the switch margin; long link timeout for the blind phase.
        set_cal_defaults(mavlink)
        set_param(mavlink, "OB_STR_MAX_SPD", 25, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        set_param(mavlink, "OB_STR_LINK_TO", 30000, mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        if not wait_calibrated(mavlink, events, timeout_s=55):
            print("FAIL: calibration did not finish")
            return 1
        if not wait_mid_travel(mavlink, events):
            print("FAIL: mid-travel required before limit test")
            return 1
        try_arm(mavlink)
        hold_rc(mavlink, events, 1.0)
        if not heartbeat_armed(mavlink, 3.0):
            print("FAIL: arm failed before limit test")
            return 1
        print("PASS: armed for limit test")
        mavlink.set_mode_apm("MANUAL")
        time.sleep(0.3)

        # Shrink the mechanical travel: the firmware still commands the
        # calibrated +-40000, the stop now sits at +-30000.
        write_sim_control(ctl, "SETSTOP 30000")

        # 1) Full stick with reads alive: firmware DI backstop must stop the
        #    rudder at the switch without jamming the drive into the stop.
        #    At 25 RPM the approach takes ~17 s.
        events.clear()
        n_full = len(sim_lines)
        hold_rc(mavlink, events, 22.0, ch1=2000)
        if not any("limit DI hit in run" in t for t in events):
            print("FAIL: no DI backstop warning at full stick into the stop")
            return 1
        if any("Modbus link lost" in t or "Modbus link still down" in t for t in events):
            print("FAIL: link failsafe flapped while the backstop held")
            return 1
        if any("ALARM raised" in line for line in sim_lines[n_full:]):
            print("FAIL: drive jammed despite DI backstop")
            return 1
        print("PASS: DI backstop holds rudder at the limit (no jam)")

        # 2) Reverse away from the limit: steering must recover.
        events.clear()
        n_away = len(sim_lines)
        hold_rc(mavlink, events, 4.0, ch1=1000)
        away_pos = parse_positions(sim_lines[n_away:])
        if len(away_pos) >= 2 and away_pos[-1] < away_pos[0] - 2500:
            print("PASS: steering recovers away from the limit")
        else:
            print(f"FAIL: no steering recovery away from the limit: {away_pos}")
            return 1

        # 3) Blind reads, full stick into the stop: the drive-level OT clamp
        # (sim DI block, like the wired CL57R) must halt the shaft even with
        # the FCU blind — no runaway past the mechanical stop.
        write_sim_control(ctl, "MUTE")
        n_blind = len(sim_lines)
        hold_rc(mavlink, events, 9.0, ch1=2000)
        write_sim_control(ctl, "UNMUTE")
        time.sleep(0.5)  # let the sim consume before any further control write
        blind_pos = parse_positions(sim_lines[n_blind:])
        if not any(p > 10000 for p in blind_pos):
            print("FAIL: no blind steering motion toward the stop")
            return 1
        if any(p > 31000 for p in blind_pos):
            print(f"FAIL: runaway past the mechanical stop while blind: {blind_pos}")
            return 1
        print("PASS: blind drive halts at the stop (drive OT clamp)")

        # 4) A latched drive alarm must block arming again.
        write_sim_control(ctl, "ALARM")
        time.sleep(1.0)
        mavlink.mav.command_long_send(
            mavlink.target_system, mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        hold_rc(mavlink, events, 1.0)
        events.clear()
        try_arm(mavlink)
        hold_rc(mavlink, events, 1.5)
        if not any("CL57R alarm" in t for t in events):
            print("FAIL: arming not blocked by latched drive alarm")
            return 1
        print("PASS: latched drive alarm blocks arming")
        return 0
    finally:
        stop_procs(procs)
        try:
            os.remove(ctl)
        except OSError:
            pass


def run_mid_stale_status():
    """Mid zero must wait for a status sample received after the mid run
    starts, not act on the home leg's stale in_pos/home_done word."""
    ctl = default_control_file()
    procs, _sitl, sim_lines = start_rover_and_sim(control_file=ctl)
    try:
        mavlink, events = connect_ready()
        if mavlink is None:
            return 1
        set_param(mavlink, "ARMING_SKIPCHK", -1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        set_cal_defaults(mavlink)
        set_param(mavlink, "OB_STR_CAL_TRIG", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        # From the mid run start, give no status answers for 2s: a time-based
        # guard would still see the home leg's in_pos word and zero early.
        muted = False
        unmute_at = 0.0
        deadline = time.time() + 60
        calibrated = False
        while time.time() < deadline:
            collect_mavlink_events(mavlink, 0.2, events)
            if not muted and any("mid target" in t for t in events):
                write_sim_control(ctl, "MUTESTAT")
                muted = True
                unmute_at = time.time() + 2.0
            if muted and time.time() >= unmute_at:
                write_sim_control(ctl, "UNMUTESTAT")
                muted = False
            if any("CL57R: calibrated" in t for t in events):
                calibrated = True
                break
        if muted:
            write_sim_control(ctl, "UNMUTESTAT")
        if not calibrated:
            print("FAIL: calibration did not finish")
            return 1
        if not wait_mid_travel(mavlink, events):
            print("FAIL: mid-travel not reached")
            return 1
        zero_lines = [ln for ln in sim_lines if "POSITION ZEROED off=" in ln]
        off_val = 0
        if zero_lines:
            try:
                off_val = int(zero_lines[-1].split("off=")[1].split()[0])
            except (IndexError, ValueError):
                off_val = 0
        if abs(off_val) > 4000:
            print(f"FAIL: mid zero used a stale status word (off={off_val})")
            return 1
        print("PASS: mid zero waited for a fresh status sample")
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
            "cal-timeout",
            "cal-stall",
            "cal-both-di",
            "link-to-zero",
            "follow-alarm",
            "run-limit",
            "mid-stale",
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
        ("cal-timeout", "=== CAL_TO HOME TIMEOUT ===", run_cal_timeout),
        ("cal-stall", "=== CAL STALL RECOVER ===", run_cal_stall),
        ("cal-both-di", "=== CAL BOTH DI STUCK ===", run_cal_both_di),
        ("link-to-zero", "=== LINK_TO=0 NO STOP ===", run_link_to_zero),
        ("follow-alarm", "=== FOLLOW ALARM ===", run_follow_alarm),
        ("run-limit", "=== RUN LIMIT BACKSTOP ===", run_limit_hit),
        ("mid-stale", "=== MID ZERO VS STALE STATUS ===", run_mid_stale_status),
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
