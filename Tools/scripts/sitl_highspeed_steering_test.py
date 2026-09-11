#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""High-speed steering smoothness and encoder tracking error SITL tests."""

import csv
import math
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

CL57R_STEPS_PER_REV = 4000
POSITION_INTERVAL_MS = 50


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


def speed_move_limit_pulses(rpm, cycle_ms):
    return max(int(rpm * CL57R_STEPS_PER_REV / 60 * cycle_ms / 1000), 1)


def analyze_telemetry(path, rpm):
    rows = []
    with open(path, newline="", encoding="ascii") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            rows.append({
                "cmd": int(float(row["cmd_target"])),
                "actual": int(float(row["actual"])),
                "encoder": int(float(row["encoder"])),
                "vel": float(row["vel"]),
                "follow": int(float(row["follow"])),
                "err": int(float(row["err"])),
            })

    if len(rows) < 5:
        return {"ok": False, "reason": "too few telemetry samples"}

    cmd_steps = [abs(rows[i]["cmd"] - rows[i - 1]["cmd"]) for i in range(1, len(rows))]
    active_limit = speed_move_limit_pulses(rpm, POSITION_INTERVAL_MS * 2)
    max_step = max(cmd_steps) if cmd_steps else 0
    big_steps = sum(1 for s in cmd_steps if s > active_limit * 2)
    track_err_count = sum(1 for r in rows if r["err"] == 4)
    max_follow = max(r["follow"] for r in rows)
    enc_lags = [abs(r["actual"] - r["encoder"]) for r in rows]
    max_enc_lag = max(enc_lags) if enc_lags else 0

    vel_samples = [r["vel"] for r in rows]
    vel_deltas = [abs(vel_samples[i] - vel_samples[i - 1]) for i in range(1, len(vel_samples))]
    max_vel_jump = max(vel_deltas) if vel_deltas else 0.0
    mean_vel_jump = sum(vel_deltas) / len(vel_deltas) if vel_deltas else 0.0

    return {
        "ok": True,
        "samples": len(rows),
        "active_limit": active_limit,
        "max_cmd_step": max_step,
        "big_cmd_steps": big_steps,
        "track_err_count": track_err_count,
        "max_follow": max_follow,
        "max_enc_lag": max_enc_lag,
        "max_vel_jump": max_vel_jump,
        "mean_vel_jump": mean_vel_jump,
    }


def run_case(name, rpm, stick_fn, duration_s, encoder_lag_ms=0.0):
    print(f"\n=== CASE: {name} @ {rpm} RPM ===")
    telemetry = tempfile.NamedTemporaryFile(prefix="steer_", suffix=".csv", delete=False)
    telemetry_path = telemetry.name
    telemetry.close()

    parm_extra = tempfile.NamedTemporaryFile(mode="w", prefix="ob_str_", suffix=".parm", delete=False)
    with open(os.path.join(ROOT, "ports.parm"), encoding="ascii") as src:
        parm_extra.write(src.read())
    parm_extra.write(f"\nOB_STR_MAX_SPD {rpm}\n")
    parm_extra.close()

    sim_lines = []
    events = []
    sim_cmd = [
        sys.executable, "-u", os.path.join(ROOT, "outb_steer_sim.py"),
        "--telemetry-file", telemetry_path,
        "--encoder-lag-ms", str(encoder_lag_ms),
    ]
    sim_proc = start_process(sim_cmd)
    threading.Thread(target=stream_output, args=(sim_proc, "SIM", sim_lines), daemon=True).start()
    time.sleep(0.5)

    rover_proc = start_process([
        os.path.join(ROOT, "build/sitl/bin/ardurover"),
        "--model", "rover", "--speedup", "1",
        "--defaults", parm_extra.name,
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

        deadline = time.time() + 60
        ready = False
        while time.time() < deadline:
            msg = m.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "STATUSTEXT":
                    text = msg.text
                    print(f"[MAV] {text}")
                    events.append(text)
                    if "Modbus link restored" in text or "Modbus Driver READY" in text:
                        ready = True
                msg = m.recv_match(blocking=False)
            if ready:
                break
            time.sleep(0.05)

        if not ready:
            print("FAIL: driver not ready")
            return rc

        m.param_set_send("OB_STR_MAX_SPD", float(rpm), mavutil.mavlink.MAV_PARAM_TYPE_INT16)
        time.sleep(0.5)
        m.set_mode_apm("MANUAL")
        time.sleep(0.5)

        t0 = time.time()
        while time.time() - t0 < duration_s:
            pwm = stick_fn(time.time() - t0)
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component,
                int(pwm), 0, 1500, 1500, 1500, 1500, 1500, 1500,
            )
            msg = m.recv_match(type="STATUSTEXT", blocking=False)
            while msg is not None:
                print(f"[MAV] {msg.text}")
                events.append(msg.text)
                msg = m.recv_match(type="STATUSTEXT", blocking=False)
            time.sleep(0.05)

        m.mav.rc_channels_override_send(
            m.target_system, m.target_component,
            1500, 0, 1500, 1500, 1500, 1500, 1500, 1500,
        )
        time.sleep(1.0)

        stats = analyze_telemetry(telemetry_path, rpm)
        if not stats["ok"]:
            print(f"FAIL: {stats['reason']}")
            return rc

        print(f"  samples={stats['samples']} active_limit={stats['active_limit']}")
        print(f"  max_cmd_step={stats['max_cmd_step']} big_steps={stats['big_cmd_steps']}")
        print(f"  max_follow={stats['max_follow']} track_err_rows={stats['track_err_count']}")
        print(f"  max_vel_jump={stats['max_vel_jump']:.0f} mean_vel_jump={stats['mean_vel_jump']:.0f}")
        print(f"  max_enc_lag={stats['max_enc_lag']}")

        mav_track = sum(1 for t in events if "error 0x0004" in t or "Tracking error" in t)
        print(f"  mav_tracking_errors={mav_track}")

        # Pass criteria
        if name.startswith("slow"):
            if stats["big_cmd_steps"] > 1 or stats["max_cmd_step"] > stats["active_limit"] * 6:
                print("FAIL: slow sweep has jerky command steps")
                return rc
            if stats["max_vel_jump"] > stats["active_limit"] * 10:
                print("FAIL: velocity jumps too large in slow sweep")
                return rc
        elif name.startswith("fast"):
            if mav_track == 0 and stats["track_err_count"] == 0:
                print("WARN: expected tracking errors on fast step at high RPM")

        if encoder_lag_ms > 0 and stats["max_enc_lag"] < encoder_lag_ms * rpm * CL57R_STEPS_PER_REV / 60000:
            print("WARN: encoder lag lower than expected")

        if name.startswith("desync") and stats["track_err_count"] == 0 and mav_track == 0:
            print("FAIL: encoder desync case produced no tracking errors")
            return rc

        print(f"PASS: {name} @ {rpm} RPM")
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
        time.sleep(2)
        if os.path.exists(telemetry_path):
            os.unlink(telemetry_path)
        if os.path.exists(parm_extra.name):
            os.unlink(parm_extra.name)


def main():
    cases = [
        ("slow_sweep", 1500, lambda t: 1500 + min(t / 8.0, 1.0) * 400, 10.0, 0.0),
        # 2000 RPM slow sweep: run manually; full travel may trigger past_limit pulls (~80k steps)
        ("fast_step", 1500, lambda t: 1900 if t > 0.5 else 1500, 6.0, 0.0),
        ("fast_step", 2000, lambda t: 1900 if t > 0.5 else 1500, 6.0, 0.0),
        ("desync_lag", 2000, lambda t: 1500 + min(t / 4.0, 1.0) * 400, 8.0, 150.0),
    ]

    failures = 0
    for i, (name, rpm, fn, dur, lag) in enumerate(cases):
        if i > 0:
            time.sleep(5)
        if run_case(name, rpm, fn, dur, lag) != 0:
            failures += 1

    if failures:
        print(f"\n{failures} CASE(S) FAILED")
        return 1
    print("\nALL HIGH-SPEED TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
