#!/usr/bin/env python3
"""
indoor_test_cli.py
Interactive CLI for indoor flight testing — no GPS.
Runs on Raspberry Pi 5 connected to Matek H743 via UART.

Usage:
    python indoor_test_cli.py
"""

import time
import sys
import threading
import collections
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
SERIAL_PORT    = "/dev/ttyAMA0"
BAUD_RATE      = 921600

THROTTLE_CLIMB   = 1570   # PWM to climb in ALT_HOLD
THROTTLE_HOVER   = 1500   # PWM to hold altitude
THROTTLE_DESCEND = 1380   # PWM for gentle manual descent

CLIMB_TIMEOUT  = 25.0     # Max seconds to wait for target altitude
RC_RATE_HZ     = 10.0     # How often to send RC overrides while holding


# ==============================================================================
# MAVLINK STATE — updated by background monitor thread
# ==============================================================================
state = {
    "armed":   False,
    "mode":    "UNKNOWN",
    "alt_m":   None,      # relative altitude (barometer)
    "bat_pct": None,
    "ekf_ok":  None,
}

# STATUSTEXT messages captured by the monitor thread for arm() to print
_statustext_buf = collections.deque(maxlen=20)
_monitor_stop   = threading.Event()


def _monitor_loop(m):
    """
    Background daemon thread — sole owner of MAVLink recv.
    Updates state dict and auto-releases RC overrides on STABILIZE detection.
    """
    while not _monitor_stop.is_set():
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        t = msg.get_type()

        if t == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                mode = mavutil.mode_string_v10(msg)
            except Exception:
                mode = str(msg.custom_mode)

            # Auto-release RC overrides when pilot switches to STABILIZE
            if mode == "STABILIZE" and state["mode"] != "STABILIZE":
                print("\n\n  !! Pilot switched to STABILIZE — auto-releasing RC overrides !!\n")
                release_rc(m)

            state["armed"] = armed
            state["mode"]  = mode

        elif t == "GLOBAL_POSITION_INT":
            state["alt_m"] = msg.relative_alt / 1000.0

        elif t == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                state["bat_pct"] = msg.battery_remaining

        elif t == "EKF_STATUS_REPORT":
            flags = msg.flags
            state["ekf_ok"] = bool(flags & 0x1F)  # attitude + vert pos + vert vel OK

        elif t == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode()
            _statustext_buf.append(text.strip())


def drain(m, max_msgs=30):
    """Background thread owns MAVLink reading; drain() just yields briefly."""
    time.sleep(0.05)


def status_line():
    alt  = f"{state['alt_m']:.2f}m" if state['alt_m'] is not None else "?"
    bat  = f"{state['bat_pct']}%"   if state['bat_pct'] is not None else "?"
    ekf  = ("OK" if state['ekf_ok'] else "WARN") if state['ekf_ok'] is not None else "?"
    arm  = "ARMED" if state["armed"] else "disarmed"
    return f"[{arm}] mode={state['mode']} alt={alt} bat={bat} ekf={ekf}"


# ==============================================================================
# HELPERS
# ==============================================================================

def send_rc(m, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    chans = [0] * 18
    chans[0] = roll
    chans[1] = pitch
    chans[2] = throttle
    chans[3] = yaw
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chans)

def release_rc(m):
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *([0] * 18))
    print("  RC overrides released.")

def set_mode(m, mode_str):
    mode_str = mode_str.upper()
    mm = m.mode_mapping()
    if mode_str not in mm:
        print(f"  ERROR: mode '{mode_str}' not available. Options: {list(mm.keys())}")
        return False
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode_str]
    )
    time.sleep(0.5)
    drain(m)
    print(f"  Mode set to {mode_str}. Current: {state['mode']}")
    return True

def arm(m):
    if state["armed"]:
        print("  Already armed.")
        return True
    print("  Sending arm command...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    # Poll state (updated by monitor thread); print any STATUSTEXT pre-arm failures
    t0 = time.time()
    while time.time() - t0 < 10.0:
        while _statustext_buf:
            print(f"  FC: {_statustext_buf.popleft()}")
        if state["armed"]:
            print("  ARMED successfully.")
            return True
        time.sleep(0.2)
    while _statustext_buf:
        print(f"  FC: {_statustext_buf.popleft()}")
    print("  Arm failed (timeout). Check FC messages above for pre-arm failures.")
    return False

def disarm(m, force=False):
    print("  Sending disarm command...")
    p2 = 21196.0 if force else 0.0  # magic number forces disarm in flight
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, p2, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < 5.0:
        drain(m)
        if not state["armed"]:
            print("  Disarmed.")
            return True
        time.sleep(0.2)
    print("  Disarm timeout — FC may still be armed.")
    return False

def wait_for_alt(m, target_m, tolerance=0.3, timeout=CLIMB_TIMEOUT):
    """Block until altitude is within tolerance of target, sending RC climb the whole time."""
    print(f"  Climbing to {target_m:.1f}m... (timeout {timeout:.0f}s)")
    t0 = time.time()
    last_rc = 0.0
    while time.time() - t0 < timeout:
        now = time.time()
        drain(m)
        alt = state["alt_m"]
        if alt is not None:
            print(f"  Alt: {alt:.2f}m / {target_m:.1f}m", end="\r")
            if alt >= target_m - tolerance:
                send_rc(m, throttle=THROTTLE_HOVER)
                print(f"\n  Target altitude reached: {alt:.2f}m")
                return True
        if now - last_rc >= 1.0 / RC_RATE_HZ:
            send_rc(m, throttle=THROTTLE_CLIMB)
            last_rc = now
    print(f"\n  Climb timeout after {timeout:.0f}s. Current alt: {state['alt_m']}")
    return False


# ==============================================================================
# MENU ACTIONS
# ==============================================================================

def action_status(m):
    drain(m)
    print(f"\n  {status_line()}")

def action_arm(m):
    print("\n  Setting ALT_HOLD before arming (no GPS)...")
    set_mode(m, "ALT_HOLD")
    arm(m)

def action_disarm(m):
    print()
    release_rc(m)
    disarm(m)

def action_takeoff(m):
    if not state["armed"]:
        print("  Not armed — arm first.")
        return
    try:
        target = float(input("  Takeoff altitude (m) [default 1.5]: ").strip() or "1.5")
    except ValueError:
        print("  Invalid input.")
        return

    print(f"\n  Takeoff to {target:.1f}m in ALT_HOLD...")
    set_mode(m, "ALT_HOLD")
    time.sleep(0.3)

    reached = wait_for_alt(m, target)
    if reached:
        print(f"  Holding at {target:.1f}m. Throttle neutral.")
        send_rc(m, throttle=THROTTLE_HOVER)
    else:
        print("  WARNING: Did not reach target altitude. Holding throttle neutral.")
        send_rc(m, throttle=THROTTLE_HOVER)

def action_land(m):
    print("\n  Releasing RC overrides and triggering LAND mode...")
    release_rc(m)
    time.sleep(0.2)
    set_mode(m, "LAND")
    print("  Landing... waiting for disarm.")
    t0 = time.time()
    while time.time() - t0 < 60.0:
        drain(m)
        alt = state["alt_m"]
        print(f"  Alt: {alt:.2f}m" if alt is not None else "  Alt: ?", end="\r")
        if not state["armed"]:
            print("\n  Landed and disarmed.")
            return
        time.sleep(0.3)
    print("\n  Land timeout — check FC.")

def action_alt_hold(m):
    print("\n  Switching to ALT_HOLD and holding current altitude...")
    set_mode(m, "ALT_HOLD")
    send_rc(m, throttle=THROTTLE_HOVER)
    print(f"  Holding. Current alt: {state['alt_m']:.2f}m" if state['alt_m'] else "  Holding.")
    print("  (RC overrides active — call 'Release RC' to hand back control)")

def action_set_mode(m):
    mode = input("\n  Enter mode name (e.g. ALT_HOLD, STABILIZE, LAND): ").strip()
    if mode:
        set_mode(m, mode)

def action_release_rc(m):
    print()
    release_rc(m)

def action_emergency_disarm(m):
    print("\n  !! EMERGENCY DISARM !!")
    confirm = input("  Type YES to confirm force disarm: ").strip()
    if confirm == "YES":
        release_rc(m)
        disarm(m, force=True)
    else:
        print("  Cancelled.")


# ==============================================================================
# MAIN
# ==============================================================================

MENU = [
    ("Status",              action_status),
    ("Arm  (ALT_HOLD)",     action_arm),
    ("Disarm",              action_disarm),
    ("Takeoff to altitude", action_takeoff),
    ("Land",                action_land),
    ("ALT_HOLD (hold alt)", action_alt_hold),
    ("Set mode manually",   action_set_mode),
    ("Release RC overrides",action_release_rc),
    ("EMERGENCY DISARM",    action_emergency_disarm),
]

def print_menu():
    print("\n" + "="*45)
    print(f"  {status_line()}")
    print("="*45)
    for i, (label, _) in enumerate(MENU):
        print(f"  {i+1}. {label}")
    print("  0. Exit")
    print("="*45)

def main():
    print(f"Connecting to {SERIAL_PORT} @ {BAUD_RATE}...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)

    print("Waiting for heartbeat...")
    m.wait_heartbeat(timeout=15)
    print("Starting background monitor thread...")
    _monitor_stop.clear()
    monitor = threading.Thread(target=_monitor_loop, args=(m,), daemon=True)
    monitor.start()
    time.sleep(0.3)  # let monitor populate state
    print(f"Connected. {status_line()}\n")
    print("  Note: RC overrides auto-release if pilot switches to STABILIZE.\n")

    while True:
        print_menu()
        try:
            choice = input("  Select: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nCtrl+C — exiting. Releasing RC overrides.")
            _monitor_stop.set()
            release_rc(m)
            sys.exit(0)

        if choice == "0":
            print("Releasing RC overrides and exiting.")
            _monitor_stop.set()
            release_rc(m)
            break

        try:
            idx = int(choice) - 1
            if 0 <= idx < len(MENU):
                MENU[idx][1](m)
            else:
                print("  Invalid option.")
        except ValueError:
            print("  Invalid input.")

if __name__ == "__main__":
    main()
