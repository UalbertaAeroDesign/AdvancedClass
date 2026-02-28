#!/usr/bin/env python3
"""
testTakeoff.py
Simple non-interactive takeoff test — no GPS.
Arms in ALT_HOLD, climbs to target altitude via RC override,
holds for HOVER_SEC, then lands.

Runs on Raspberry Pi 5 connected to Matek H743 via UART.

Usage:
    python testTakeoff.py
"""

import time
import sys
from pymavlink import mavutil

# ==============================================================================
# CONFIG — edit these before each run
# ==============================================================================
SERIAL_PORT   = "/dev/ttyAMA0"
BAUD_RATE     = 921600

TARGET_ALT_M  = 1.5    # metres to climb to
HOVER_SEC     = 5.0    # seconds to hold before landing

THROTTLE_CLIMB   = 1620   # PWM to climb in ALT_HOLD
THROTTLE_HOVER   = 1500   # PWM to hold altitude
CLIMB_TIMEOUT    = 25.0   # seconds before giving up on climb
RC_RATE_HZ       = 10.0   # RC override send rate while climbing


# ==============================================================================
# HELPERS
# ==============================================================================

def drain(m, max_msgs=30):
    for _ in range(max_msgs):
        msg = m.recv_match(blocking=False)
        if msg is None:
            break


def send_rc(m, throttle=1500):
    chans = [0] * 18
    chans[2] = throttle
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chans)


def release_rc(m):
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *([0] * 18))
    print("RC overrides released.")


def set_mode(m, mode_str):
    mm = m.mode_mapping()
    if mode_str not in mm:
        print(f"ERROR: mode '{mode_str}' not available.")
        sys.exit(1)
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode_str]
    )
    time.sleep(0.5)
    drain(m)


def arm(m, timeout=12):
    print("Arming...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode()
            print(f"  FC: {text.strip()}")
        if msg.get_type() == "HEARTBEAT":
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("ARMED.")
                return True
    print("Arm failed — check pre-arm messages above.")
    return False


def get_alt(m):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        return msg.relative_alt / 1000.0
    return None


def climb_to(m, target_m):
    print(f"Climbing to {target_m:.1f}m...")
    t0 = time.time()
    last_rc = 0.0
    alt = None
    while time.time() - t0 < CLIMB_TIMEOUT:
        now = time.time()
        drain(m)
        a = get_alt(m)
        if a is not None:
            alt = a
        if alt is not None:
            print(f"  alt: {alt:.2f}m / {target_m:.1f}m", end="\r")
            if alt >= target_m - 0.3:
                send_rc(m, throttle=THROTTLE_HOVER)
                print(f"\nReached {alt:.2f}m.")
                return True
        if now - last_rc >= 1.0 / RC_RATE_HZ:
            send_rc(m, throttle=THROTTLE_CLIMB)
            last_rc = now
    print(f"\nClimb timeout. Last alt: {alt}")
    return False


def land_and_wait(m, timeout=60):
    print("Switching to LAND...")
    release_rc(m)
    time.sleep(0.2)
    set_mode(m, "LAND")
    t0 = time.time()
    while time.time() - t0 < timeout:
        drain(m)
        msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.3)
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            print(f"  alt: {alt:.2f}m", end="\r")
        if msg.get_type() == "HEARTBEAT":
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                print("\nLanded and disarmed.")
                return
    print("\nLand timeout — check FC.")


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print(f"Connecting to {SERIAL_PORT} @ {BAUD_RATE}...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    m.wait_heartbeat(timeout=15)
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")

    print("Setting ALT_HOLD...")
    set_mode(m, "ALT_HOLD")

    if not arm(m):
        sys.exit(1)

    if not climb_to(m, TARGET_ALT_M):
        release_rc(m)
        set_mode(m, "LAND")
        sys.exit(1)

    print(f"Holding for {HOVER_SEC:.0f}s...")
    t0 = time.time()
    last_rc = 0.0
    while time.time() - t0 < HOVER_SEC:
        now = time.time()
        drain(m)
        if now - last_rc >= 1.0 / RC_RATE_HZ:
            send_rc(m, throttle=THROTTLE_HOVER)
            last_rc = now
        time.sleep(0.05)

    land_and_wait(m)
    print("Done.")


if __name__ == "__main__":
    main()
