#!/usr/bin/env python3
"""
testFixedWingMission.py  —  Tricopter fixed-wing waypoint test.

Uploads a 3-item mission (TAKEOFF → WAYPOINT → RTL), arms in STABILIZE,
then switches to AUTO to execute. The aircraft hand-launches, flies to the
waypoint in fixed-wing mode, and returns to land.

NOTE: Q_RTL_MODE controls whether RTL lands as fixed-wing (0) or VTOL (1).
Set Q_RTL_MODE=0 in params for a full fixed-wing return and landing.

Runs on Raspberry Pi connected to flight controller via UART.

Usage:
    python testFixedWingMission.py
"""

import time
import sys
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
SERIAL_PORT  = "/dev/ttyAMA0"
BAUD_RATE    = 921600

TAKEOFF_ALT_M = 30.0   # metres AGL to climb before transitioning to cruise

# Waypoint to fly to - these arent real so make sure to replace with real coords before flight
WP_LAT =  53.5269      # decimal degrees
WP_LON = -113.5256     # decimal degrees
WP_ALT =  30.0         # metres AGL

ARM_TIMEOUT     = 15   # seconds to wait for arm confirmation
MISSION_TIMEOUT = 300  # seconds before giving up on mission completion


# ==============================================================================
# HELPERS
# ==============================================================================

def clear_mailbox(m, max_msgs=30):
    for _ in range(max_msgs):
        if m.recv_match(blocking=False) is None:
            break


def set_mode(m, mode_str):
    mm = m.mode_mapping()
    if mode_str not in mm:
        print(f"ERROR: mode '{mode_str}' not available on this vehicle.")
        sys.exit(1)
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode_str]
    )
    time.sleep(0.5)
    clear_mailbox(m)


def arm(m, timeout=15):
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


def clear_mission(m):
    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        msg = m.recv_match(type="MISSION_ACK", blocking=False)
        if msg:
            break
        time.sleep(0.05)


def upload_mission(m, items):
    print(f"Uploading {len(items)}-item mission...")
    m.mav.mission_count_send(m.target_system, m.target_component, len(items))
    sent = 0
    while sent < len(items):
        req = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"],
                           blocking=True, timeout=10)
        if not req:
            print("Timeout waiting for MISSION_REQUEST. Aborting.")
            sys.exit(1)
        i = req.seq
        it = items[i]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component,
            i,
            it["frame"], it["cmd"],
            it["current"], it["autocontinue"],
            it["p1"], it["p2"], it["p3"], it["p4"],
            int(it["lat"] * 1e7), int(it["lon"] * 1e7), it["alt"]
        )
        print(f"  Sent item {i}: {it['label']}")
        sent += 1
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"Mission upload rejected (type={getattr(ack, 'type', '?')}). Aborting.")
        sys.exit(1)
    print("Mission uploaded OK.")


def wait_for_position(m, timeout=20):
    """Block until we have a valid GPS fix."""
    print("Waiting for GPS fix...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=False)
        if msg and msg.fix_type >= 3:
            print(f"  GPS fix OK (type={msg.fix_type}, sats={msg.satellites_visible})")
            return True
        time.sleep(0.1)
    print("No GPS fix within timeout.")
    return False


def monitor_mission(m, timeout=300):
    """Print status until the mission completes (vehicle disarms or RTL finishes)."""
    print("Mission running — monitoring (Ctrl+C to abort)...")
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            clear_mailbox(m)
            msg = m.recv_match(
                type=["MISSION_CURRENT", "MISSION_ITEM_REACHED",
                      "HEARTBEAT", "GLOBAL_POSITION_INT"],
                blocking=True, timeout=0.5
            )
            if msg is None:
                continue
            t = msg.get_type()
            if t == "MISSION_CURRENT":
                print(f"  Mission item: {msg.seq}")
            elif t == "MISSION_ITEM_REACHED":
                print(f"  Reached item: {msg.seq}")
            elif t == "GLOBAL_POSITION_INT":
                alt = msg.relative_alt / 1000.0
                print(f"  alt: {alt:.1f}m", end="\r")
            elif t == "HEARTBEAT":
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if not armed:
                    print("\nVehicle disarmed — mission complete.")
                    return
    except KeyboardInterrupt:
        print("\nAborted by user.")


# ==============================================================================
# MISSION DEFINITION
# ==============================================================================

def build_mission(takeoff_alt, wp_lat, wp_lon, wp_alt):
    # Item 0: dummy home (seq 0 is skipped by ArduPlane in AUTO)
    # Item 1: fixed-wing takeoff
    # Item 2: cruise to waypoint
    # Item 3: RTL (return to home and land)
    return [
        {
            "label": "Home (placeholder)",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": 0, "lon": 0, "alt": 0
        },
        {
            "label": f"TAKEOFF to {takeoff_alt}m",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "current": 1, "autocontinue": 1,
            "p1": 15,  # min pitch (deg) during climb
            "p2": 0, "p3": 0, "p4": 0,
            "lat": 0, "lon": 0, "alt": takeoff_alt
        },
        {
            "label": f"WAYPOINT ({wp_lat:.4f}, {wp_lon:.4f})",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": wp_lat, "lon": wp_lon, "alt": wp_alt
        },
        {
            "label": "RTL",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": 0, "lon": 0, "alt": 0
        },
    ]


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print(f"Connecting to {SERIAL_PORT} @ {BAUD_RATE}...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    m.wait_heartbeat(timeout=15)
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")

    if not wait_for_position(m):
        print("Aborting — no GPS.")
        sys.exit(1)

    mission = build_mission(TAKEOFF_ALT_M, WP_LAT, WP_LON, WP_ALT)
    clear_mission(m)
    upload_mission(m, mission)

    # Arm in STABILIZE — ArduPlane won't arm in AUTO
    print("Setting STABILIZE...")
    set_mode(m, "STABILIZE")

    if not arm(m, timeout=ARM_TIMEOUT):
        sys.exit(1)

    # Switch to AUTO — hand-launch the aircraft to trigger takeoff
    print("Setting AUTO — hand-launch now!")
    set_mode(m, "AUTO")

    monitor_mission(m, timeout=MISSION_TIMEOUT)
    print("Done.")


if __name__ == "__main__":
    main()
