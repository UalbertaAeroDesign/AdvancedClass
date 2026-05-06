#!/usr/bin/env python3
"""
fixedWingMission.py  —  Gazebo SITL fixed-wing waypoint mission for tricopter.

Mission: TAKEOFF → WP1 → WP2 → LAND (back at runway/home).
All legs in fixed-wing mode (ArduPlane AUTO).

Before running:
    1. Start Gazebo server:
           gz sim -s -v4 -r worlds/<your_tricopter_world>.sdf
    2. Start ArduPlane SITL (separate terminal):
           sim_vehicle.py -v ArduPlane -f JSON --model JSON --console --map \
               --add-param-file=$HOME/gz_ws/src/ardupilot_gazebo/config/minihawk_vtol.parm \
               --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
    3. Run this script:
           python fixedWingMission.py

NOTE: If arming fails due to pre-arm checks in SITL, set ARMING_CHECK=0
      in the SITL console first, or enable force-arm by setting FORCE_ARM=True below.
"""

import time
import sys
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
CONNECTION_STRING = "udp:127.0.0.1:14551"

# Canberra SITL home (set by minihawk_vtol.parm SIM_OPOS_*):
#   lat=-35.363262, lon=149.165237, alt=584m, hdg=353
# Offsets below are roughly 300m north, then 400m east
TAKEOFF_ALT_M = 30.0   # metres AGL

WP1_LAT = -35.3606     # ~300m north of home
WP1_LON =  149.1652
WP1_ALT =  30.0

WP2_LAT = -35.3606     # ~400m east of WP1
WP2_LON =  149.1696
WP2_ALT =  30.0

# Land back at home runway
LAND_LAT = -35.3633
LAND_LON =  149.1652
LAND_ALT =  0.0

FORCE_ARM       = False  # Set True to bypass pre-arm checks in SITL
MISSION_TIMEOUT = 300    # seconds


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
        print(f"ERROR: mode '{mode_str}' not available.")
        sys.exit(1)
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode_str]
    )
    time.sleep(0.5)
    clear_mailbox(m)
    print(f"Mode: {mode_str}")


def arm(m, force=False, timeout=15):
    print("Arming...")
    force_param = 21196 if force else 0   # 21196 bypasses pre-arm checks
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, force_param, 0, 0, 0, 0, 0
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
    print("Arm failed.")
    return False


def clear_mission(m):
    m.mav.mission_clear_all_send(m.target_system, m.target_component)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        if m.recv_match(type="MISSION_ACK", blocking=False):
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


def wait_for_ekf(m, timeout=30):
    """Wait until STATUSTEXT reports EKF3 active, or GPS fix appears."""
    print("Waiting for EKF / GPS...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=["STATUSTEXT", "GPS_RAW_INT"], blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == "GPS_RAW_INT" and msg.fix_type >= 3:
            print(f"  GPS fix (type={msg.fix_type})")
            return True
        if msg.get_type() == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode()
            if "EKF3 active" in text or "origin" in text.lower():
                print(f"  {text.strip()}")
                return True
    print("EKF/GPS timeout — proceeding anyway (SITL usually has immediate fix).")
    return True   # In SITL the fix is instant; don't block here


def monitor_mission(m, timeout=300):
    print("Mission running — monitoring (Ctrl+C to abort)...")
    t0 = time.time()
    try:
        while time.time() - t0 < timeout:
            clear_mailbox(m)
            msg = m.recv_match(
                type=["MISSION_CURRENT", "MISSION_ITEM_REACHED",
                      "HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"],
                blocking=True, timeout=0.5
            )
            if msg is None:
                continue
            t = msg.get_type()
            if t == "MISSION_CURRENT":
                print(f"  Active item: {msg.seq}")
            elif t == "MISSION_ITEM_REACHED":
                print(f"  Reached item: {msg.seq}")
            elif t == "GLOBAL_POSITION_INT":
                alt = msg.relative_alt / 1000.0
                print(f"  alt: {alt:.1f}m", end="\r")
            elif t == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode()
                print(f"\n  FC: {text.strip()}")
            elif t == "HEARTBEAT":
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("\nVehicle disarmed — mission complete.")
                    return
    except KeyboardInterrupt:
        print("\nAborted by user.")


# ==============================================================================
# MISSION DEFINITION
# ==============================================================================

def build_mission():
    # Let ArduPlane manage the VTOL↔FW transitions automatically.
    # Explicitly commanding DO_VTOL_TRANSITION from a dead hover (0 m/s)
    # causes the aircraft to roll/tumble before it has aerodynamic authority.
    # With NAV_VTOL_TAKEOFF → waypoints → NAV_VTOL_LAND, ArduPlane will
    # auto-transition to FW once it has built sufficient airspeed en route
    # to WP1, and back to VTOL before landing.
    return [
        # Item 0: home placeholder — seq 0 is skipped by ArduPlane in AUTO
        {
            "label": "Home (placeholder)",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": 0.0, "lon": 0.0, "alt": 0.0,
        },
        # Item 1: VTOL takeoff to cruise altitude
        {
            "label": f"VTOL TAKEOFF to {TAKEOFF_ALT_M}m",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
            "current": 1, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": float("nan"),
            "lat": 0.0, "lon": 0.0, "alt": TAKEOFF_ALT_M,
        },
        # Item 2: first waypoint — ArduPlane auto-transitions to FW en route
        {
            "label": f"WP1 ({WP1_LAT:.4f}, {WP1_LON:.4f})",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": WP1_LAT, "lon": WP1_LON, "alt": WP1_ALT,
        },
        # Item 3: second waypoint
        {
            "label": f"WP2 ({WP2_LAT:.4f}, {WP2_LON:.4f})",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": 0,
            "lat": WP2_LAT, "lon": WP2_LON, "alt": WP2_ALT,
        },
        # Item 4: VTOL land — ArduPlane auto-transitions back to VTOL on approach
        {
            "label": "VTOL LAND",
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            "cmd":   mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
            "current": 0, "autocontinue": 1,
            "p1": 0, "p2": 0, "p3": 0, "p4": float("nan"),
            "lat": LAND_LAT, "lon": LAND_LON, "alt": LAND_ALT,
        },
    ]


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    print(f"Connecting to {CONNECTION_STRING}...")
    m = mavutil.mavlink_connection(CONNECTION_STRING)
    m.wait_heartbeat(timeout=15)
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")

    wait_for_ekf(m)

    clear_mission(m)
    upload_mission(m, build_mission())

    # ArduPlane won't arm in AUTO — arm in STABILIZE first
    set_mode(m, "STABILIZE")

    if not arm(m, force=FORCE_ARM):
        sys.exit(1)

    # AUTO executes the uploaded mission
    set_mode(m, "AUTO")
    print("AUTO engaged — takeoff, WP1, WP2, land.")

    monitor_mission(m, timeout=MISSION_TIMEOUT)
    print("Done.")


if __name__ == "__main__":
    main()
