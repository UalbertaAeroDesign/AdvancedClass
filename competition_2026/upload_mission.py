"""
upload_mission.py

Uploads the competition mission to the FC over a serial connection.

Mission:
  WP1: NAV_WAYPOINT     — cruise to DLZ at 40 m
  WP2: NAV_VTOL_LAND    — transition + vertical land on DLZ
  WP3: NAV_DELAY        — wait 3 s on ground
  WP4: DO_SET_SERVO     — release payload (ch7 → 1800 µs)

Usage (on the RPi, connected to H743 via UART):
  python upload_mission.py

Or over USB from a laptop:
  python upload_mission.py --port /dev/ttyACM0 --baud 115200
"""

import argparse
import time
from pymavlink import mavutil, mavwp

# ==============================================================================
# MISSION DEFINITION
# ==============================================================================
DLZ_LAT = 32.6101089
DLZ_LON = -97.4840494
CRUISE_ALT = 40  # metres (relative to home)

def build_mission():
    """Returns a list of MAVLink mission items."""
    wp = mavwp.MAVWPLoader()

    # WP0: HOME — ignored on upload, FC sets this at arming location
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        1, 1,  # current=1 (home), autocontinue
        0, 0, 0, 0,
        DLZ_LAT, DLZ_LON, 0
    ))

    # WP1: NAV_WAYPOINT — cruise to DLZ at CRUISE_ALT
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        0, 0, 0, 0,
        DLZ_LAT, DLZ_LON, CRUISE_ALT
    ))

    # WP2: NAV_VTOL_LAND — transition and land on DLZ
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 2,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
        0, 1,
        0, 0, 0, 0,
        DLZ_LAT, DLZ_LON, 0
    ))

    # WP3: NAV_DELAY — sit on ground for 3 seconds
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 3,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_DELAY,
        0, 1,
        3, -1, -1, 0,  # param1=3s, param2/3=-1 (unused)
        0, 0, 0
    ))

    # WP4: DO_SET_SERVO — release payload (servo 7 → 1800 µs)
    wp.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, 4,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, 1,
        7, 1800, 0, 0,  # param1=servo 7, param2=1800 PWM
        0, 0, 0
    ))

    return wp


# ==============================================================================
# UPLOAD
# ==============================================================================
def upload_mission(port, baud):
    print(f"Connecting to {port} at {baud} baud...")
    m = mavutil.mavlink_connection(port, baud=baud)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")

    wp = build_mission()
    count = wp.count()
    print(f"\nUploading {count} mission items...")

    # Send mission count
    m.mav.mission_count_send(m.target_system, m.target_component, count)

    # FC requests each item by sequence number
    for i in range(count):
        msg = m.recv_match(type="MISSION_REQUEST", blocking=True, timeout=5.0)
        if msg is None:
            print(f"  ERROR: no MISSION_REQUEST received for item {i}")
            return False
        item = wp.wp(msg.seq)
        item.target_system = m.target_system
        item.target_component = m.target_component
        m.mav.send(item)
        label = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "NAV_WAYPOINT",
            mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND: "NAV_VTOL_LAND",
            mavutil.mavlink.MAV_CMD_NAV_DELAY: "NAV_DELAY",
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO: "DO_SET_SERVO",
        }.get(item.command, str(item.command))
        print(f"  sent WP{msg.seq}: {label}")

    # Wait for ACK
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5.0)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print(f"\nMission uploaded successfully ({count} items)")
    else:
        result = ack.type if ack else "no response"
        print(f"\nERROR: mission upload failed (ack={result})")
        return False

    # Verify by reading back
    print("\nVerifying — reading mission back from FC...")
    m.mav.mission_request_list_send(m.target_system, m.target_component)
    msg = m.recv_match(type="MISSION_COUNT", blocking=True, timeout=5.0)
    if msg:
        print(f"  FC reports {msg.count} mission items ✓")
    else:
        print("  WARNING: could not read back mission count")

    # Set current waypoint to 1 (skip HOME)
    m.mav.mission_set_current_send(m.target_system, m.target_component, 1)
    time.sleep(0.5)
    print("  Current WP set to 1 ✓")

    print("\nDone. Mission is ready.")
    print(f"  WP1: Cruise to DLZ ({DLZ_LAT}, {DLZ_LON}) at {CRUISE_ALT} m")
    print(f"  WP2: VTOL land on DLZ")
    print(f"  WP3: Wait 3 s")
    print(f"  WP4: Release payload (servo 7 → 1800 µs)")
    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Upload competition mission to FC")
    parser.add_argument("--port", default="/dev/ttyAMA0",
                        help="Serial port (default: /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=921600,
                        help="Baud rate (default: 921600)")
    args = parser.parse_args()
    upload_mission(args.port, args.baud)
