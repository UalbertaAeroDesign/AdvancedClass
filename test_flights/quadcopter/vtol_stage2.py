#!/usr/bin/env python3
"""
vtol_stage2.py  —  VTOL re-takeoff → FW return → FW runway landing

Run this after vtol_stage1.py has landed and disarmed.

The "PreArm: In landing sequence" lock is cleared by uploading a fresh
mission before arming.  The VTOL re-takeoff is performed externally in
GUIDED mode (MAV_CMD_NAV_VTOL_TAKEOFF via command_long); once at cruise
altitude the script switches to AUTO to execute the return mission.

Mission (uploaded to flight controller):
  [0]  DO_VTOL_TRANSITION FW   convert to FW immediately on AUTO entry
  [1]  NAV_WAYPOINT            FW leg toward original home (builds airspeed)
  [2]  DO_SET_HOME             restore original home before landing
  [3]  DO_LAND_START           marks beginning of landing sequence
  [4]  NAV_WAYPOINT            FW approach waypoint (lined up with runway)
  [5]  NAV_LAND                FW touchdown on runway
"""

import argparse, math, sys, time
from dataclasses import dataclass
from typing import Dict, List, Optional
from pymavlink import mavutil

@dataclass
class GeoPoint:
    lat: float; lon: float; alt: float

# =====================================================================
# Config  (must match stage 1 values)
# =====================================================================
VTOL_RETAKEOFF_ALT  = 25.0    # AGL — must match FW_RETURN_ALT_M so vehicle
                                # is already at cruise alt before FW transition
FW_RETURN_LEG_M     = 50.0    # FW leg toward home after transition
FW_RETURN_ALT_M     = 15.0
ARM_TIMEOUT_S       = 25
GENERAL_TIMEOUT_S   = 600

# sim_vehicle.py --custom-location=32.609354,-97.484479,216,0
# Desired stop point (south end of runway)
RUNWAY_HOME = GeoPoint(32.609354, -97.484479, 216.0)
# FIXME: Touchdown and approach positions are tuned for SITL rollout.
# The sim overshoots heavily — touchdown is placed ~250 m north of the
# desired stop point (south end of runway) to compensate.
# Readjust for the real model before flight.
RUNWAY_TOUCHDOWN = GeoPoint(32.611600, -97.484430, 216.0)
# FW approach start — must be >= 117 m north of touchdown
RUNWAY_APPROACH = GeoPoint(32.612750, -97.484380, 216.0)

# =====================================================================
# Geometry  (duplicated from stage 1 for standalone operation)
# =====================================================================
def wrap_360(d): return d % 360.0

def destination_from_bearing(lat, lon, bearing, dist):
    R = 6_378_137.0
    lat1, lon1, b = math.radians(lat), math.radians(lon), math.radians(bearing)
    d = dist / R
    lat2 = math.asin(math.sin(lat1)*math.cos(d) + math.cos(lat1)*math.sin(d)*math.cos(b))
    lon2 = lon1 + math.atan2(math.sin(b)*math.sin(d)*math.cos(lat1),
                              math.cos(d) - math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)

def offset_ne(lat, lon, n, e):
    R = 6_378_137.0
    return lat + math.degrees(n/R), lon + math.degrees(e/(R*math.cos(math.radians(lat))))

def ground_dist(a_lat, a_lon, b_lat, b_lon):
    R = 6_378_137.0
    lat1, lat2 = math.radians(a_lat), math.radians(b_lat)
    x = math.radians(b_lon-a_lon)*math.cos((lat1+lat2)/2)
    return math.sqrt(x*x + (lat2-lat1)**2) * R

def bearing_to(a_lat, a_lon, b_lat, b_lon):
    dlon = math.radians(b_lon-a_lon)
    la1, la2 = math.radians(a_lat), math.radians(b_lat)
    y = math.sin(dlon)*math.cos(la2)
    x = math.cos(la1)*math.sin(la2) - math.sin(la1)*math.cos(la2)*math.cos(dlon)
    return wrap_360(math.degrees(math.atan2(y, x)))

# =====================================================================
# MAVLink helpers  (same as stage 1)
# =====================================================================
def drain(master, dur=0.4):
    t = time.time()+dur
    while time.time() < t:
        master.recv_match(blocking=False)
        time.sleep(0.01)

def wait_hb(master):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Connected  sys={master.target_system}")

def req_interval(master, msg_id, hz):
    master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6/hz), 0,0,0,0,0)

def prep_streams(master):
    req_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 4.0)
    req_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,  2.0)
    req_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,      2.0)

def wait_msg(master, types, timeout):
    if isinstance(types, str): types = [types]
    watch = list(set(types+["STATUSTEXT"]))
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(type=watch, blocking=True, timeout=1)
        if not msg: continue
        mt = msg.get_type()
        if mt=="STATUSTEXT" and "STATUSTEXT" not in types:
            print(f"  FC: {msg.text.strip()}"); continue
        if mt in types: return msg
    raise TimeoutError(f"Timeout waiting {types}")

def get_home(master):
    master.mav.command_long_send(master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, 0,0,0,0,0,0,0,0)
    t0 = time.time()
    while time.time()-t0 < 20:
        msg = master.recv_match(type=["HOME_POSITION","GLOBAL_POSITION_INT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="HOME_POSITION":
            return GeoPoint(msg.latitude/1e7, msg.longitude/1e7, msg.altitude/1000)
        if msg.get_type()=="GLOBAL_POSITION_INT":
            return GeoPoint(msg.lat/1e7, msg.lon/1e7, msg.alt/1000)
    raise TimeoutError("No home")

def get_position(master):
    t0 = time.time()
    while time.time()-t0 < 10:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg: return GeoPoint(msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000)
    raise TimeoutError("No position")

def is_armed(hb): return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def set_mode(master, name, timeout=12):
    mm = master.mode_mapping()
    if name not in mm: raise RuntimeError(f"Mode {name} unknown")
    drain(master, 0.3); master.set_mode(mm[name])
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(type=["HEARTBEAT","STATUSTEXT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="STATUSTEXT": print(f"  FC: {msg.text.strip()}"); continue
        if mavutil.mode_string_v10(msg)==name: print(f"Mode → {name}"); return
    raise TimeoutError(f"Mode {name} failed")

def arm(master, timeout=ARM_TIMEOUT_S):
    drain(master, 0.3); print("Arming..."); master.arducopter_arm()
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(type=["HEARTBEAT","STATUSTEXT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="STATUSTEXT": print(f"  FC: {msg.text.strip()}"); continue
        if is_armed(msg): print("Armed"); return
    raise TimeoutError("Arm timeout")

def set_current(master, seq=0):
    drain(master, 0.3); master.waypoint_set_current_send(seq)
    t0 = time.time()
    while time.time()-t0 < 5:
        msg = master.recv_match(type=["MISSION_CURRENT","STATUSTEXT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="STATUSTEXT": print(f"  FC: {msg.text.strip()}"); continue
        if msg.seq==seq: return
    print("  Warning: mission item unconfirmed")

def set_param(master, pid, val, timeout=5):
    master.mav.param_set_send(master.target_system, master.target_component,
        pid.encode(), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.rstrip("\x00")==pid:
            print(f"  {pid} = {msg.param_value}"); return
    print(f"  Warning: no confirm for {pid}")

def print_mission(items):
    CMD_NAMES = {16: "NAV_WAYPOINT", 21: "NAV_LAND", 85: "NAV_VTOL_TAKEOFF",
                 3000: "DO_VTOL_TRANSITION", 20: "NAV_RTL", 179: "DO_SET_HOME",
                 189: "DO_LAND_START"}
    for i, it in enumerate(items):
        name = CMD_NAMES.get(it["command"], str(it["command"]))
        if it["lat"] != 0 or it["lon"] != 0:
            print(f"  [{i}] {name:<22s} lat={it['lat']:.7f} lon={it['lon']:.7f} alt={it['alt']}")
        else:
            print(f"  [{i}] {name}")

def upload(master, items, timeout=20):
    M = mavutil.mavlink; n = len(items)
    print(f"Uploading {n} mission items...")
    drain(master, 0.5); master.waypoint_clear_all_send()
    time.sleep(0.3); drain(master, 0.3)
    master.mav.mission_count_send(master.target_system, master.target_component,
                                   n, M.MAV_MISSION_TYPE_MISSION)
    sent = set(); t0 = time.time()
    while len(sent) < n:
        if time.time()-t0 > timeout: raise TimeoutError("Upload timeout")
        msg = master.recv_match(type=["MISSION_REQUEST_INT","MISSION_REQUEST","STATUSTEXT"],
                                blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="STATUSTEXT": print(f"  FC: {msg.text.strip()}"); continue
        seq = msg.seq
        if seq<0 or seq>=n: raise RuntimeError(f"Invalid seq {seq}")
        it = items[seq]
        master.mav.mission_item_int_send(
            master.target_system, master.target_component, seq,
            it["frame"], it["command"], 1 if seq==0 else 0, 1,
            float(it["p1"]), float(it["p2"]), float(it["p3"]), float(it["p4"]),
            int(it["lat"]*1e7), int(it["lon"]*1e7), float(it["alt"]),
            M.MAV_MISSION_TYPE_MISSION)
        sent.add(seq)
    ack = wait_msg(master, "MISSION_ACK", timeout)
    if ack.type != M.MAV_MISSION_ACCEPTED:
        raise RuntimeError(f"Upload rejected: ack={ack.type}")
    print("Upload accepted")

# =====================================================================
# Stage 2 mission
# =====================================================================
def make_stage2(original_home: GeoPoint, land_pos: GeoPoint,
                approach_pt: GeoPoint = RUNWAY_APPROACH):
    """
    Build the AUTO mission for the return leg.
    Takeoff is handled externally in GUIDED; the mission starts with
    DO_VTOL_TRANSITION FW so the vehicle converts immediately on AUTO entry.

    The vehicle flies directly to the far end of the runway (approach WP),
    then descends along the runway centerline to touch down at home.
    """
    M = mavutil.mavlink

    items = [
        # [0] Transition to FW
        dict(frame=M.MAV_FRAME_MISSION,
             command=M.MAV_CMD_DO_VTOL_TRANSITION,
             p1=M.MAV_VTOL_STATE_FW, p2=0, p3=0, p4=0,
             lat=0.0, lon=0.0, alt=0.0),

        # [1] Restore original home before landing
        dict(frame=M.MAV_FRAME_MISSION,
             command=M.MAV_CMD_DO_SET_HOME,
             p1=0, p2=0, p3=0, p4=0,
             lat=original_home.lat, lon=original_home.lon, alt=original_home.alt),

        # [2] DO_LAND_START — marks beginning of landing sequence
        dict(frame=M.MAV_FRAME_MISSION,
             command=M.MAV_CMD_DO_LAND_START,
             p1=0, p2=0, p3=0, p4=0,
             lat=0.0, lon=0.0, alt=0.0),

        # [3] Approach WP — fly to far end of runway at altitude
        dict(frame=M.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             command=M.MAV_CMD_NAV_WAYPOINT,
             p1=0, p2=0, p3=0, p4=0,
             lat=approach_pt.lat, lon=approach_pt.lon, alt=FW_RETURN_ALT_M),

        # [4] NAV_LAND — touchdown ~50 m north of home so rollout ends at home
        dict(frame=M.MAV_FRAME_GLOBAL_RELATIVE_ALT,
             command=M.MAV_CMD_NAV_LAND,
             p1=0, p2=0, p3=0, p4=0,
             lat=RUNWAY_TOUCHDOWN.lat, lon=RUNWAY_TOUCHDOWN.lon, alt=0.0),
    ]
    return items


# =====================================================================
# GUIDED VTOL takeoff
# =====================================================================
def guided_vtol_takeoff(master, alt_agl, timeout=90):
    """
    Command VTOL takeoff via command_long in GUIDED mode.
    Blocks until relative altitude >= 90% of alt_agl.
    """
    print(f"VTOL takeoff to {alt_agl:.0f} m AGL...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_agl,
    )
    t0 = time.time(); last_alt_print = -15
    while time.time() - t0 < timeout:
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "STATUSTEXT", "HEARTBEAT"],
            blocking=True, timeout=1)
        if not msg: continue
        mt = msg.get_type()
        if mt == "STATUSTEXT": print(f"  FC: {msg.text.strip()}")
        elif mt == "GLOBAL_POSITION_INT":
            alt = msg.relative_alt / 1000.0
            if alt - last_alt_print >= 15:
                print(f"  alt {alt:.0f} m")
                last_alt_print = alt
            if alt >= alt_agl * 0.90:
                print(f"  alt {alt:.0f} m — target reached")
                return
        elif mt == "HEARTBEAT":
            if not is_armed(msg):
                raise RuntimeError("Unexpected disarm during VTOL takeoff")
    raise TimeoutError(f"Timed out climbing to {alt_agl}m")

# =====================================================================
# Monitor stage 2
# =====================================================================
def monitor_stage2(master, original_home: GeoPoint):
    M = mavutil.mavlink
    print("Monitoring AUTO mission...")
    t0 = time.time(); last_seq = -1; last_pos = None
    seen_fw = False; seen_to_fw = False
    last_print_t = 0

    while time.time()-t0 < GENERAL_TIMEOUT_S:
        msg = master.recv_match(
            type=["MISSION_CURRENT","EXTENDED_SYS_STATE","GLOBAL_POSITION_INT",
                  "STATUSTEXT","HEARTBEAT"], blocking=True, timeout=1)
        if not msg: continue
        mt = msg.get_type()

        if mt=="GLOBAL_POSITION_INT":
            last_pos = GeoPoint(msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000)
            now = time.time()
            if now - last_print_t >= 4:
                dist = ground_dist(last_pos.lat, last_pos.lon, original_home.lat, original_home.lon)
                print(f"  dist_home={dist:.0f} m  alt={last_pos.alt:.0f} m  seq={last_seq}")
                last_print_t = now

        elif mt=="MISSION_CURRENT":
            if msg.seq != last_seq:
                last_seq = msg.seq
                labels = {0: "FW transition", 1: "set home", 2: "land start",
                          3: "FW approach", 4: "FW landing"}
                print(f"  waypoint {last_seq}  ({labels.get(last_seq, '?')})")

        elif mt=="EXTENDED_SYS_STATE":
            if msg.vtol_state==M.MAV_VTOL_STATE_TRANSITION_TO_FW and not seen_to_fw:
                seen_to_fw = True; print("  FW transition started")
            elif msg.vtol_state==M.MAV_VTOL_STATE_FW and not seen_fw:
                seen_fw = True; print("  FW mode confirmed")

        elif mt=="STATUSTEXT":
            print(f"  FC: {msg.text.strip()}")

        elif mt=="HEARTBEAT":
            if not is_armed(msg):
                print("Disarmed — stage 2 complete")
                break

    if not last_pos: raise RuntimeError("No position data")
    dist_home = ground_dist(last_pos.lat, last_pos.lon, original_home.lat, original_home.lon)

    print(f"\n=== Stage 2 result ===")
    print(f"  FW transition : {'OK' if seen_to_fw and seen_fw else 'MISSING'}")
    print(f"  Final seq     : {last_seq}")
    print(f"  Dist to home  : {dist_home:.0f} m")

    problems = []
    if not (seen_to_fw and seen_fw): problems.append("FW transition not observed")
    if last_seq < 4: problems.append(f"Mission stalled at seq={last_seq}")
    if problems: raise RuntimeError("Stage 2 failed:\n  " + "\n  ".join(problems))
    print("PASS")

# =====================================================================
# Main
# =====================================================================
def main():
    parser = argparse.ArgumentParser(description="Stage 2: VTOL re-takeoff → FW RTL")
    parser.add_argument("--connect", default="udpin:0.0.0.0:14551")
    args = parser.parse_args()

    print("=== Stage 2: VTOL takeoff → FW transition → RTL ===")
    master = mavutil.mavlink_connection(args.connect)
    wait_hb(master); prep_streams(master)

    original_home = RUNWAY_HOME
    print(f"Runway home (hardcoded): {original_home.lat:.7f}, {original_home.lon:.7f}  alt={original_home.alt:.1f} m")

    land_pos = get_position(master)
    print(f"Current pos (land site): {land_pos.lat:.7f}, {land_pos.lon:.7f}")

    hdg = bearing_to(land_pos.lat, land_pos.lon, original_home.lat, original_home.lon)
    dist = ground_dist(land_pos.lat, land_pos.lon, original_home.lat, original_home.lon)
    print(f"Bearing to runway: {hdg:.0f}°  dist: {dist:.0f} m")

    items = make_stage2(original_home, land_pos)
    print("\nMission plan:")
    print_mission(items)
    print()

    set_param(master, "THR_FAILSAFE", 0)
    set_param(master, "RTL_AUTOLAND", 2)
    # FIXME: THR_MAX, TRIM_THROTTLE, TRIM_ARSPD_CM, ARSPD_FBW_MAX,
    # LAND_FLARE_SEC, TECS_LAND_ARSPD are tuned for SITL sim speeds.
    # Readjust these for the real balsa model before flight.
    set_param(master, "THR_MAX", 30)
    set_param(master, "TRIM_THROTTLE", 20)
    set_param(master, "TRIM_ARSPD_CM", 900)
    set_param(master, "ARSPD_FBW_MAX", 12)
    set_param(master, "LAND_FLARE_SEC", 6)
    set_param(master, "LAND_FLARE_ALT", 8)
    set_param(master, "TECS_LAND_ARSPD", 5)
    set_param(master, "TECS_LAND_SINK", 0.5)
    # Bit 2 (value 4): Allow FW Land — without this, quadplanes
    # always transition to VTOL for NAV_LAND
    set_param(master, "Q_OPTIONS", 4)

    upload(master, items)
    set_current(master, 0)

    set_mode(master, "GUIDED")
    arm(master)
    guided_vtol_takeoff(master, VTOL_RETAKEOFF_ALT)

    set_mode(master, "AUTO", timeout=30)
    monitor_stage2(master, original_home)

if __name__ == "__main__":
    try: main()
    except Exception as e: print(f"\nERROR: {e}"); sys.exit(1)
