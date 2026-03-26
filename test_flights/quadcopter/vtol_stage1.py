#!/usr/bin/env python3
"""
vtol_stage1.py  —  FW takeoff → MC transition → VTOL land

Run this first. After the vehicle lands and disarms, run vtol_stage2.py.

AUTO mission (uploaded after FW TAKEOFF climb):
  [0]  NAV_WAYPOINT          FW cruise / transition point
  [1]  DO_VTOL_TRANSITION MC begin MC transition (fires when [0] reached)
  [2]  NAV_VTOL_LAND         land 120 m past [0] along outbound heading

The FW cruise WP doubles as the firmware approach reference.
ArduPlane requires >= 117 m between the last NAV WP and NAV_VTOL_LAND;
the 120 m gap satisfies this while keeping the land point close to the
transition spot.
"""

import argparse, math, sys, time
from dataclasses import dataclass
from typing import Dict, List, Optional
from pymavlink import mavutil

# =====================================================================
# Config
#
#   AUTO entry ──80 m──▶ FW cruise WP / transition ──120 m──▶ land
#   total one-way footprint: 200 m from AUTO entry
# =====================================================================
TKOFF_ALT_M           = 25.0
FW_CRUISE_DIST_M      = 20.0    # FW cruise leg; vehicle already at speed from TAKEOFF
FW_CRUISE_ALT_M       = 25.0
VTOL_APPROACH_MIN_M   = 120.0   # FW cruise WP → land point; must be >= 117 m (firmware min)
LAND_ACCURACY_M       = 35.0
ARM_TIMEOUT_S         = 25
TAKEOFF_TIMEOUT_S     = 120
GENERAL_TIMEOUT_S     = 600

@dataclass
class GeoPoint:
    lat: float; lon: float; alt: float

# =====================================================================
# Geometry
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

# =====================================================================
# MAVLink helpers  (shared with stage 2)
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
        if mt == "STATUSTEXT" and "STATUSTEXT" not in types:
            print(f"  FC: {msg.text.strip()}"); continue
        if mt in types: return msg
    raise TimeoutError(f"Timeout waiting {types}")

def wait_gps(master):
    print("Waiting for GPS fix...")
    t0 = time.time()
    while time.time()-t0 < 60:
        msg = master.recv_match(type=["GPS_RAW_INT","GLOBAL_POSITION_INT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="GPS_RAW_INT" and getattr(msg,"fix_type",0)>=3: print("GPS OK"); return
        if msg.get_type()=="GLOBAL_POSITION_INT": print("GPS OK"); return
    raise TimeoutError("No GPS")

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

def set_param(master, pid, val, timeout=5):
    master.mav.param_set_send(master.target_system, master.target_component,
        pid.encode(), float(val), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.rstrip("\x00")==pid:
            print(f"  {pid} = {msg.param_value}"); return
    print(f"  Warning: no confirm for {pid}")

def set_current(master, seq=0):
    drain(master, 0.3); master.waypoint_set_current_send(seq)
    t0 = time.time()
    while time.time()-t0 < 5:
        msg = master.recv_match(type=["MISSION_CURRENT","STATUSTEXT"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type()=="STATUSTEXT": print(f"  FC: {msg.text.strip()}"); continue
        if msg.seq==seq: return
    print("  Warning: mission item unconfirmed")

def print_mission(items):
    CMD_NAMES = {16: "NAV_WAYPOINT", 21: "NAV_VTOL_LAND", 85: "NAV_VTOL_TAKEOFF",
                 3000: "DO_VTOL_TRANSITION", 20: "NAV_RTL", 179: "DO_SET_HOME"}
    for i, it in enumerate(items):
        name = CMD_NAMES.get(it["command"], str(it["command"]))
        if it["lat"] != 0 or it["lon"] != 0:
            print(f"  [{i}] {name:<22s} lat={it['lat']:.7f} lon={it['lon']:.7f} alt={it['alt']}")
        else:
            print(f"  [{i}] {name}")

# =====================================================================
# Mission upload
# =====================================================================
def _nav(cmd, lat=0., lon=0., alt=0., p1=0, p2=0, p3=0, p4=0,
         frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT):
    return dict(frame=frame, command=cmd, p1=p1,p2=p2,p3=p3,p4=p4, lat=lat,lon=lon,alt=alt)

def _do(cmd, p1=0, p2=0, p3=0, p4=0, p5=0., p6=0., p7=0.,
        frame=mavutil.mavlink.MAV_FRAME_MISSION):
    return dict(frame=frame, command=cmd, p1=p1,p2=p2,p3=p3,p4=p4, lat=p5,lon=p6,alt=p7)

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
# Stage 1 mission
# =====================================================================
def make_stage1(current_pos: GeoPoint, heading_deg: float):
    """
    3-item AUTO mission along current heading.  The FW cruise WP is the
    last NAV item before NAV_VTOL_LAND, so it doubles as the firmware
    approach reference (>= 117 m to land point).

      AUTO entry ──FW_CRUISE_DIST_M──▶ [0] cruise/transition
                                        [1] DO_VTOL_TRANSITION MC
                   ──VTOL_APPROACH_MIN_M──▶ [2] land
    """
    M = mavutil.mavlink

    # FW cruise WP — vehicle continues in FW from TAKEOFF until here
    fw_lat, fw_lon = destination_from_bearing(
        current_pos.lat, current_pos.lon, heading_deg, FW_CRUISE_DIST_M)

    # Land point — 120 m past cruise WP along same heading.
    # The vehicle decelerates in MC across this 120 m approach leg.
    land_lat, land_lon = destination_from_bearing(
        fw_lat, fw_lon, heading_deg, VTOL_APPROACH_MIN_M)

    items = [
        # [0] FW cruise WP (also serves as approach reference for VTOL land)
        _nav(M.MAV_CMD_NAV_WAYPOINT,
             lat=fw_lat, lon=fw_lon, alt=FW_CRUISE_ALT_M),
        # [1] Transition to MC — fires when [0] is reached
        _do(M.MAV_CMD_DO_VTOL_TRANSITION, p1=M.MAV_VTOL_STATE_MC),
        # [2] VTOL land — 120 m from [0], satisfies firmware >= 117 m minimum
        _nav(M.MAV_CMD_NAV_VTOL_LAND,
             lat=land_lat, lon=land_lon, alt=0.0),
    ]
    return items, GeoPoint(land_lat, land_lon, 0.0)

# =====================================================================
# FW takeoff
# =====================================================================
def get_pos_and_heading(master, timeout=5):
    """Return current position and ground-track heading."""
    pos = None; hdg = None
    t0 = time.time()
    while time.time()-t0 < timeout:
        msg = master.recv_match(
            type=["GLOBAL_POSITION_INT", "VFR_HUD"], blocking=True, timeout=1)
        if not msg: continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            pos = GeoPoint(msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000)
            hdg = msg.hdg / 100.0 if hasattr(msg, "hdg") and msg.hdg != 65535 else hdg
        elif msg.get_type() == "VFR_HUD":
            hdg = msg.heading
        if pos is not None and hdg is not None:
            return pos, hdg
    if pos and hdg is None:
        return pos, 0.0
    raise TimeoutError("Could not get position/heading")


def fw_climb(master, target, timeout=TAKEOFF_TIMEOUT_S):
    print(f"Climbing to {target:.0f} m AGL...")
    t0 = time.time(); last_alt_print = -15
    while time.time()-t0 < timeout:
        msg = master.recv_match(type=["GLOBAL_POSITION_INT","STATUSTEXT","HEARTBEAT"],
                                blocking=True, timeout=1)
        if not msg: continue
        mt = msg.get_type()
        if mt=="STATUSTEXT": print(f"  FC: {msg.text.strip()}")
        elif mt=="GLOBAL_POSITION_INT":
            alt = msg.relative_alt/1000
            if alt - last_alt_print >= 15:
                print(f"  alt {alt:.0f} m")
                last_alt_print = alt
            if alt >= target*0.90: print(f"  alt {alt:.0f} m — cruise alt reached"); return
        elif mt=="HEARTBEAT":
            if not is_armed(msg): raise RuntimeError("Unexpected disarm during climb")
    raise TimeoutError("Climb timeout")

# =====================================================================
# Monitor stage 1
# =====================================================================
def monitor_stage1(master, land_point: GeoPoint):
    M = mavutil.mavlink
    print("Monitoring AUTO mission...")
    t0 = time.time(); last_seq = -1; last_pos = None
    seen_mc = False; touchdown_checked = False
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
                dist = ground_dist(last_pos.lat, last_pos.lon, land_point.lat, land_point.lon)
                print(f"  dist={dist:.0f} m  alt={last_pos.alt:.0f} m  seq={last_seq}")
                last_print_t = now

        elif mt=="MISSION_CURRENT":
            if msg.seq != last_seq:
                last_seq = msg.seq
                labels = {0: "FW cruise", 1: "VTOL transition", 2: "VTOL land"}
                print(f"  waypoint {last_seq}  ({labels.get(last_seq, '?')})")

        elif mt=="EXTENDED_SYS_STATE":
            if msg.vtol_state == M.MAV_VTOL_STATE_MC and not seen_mc:
                seen_mc = True; print("  MC mode confirmed")
            if (not touchdown_checked and last_seq >= 2
                    and msg.landed_state == M.MAV_LANDED_STATE_ON_GROUND
                    and last_pos):
                touchdown_checked = True
                dist = ground_dist(last_pos.lat, last_pos.lon,
                                   land_point.lat, land_point.lon)
                print(f"  TOUCHDOWN  {dist:.1f} m from target")
                if dist > LAND_ACCURACY_M:
                    raise RuntimeError(f"Landed too far: {dist:.1f} m")

        elif mt=="STATUSTEXT":
            print(f"  FC: {msg.text.strip()}")

        elif mt=="HEARTBEAT":
            if not is_armed(msg):
                print("Disarmed — stage 1 complete")
                if not seen_mc:
                    raise RuntimeError("MC transition never observed")
                if last_seq < 2:
                    raise RuntimeError(f"Mission stalled at seq={last_seq}")
                return GeoPoint(last_pos.lat, last_pos.lon, last_pos.alt) if last_pos else None

    raise TimeoutError("Stage 1 timeout")

# =====================================================================
# Main
# =====================================================================
def main():
    parser = argparse.ArgumentParser(description="Stage 1: FW takeoff → VTOL land")
    parser.add_argument("--connect", default="udpin:0.0.0.0:14551")
    args = parser.parse_args()

    print("=== Stage 1: FW takeoff → MC transition → VTOL land ===")
    master = mavutil.mavlink_connection(args.connect)
    wait_hb(master); wait_gps(master); prep_streams(master)

    home = get_home(master)
    print(f"Home: {home.lat:.7f}, {home.lon:.7f}  alt={home.alt:.1f} m")

    print("Setting parameters...")
    set_param(master, "TKOFF_ALT", TKOFF_ALT_M)
    set_param(master, "THR_FAILSAFE", 0)
    # FIXME: THR_MAX, TRIM_THROTTLE, TRIM_ARSPD_CM, ARSPD_FBW_MAX are tuned
    # for SITL sim speeds. Readjust these for the real balsa model before flight.
    set_param(master, "THR_MAX", 30)
    set_param(master, "TRIM_THROTTLE", 20)
    set_param(master, "TRIM_ARSPD_CM", 900)
    set_param(master, "ARSPD_FBW_MAX", 12)

    set_mode(master, "TAKEOFF")
    arm(master)
    fw_climb(master, TKOFF_ALT_M)

    current_pos, current_hdg = get_pos_and_heading(master)
    print(f"Position at cruise: {current_pos.lat:.7f}, {current_pos.lon:.7f}  hdg={current_hdg:.1f}°")

    items, land_point = make_stage1(current_pos, current_hdg)
    total_dist = ground_dist(current_pos.lat, current_pos.lon, land_point.lat, land_point.lon)
    print(f"\nMission plan  (total {total_dist:.0f} m along {current_hdg:.0f}°):")
    print_mission(items)
    print(f"Land target: {land_point.lat:.7f}, {land_point.lon:.7f}\n")

    upload(master, items)
    set_current(master, 0)
    set_mode(master, "AUTO")
    monitor_stage1(master, land_point)
    print("\nStage 1 done. Run vtol_stage2.py when ready.")

if __name__ == "__main__":
    try: main()
    except Exception as e: print(f"\nERROR: {e}"); sys.exit(1)
