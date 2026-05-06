"""
manual_takeoff_then_auto.py  (ArduCopter / quadcopter version)

Simulates the real pilot workflow in SITL/Gazebo for a multirotor:
  1. Arm the copter in LOITER
  2. RC-override throttle above center to climb like a pilot would
  3. Pitch forward via RC override to move straight ahead
  4. Release RC overrides and switch to AUTO
  5. Let the pre-uploaded mission fly itself from there

All pilot inputs are sent via RC_CHANNELS_OVERRIDE so the FC treats them
exactly like transmitter stick movements. Mirrors the fixed-wing version
in gazebo_simulation/tricopter/manual_takeoff_then_auto.py, but uses
ArduCopter's mode numbers and multirotor-style takeoff.

Why LOITER for the manual phase:
  - Throttle: center = altitude hold, above = climb, below = descend
  - Pitch/roll: velocity control, not attitude — no drift, easy to test
  - GPS-aided position hold when sticks are centered
  - Works the same in SITL (perfect simulated GPS) as on a real copter

Prerequisites:
  - SITL running (sim_vehicle.py -v ArduCopter -f quad --console --map)
  - Mission already uploaded via Mission Planner or MAVProxy
  - Simulated GPS has a 3D fix (default in SITL — wait ~15 s after launch)

Usage:
  python manual_takeoff_then_auto.py
"""

import time
import sys
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
CONNECTION_STR   = "udp:127.0.0.1:14550"   # SITL default MAVLink output

# ArduCopter custom_mode values
MODE_STABILIZE   = 0
MODE_ALT_HOLD    = 2
MODE_AUTO        = 3
MODE_GUIDED      = 4
MODE_LOITER      = 5
MODE_RTL         = 6
MODE_LAND        = 9

# RC channel PWM values (1000=min, 1500=center, 2000=max)
# ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw
PWM_RELEASE      = 0        # 0 = release override
PWM_CENTER       = 1500

# Takeoff phase (LOITER: throttle above center = climb)
TAKEOFF_THROTTLE = 1800     # ~300 µs above center → steady climb
TAKEOFF_PITCH    = PWM_CENTER
TAKEOFF_ROLL     = PWM_CENTER
TAKEOFF_YAW      = PWM_CENTER

# Cruise phase (LOITER: throttle center = alt hold, pitch forward = fly forward)
CRUISE_THROTTLE  = PWM_CENTER   # hold altitude
CRUISE_PITCH     = 1350          # below center = nose down = fly forward
CRUISE_ROLL      = PWM_CENTER
CRUISE_YAW       = PWM_CENTER

# Phase durations / thresholds
TAKEOFF_TARGET_ALT_M = 10.0     # Switch from climb → forward motion at this alt
CRUISE_HOLD_S        = 40.0      # Fly forward this long before handing to AUTO
OVERRIDE_RATE_HZ     = 10       # How often to resend RC overrides


# ==============================================================================
# HELPERS
# ==============================================================================
def connect():
    print(f"Connecting to {CONNECTION_STR}...")
    m = mavutil.mavlink_connection(CONNECTION_STR)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")
    return m

def set_mode(m, mode_num, name):
    print(f"  → Setting mode: {name}")
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_num,
    )
    t0 = time.time()
    while time.time() - t0 < 5.0:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and msg.custom_mode == mode_num:
            print(f"    mode confirmed: {name}")
            return True
    print(f"    WARNING: mode change to {name} not confirmed within 5s")
    return False

def wait_for_gps(m, timeout=30.0):
    print("  → Waiting for GPS 3D fix...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= 6:
            print(f"    GPS OK (sats={msg.satellites_visible}, fix={msg.fix_type})")
            return True
    print("    WARNING: GPS fix not acquired — arming in LOITER may fail")
    return False

def arm(m):
    print("  → Arming...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    t0 = time.time()
    while time.time() - t0 < 5.0:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("    armed")
            return True
    print("    WARNING: arming not confirmed — check pre-arm messages")
    return False

def send_rc(m, roll=PWM_CENTER, pitch=PWM_CENTER, throttle=1000, yaw=PWM_CENTER):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0,
    )

def release_rc(m):
    print("  → Releasing RC overrides")
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        PWM_RELEASE, PWM_RELEASE, PWM_RELEASE, PWM_RELEASE,
        PWM_RELEASE, PWM_RELEASE, PWM_RELEASE, PWM_RELEASE,
    )

def get_alt(m):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    m = connect()

    wait_for_gps(m)

    print("\n[1/5] Switching to LOITER and arming...")
    if not set_mode(m, MODE_LOITER, "LOITER"):
        sys.exit(1)
    if not arm(m):
        sys.exit(1)

    dt = 1.0 / OVERRIDE_RATE_HZ

    print(f"\n[2/5] Takeoff — throttle above center until alt > {TAKEOFF_TARGET_ALT_M} m")
    t0 = time.time()
    while True:
        send_rc(m,
                roll     = TAKEOFF_ROLL,
                pitch    = TAKEOFF_PITCH,
                throttle = TAKEOFF_THROTTLE,
                yaw      = TAKEOFF_YAW)
        alt = get_alt(m)
        if alt is not None and alt >= TAKEOFF_TARGET_ALT_M:
            print(f"    alt = {alt:.1f} m — takeoff altitude reached")
            break
        if time.time() - t0 > 45.0:
            print("    WARNING: did not reach takeoff altitude within 45s")
            break
        if alt is not None and int((time.time() - t0) * 2) % 2 == 0:
            print(f"    climbing... alt = {alt:.1f} m")
        time.sleep(dt)

    print(f"\n[3/5] Cruise — pitch forward, hold altitude for {CRUISE_HOLD_S} s")
    t0 = time.time()
    while time.time() - t0 < CRUISE_HOLD_S:
        send_rc(m,
                roll     = CRUISE_ROLL,
                pitch    = CRUISE_PITCH,
                throttle = CRUISE_THROTTLE,
                yaw      = CRUISE_YAW)
        time.sleep(dt)

    print("\n[4/5] Releasing RC overrides and handing off to AUTO")
    release_rc(m)
    time.sleep(0.3)

    if not set_mode(m, MODE_AUTO, "AUTO"):
        print("    ERROR: AUTO mode not engaged — pilot should take over")
        sys.exit(1)

    print("\n[5/5] Mission running. Monitoring waypoint progress (Ctrl+C to stop)...")
    last_wp = -1
    try:
        while True:
            msg = m.recv_match(type=["MISSION_CURRENT", "STATUSTEXT", "HEARTBEAT"],
                               blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_CURRENT":
                if msg.seq != last_wp:
                    print(f"    current WP: {msg.seq}")
                    last_wp = msg.seq
            elif msg.get_type() == "STATUSTEXT":
                print(f"    FC: {msg.text}")
            elif msg.get_type() == "HEARTBEAT":
                if msg.custom_mode != MODE_AUTO:
                    print(f"    mode changed away from AUTO (custom_mode={msg.custom_mode}) — exiting")
                    break
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
