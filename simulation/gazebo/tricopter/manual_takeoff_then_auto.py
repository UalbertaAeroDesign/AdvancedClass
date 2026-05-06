"""
manual_takeoff_then_auto.py

Simulates the real pilot workflow in SITL/Gazebo:
  1. Arm the plane in FBWA
  2. RC-override throttle + elevator to take off like a pilot would
  3. Hold forward cruise for a few seconds
  4. Release RC overrides and switch to AUTO
  5. Let the pre-uploaded mission fly itself from there

All pilot inputs are sent via RC_CHANNELS_OVERRIDE so the FC treats them
exactly like transmitter stick movements. This is the same path a real
radio takes, so behavior here predicts real-hardware behavior.

Prerequisites:
  - SITL running (sim_vehicle.py -v ArduPlane -f quadplane --console --map)
  - Mission already uploaded via Mission Planner or MAVProxy
  - Nothing else sending RC overrides on the same channels

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

# ArduPlane custom_mode values
MODE_MANUAL      = 0
MODE_FBWA        = 5
MODE_AUTO        = 10

# RC channel PWM values (1000=min, 1500=center, 2000=max)
# ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw
PWM_RELEASE      = 0        # 0 = release override (let radio / FBWA defaults take over)
PWM_CENTER       = 1500

# Takeoff phase (full throttle, slight pull-back in FBWA → climb)
TAKEOFF_THROTTLE = 2000
TAKEOFF_PITCH    = 1400     # in FBWA, below center = pull back = pitch up

# Cruise phase (hold steady forward flight)
CRUISE_THROTTLE  = 1750
CRUISE_PITCH     = PWM_CENTER
CRUISE_ROLL      = PWM_CENTER
CRUISE_YAW       = PWM_CENTER

# Phase durations / thresholds
TAKEOFF_TARGET_ALT_M = 30.0     # Switch from takeoff → cruise at this altitude (relative)
CRUISE_HOLD_S        = 5.0      # Hold cruise this long before handing to AUTO
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
    # Wait for FC to report the mode change
    t0 = time.time()
    while time.time() - t0 < 5.0:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and msg.custom_mode == mode_num:
            print(f"    mode confirmed: {name}")
            return True
    print(f"    WARNING: mode change to {name} not confirmed within 5s")
    return False

def arm(m):
    print("  → Arming...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    # Wait for armed state
    t0 = time.time()
    while time.time() - t0 < 5.0:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("    armed")
            return True
    print("    WARNING: arming not confirmed")
    return False

def send_rc(m, roll=PWM_CENTER, pitch=PWM_CENTER, throttle=1000, yaw=PWM_CENTER):
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component,
        roll, pitch, throttle, yaw,
        0, 0, 0, 0,   # ch5-8 untouched
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
    return msg.relative_alt / 1000.0  # mm → m


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    m = connect()

    print("\n[1/5] Switching to FBWA and arming...")
    if not set_mode(m, MODE_FBWA, "FBWA"):
        sys.exit(1)
    if not arm(m):
        sys.exit(1)

    dt = 1.0 / OVERRIDE_RATE_HZ

    print(f"\n[2/5] Takeoff — full throttle, pitch up, until alt > {TAKEOFF_TARGET_ALT_M} m")
    t0 = time.time()
    while True:
        send_rc(m,
                roll     = PWM_CENTER,
                pitch    = TAKEOFF_PITCH,
                throttle = TAKEOFF_THROTTLE,
                yaw      = PWM_CENTER)
        alt = get_alt(m)
        if alt is not None and alt >= TAKEOFF_TARGET_ALT_M:
            print(f"    alt = {alt:.1f} m — takeoff altitude reached")
            break
        if time.time() - t0 > 60.0:
            print("    WARNING: did not reach takeoff altitude within 60s")
            break
        if alt is not None and int((time.time() - t0) * 2) % 2 == 0:
            print(f"    climbing... alt = {alt:.1f} m")
        time.sleep(dt)

    print(f"\n[3/5] Cruise — holding forward flight for {CRUISE_HOLD_S} s")
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
    time.sleep(0.3)  # give the FC a moment to see neutral sticks

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
                # Exit cleanly if pilot flips out of AUTO
                if msg.custom_mode != MODE_AUTO:
                    print(f"    mode changed away from AUTO (custom_mode={msg.custom_mode}) — exiting")
                    break
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
