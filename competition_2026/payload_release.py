"""
payload_release.py

Monitors the FC for landing completion, then commands channel 7 to release
the payload. The STM32 in the payload reads channel 7 via pulseIn and moves
its servo to release the locking prongs.

Channel 7 PWM mapping (matches the STM32 firmware):
  900–1500 µs  → servo writes 40°  (LOCKED — prongs engaged)
  1500–2100 µs → servo writes 100° (RELEASED — prongs retracted)

This script uses MAV_CMD_DO_SET_SERVO to set the FC's SERVO7 output directly.
Unlike RC_CHANNELS_OVERRIDE, DO_SET_SERVO holds its value indefinitely (no
timeout, no need to keep resending). This is the same mechanism as putting a
DO_SET_SERVO command in a mission.

Requirements:
  - SERVO7_FUNCTION = 0 (Disabled / manual) so DO_SET_SERVO can control it
    If SERVO7_FUNCTION is set to RCPassThru (1), DO_SET_SERVO won't override it.
    Verify with: param show SERVO7_FUNCTION

Workflow:
  1. Start this script before or during flight
  2. Script waits, monitoring altitude and landed state
  3. On landing detection → sends RELEASE signal on ch7
  4. Waits RELEASE_HOLD_S for the servo to fully actuate
  5. Optionally re-locks (for pickup missions)

Usage:
  python payload_release.py               # Release after landing
  python payload_release.py --relock      # Release, wait, then re-lock

Works in SITL and on real hardware (change CONNECTION_STR).
"""

import time
import sys
import argparse
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
# SITL:
CONNECTION_STR = "udp:127.0.0.1:14550"
# Real hardware (RPi to H743 via UART):
# CONNECTION_STR = "/dev/ttyAMA0"
# BAUD_RATE      = 921600

SERVO_CHANNEL    = 7       # FC output channel wired to STM32 A1
PWM_LOCKED       = 1100    # Prongs engaged (STM32 sees 900–1500 → servo 40°)
PWM_RELEASED     = 1800    # Prongs retracted (STM32 sees 1500–2100 → servo 100°)

LAND_ALT_THRESH  = 1.0     # Consider landed below this altitude (m, relative)
RELEASE_HOLD_S   = 3.0     # Hold release position this long before re-locking
RELOCK_DELAY_S   = 2.0     # Extra wait after re-lock before script exits


# ==============================================================================
# HELPERS
# ==============================================================================
def connect():
    print(f"Connecting to {CONNECTION_STR}...")
    if CONNECTION_STR.startswith("/dev"):
        m = mavutil.mavlink_connection(CONNECTION_STR, baud=921600)
    else:
        m = mavutil.mavlink_connection(CONNECTION_STR)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")
    return m

def set_servo(m, channel, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,     # param1: servo number
        pwm,         # param2: PWM value
        0, 0, 0, 0, 0,
    )

def get_alt(m):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0

def is_armed(m):
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    if msg is None:
        return None
    return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--relock", action="store_true",
                        help="Re-lock prongs after release (for pickup missions)")
    args = parser.parse_args()

    m = connect()

    # Ensure payload starts locked
    print(f"\n[1/4] Locking payload (ch{SERVO_CHANNEL} → {PWM_LOCKED} µs)")
    set_servo(m, SERVO_CHANNEL, PWM_LOCKED)
    time.sleep(0.5)

    print("\n[2/4] Waiting for landing...")
    print("       (monitoring altitude — will trigger when alt < "
          f"{LAND_ALT_THRESH} m)")

    landed = False
    was_airborne = False
    last_log = 0.0

    while not landed:
        alt = get_alt(m)
        now = time.time()

        if alt is not None:
            # Must have been airborne first — don't trigger on the ground pre-takeoff
            if alt > 5.0:
                was_airborne = True

            if was_airborne and alt < LAND_ALT_THRESH:
                # Double-check: wait 2 more seconds at low altitude to confirm
                print(f"    alt = {alt:.1f} m — low altitude detected, confirming...")
                confirm_start = time.time()
                confirmed = True
                while time.time() - confirm_start < 2.0:
                    alt2 = get_alt(m)
                    if alt2 is not None and alt2 > LAND_ALT_THRESH + 1.0:
                        confirmed = False
                        print("    false alarm — altitude climbed again")
                        break
                    time.sleep(0.2)
                if confirmed:
                    landed = True
                    break

            if now - last_log >= 3.0:
                status = "airborne" if was_airborne else "on ground (pre-takeoff)"
                print(f"    alt = {alt:.1f} m — {status}")
                last_log = now

        time.sleep(0.1)

    print(f"\n[3/4] Landing confirmed! Releasing payload "
          f"(ch{SERVO_CHANNEL} → {PWM_RELEASED} µs)")
    set_servo(m, SERVO_CHANNEL, PWM_RELEASED)
    print(f"       holding release for {RELEASE_HOLD_S} s...")
    time.sleep(RELEASE_HOLD_S)
    print("       release complete")

    if args.relock:
        print(f"\n[4/4] Re-locking payload (ch{SERVO_CHANNEL} → {PWM_LOCKED} µs)")
        set_servo(m, SERVO_CHANNEL, PWM_LOCKED)
        time.sleep(RELOCK_DELAY_S)
        print("       locked")
    else:
        print("\n[4/4] Done (payload released, prongs stay open)")

    print("\nPayload release sequence complete.")


if __name__ == "__main__":
    main()
