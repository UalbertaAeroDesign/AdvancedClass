#!/usr/bin/env python3
import argparse
import time
from pymavlink import mavutil

def wait_mode(m, desired: str, timeout_s: float = 10.0) -> bool:
    """Wait until HEARTBEAT reports a given mode string (best-effort)."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is None:
            continue
        mode = mavutil.mode_string_v10(hb)
        if mode.upper() == desired.upper():
            return True
    return False

def set_mode(m, mode: str) -> None:
    """Set ArduPilot mode (GUIDED, LAND, LOITER, etc.)."""
    # ArduPilot-specific convenience
    m.set_mode_apm(mode)
    # Optional: wait a bit for it to take effect
    time.sleep(0.5)

def arm_and_takeoff(m, alt_m: float) -> None:
    set_mode(m, "GUIDED")

    print("Arming...")
    m.arducopter_arm()
    m.motors_armed_wait()
    print("Armed.")

    print(f"Takeoff to {alt_m:.1f} m...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,   # params 1-4 unused for copter takeoff
        0, 0, alt_m   # lat, lon, alt
    )

    # Give it time to climb (simple wait; you can make this smarter later)
    time.sleep(max(5.0, alt_m * 1.5))

def send_landing_target_loop(
    m,
    angle_x: float,
    angle_y: float,
    rate_hz: float,
    duration_s: float,
) -> None:
    """
    Stream LANDING_TARGET. We use an angular-only target in MAV_FRAME_BODY_FRD.
    If motion is opposite of what you expect, flip the sign or swap angle_x/angle_y.
    """
    period = 1.0 / rate_hz
    t_end = time.time() + duration_s
    print(f"Streaming LANDING_TARGET @ {rate_hz:.1f} Hz for {duration_s:.1f}s "
          f"(angle_x={angle_x:.3f} rad, angle_y={angle_y:.3f} rad) ...")

    # Common frames:
    # - MAV_FRAME_BODY_FRD: x forward, y right, z down (body frame)
    frame = mavutil.mavlink.MAV_FRAME_BODY_FRD

    while time.time() < t_end:
        now_us = int(time.time() * 1e6)

        # MAVLink LANDING_TARGET fields:
        # time_usec, target_num, frame, angle_x, angle_y, distance,
        # size_x, size_y, x, y, z, q, type, position_valid
        #
        # We provide angles only. distance/x/y/z are set to 0 and position_valid=0.
        m.mav.landing_target_send(
            now_us,
            0,          # target_num
            frame,
            angle_x,     # angular offset in x direction (rad)
            angle_y,     # angular offset in y direction (rad)
            0.0,         # distance (unknown)
            0.0, 0.0,    # size_x, size_y (unknown)
            0.0, 0.0, 0.0,  # x, y, z (unused when position_valid=0)
            (0.0, 0.0, 0.0, 1.0),  # q (identity)
            0,          # type (0 = generic)
            0           # position_valid (0 = angles only)
        )

        time.sleep(period)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="udp:127.0.0.1:14550",
                    help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    ap.add_argument("--takeoff-alt", type=float, default=5.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--angle-x", type=float, default=0.15,
                    help="Target angular offset x (radians). Try 0.05–0.25.")
    ap.add_argument("--angle-y", type=float, default=0.0,
                    help="Target angular offset y (radians).")
    ap.add_argument("--land-first", action="store_true",
                    help="Skip takeoff and just switch to LAND + stream target (useful if already airborne).")
    args = ap.parse_args()

    print(f"Connecting to {args.port} ...")
    m = mavutil.mavlink_connection(args.port)
    m.wait_heartbeat(timeout=15)
    print(f"Heartbeat OK (sys={m.target_system}, comp={m.target_component}).")

    try:
        if not args.land_first:
            arm_and_takeoff(m, args.takeoff_alt)

        print("Switching to LAND...")
        set_mode(m, "LAND")
        # (Optional) wait_mode(m, "LAND", timeout_s=5.0)

        send_landing_target_loop(
            m,
            angle_x=args.angle_x,
            angle_y=args.angle_y,
            rate_hz=args.rate,
            duration_s=args.seconds,
        )

        print("Done. (If you want to see fallback behavior, stop sending target while still landing.)")

    except KeyboardInterrupt:
        print("\nInterrupted. Exiting.")
    finally:
        try:
            m.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
