"""
precision_land_rpi.py
Precision landing script for Raspberry Pi 5 + Matek H743, no GPS (indoor).

Hardware:
  - Raspberry Pi 5 connected to H743 via UART (/dev/serial0, GPIO 14/15)
  - Raspberry Pi Camera Module 3 (standard, ~66 deg HFOV) pointing down
  - Matek H743 running ArduCopter

Connection setup:
  RPi GPIO 14 (TX) → H743 RX pin
  RPi GPIO 15 (RX) → H743 TX pin
  RPi GND           → H743 GND
  On RPi: sudo raspi-config → Interface Options → Serial Port
          → disable login shell, enable hardware serial
  On H743: set SERIALx_PROTOCOL = 2 (MAVLink2), SERIALx_BAUD = 921 (921600)

Required ArduPilot parameters (set once in Mission Planner / QGC before flight):
  GPS_TYPE         = 0      # Disable GPS
  EK3_SRC1_POSXY   = 0      # No horizontal position source
  EK3_SRC1_VELXY   = 0      # No horizontal velocity source
  ARMING_CHECK     = 16384  # Disable GPS arming check (or set to 0 to skip all checks)
  PLND_ENABLED     = 1
  PLND_TYPE        = 1      # MAVLink LANDING_TARGET
  PLND_EST_TYPE    = 1      # Kalman filter
  PLND_STRICT      = 0      # Allow landing even if target is briefly lost
  FS_GCS_ENABLE    = 0      # Disable GCS failsafe (script runs headless)
  FS_THR_ENABLE    = 0      # Disable throttle failsafe (no RC in autonomous mode)

Install deps on RPi:
  pip install picamera2 pupil-apriltags pymavlink opencv-python numpy
"""

import time
import math
import threading
import numpy as np
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
from picamera2 import Picamera2
from pupil_apriltags import Detector
from pymavlink import mavutil

# ==============================================================================
# CONFIGURATION — edit these before each flight
# ==============================================================================

SERIAL_PORT  = "/dev/serial0"
BAUD_RATE    = 921600

# Set True to wait for the pilot to get airborne manually.
# Set False for fully autonomous arm + climb + land.
MANUAL_TAKEOFF = True

TARGET_ALT_M   = 2.0    # Target hover altitude in metres (autonomous mode) or
                         # minimum alt to detect as airborne (manual mode)
TAG_SIZE_M     = 0.16   # Physical side length of AprilTag in metres
TAG_FAMILY     = "tag36h11"
DM_MIN         = 15.0   # Minimum AprilTag detection confidence

W, H        = 1280, 720
HFOV_DEG    = 66.0      # Approximate HFOV for RPi Camera Module 3 standard.
                         # For best accuracy, calibrate with a checkerboard and
                         # replace fx/fy/cx/cy below with calibrated values.

SEND_RATE_HZ    = 15    # How often to send LANDING_TARGET to FC
HOVER_SEC       = 5.0   # How long to hover over tag before triggering LAND
                         # In manual mode, this is a mandatory wait after acquiring the tag.
                         # Increase if you need more time to stabilise.

# Yaw correction (autonomous mode only — do not apply RC overrides in manual mode)
YAW_KP           = 5.0  # P-gain: PWM per degree of yaw error
YAW_DEADBAND_DEG = 3.0
YAW_HOLD_SEC     = 1.5  # Must be within deadband this long before transitioning to LAND

# Throttle PWM for autonomous climb/hover in ALT_HOLD mode
THROTTLE_CLIMB = 1650
THROTTLE_HOVER = 1500

# Ground station monitoring stream (MJPEG over HTTP — open in any browser)
STREAM_VIDEO = True
STREAM_PORT  = 8080     # Access at http://<RPi_IP>:8080/
STREAM_FPS   = 15       # Stream framerate (independent of detection rate)
STREAM_W     = 640      # Downscaled width for stream (saves bandwidth)
STREAM_H     = 360      # Downscaled height for stream


# ==============================================================================
# MJPEG STREAM — monitoring only, runs in a background thread
# ==============================================================================

_stream_frame = None
_stream_lock  = threading.Lock()

def push_stream_frame(frame):
    """Write an annotated BGR frame to the shared stream buffer."""
    global _stream_frame
    small = cv2.resize(frame, (STREAM_W, STREAM_H))
    with _stream_lock:
        _stream_frame = small

class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # Silence per-request HTTP logs

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _stream_lock:
                    frame = _stream_frame
                if frame is None:
                    time.sleep(0.05)
                    continue
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / STREAM_FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass  # Client disconnected cleanly

def start_mjpeg_stream():
    server = HTTPServer(("0.0.0.0", STREAM_PORT), _MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"Stream: http://<RPi_IP>:{STREAM_PORT}/  (open in any browser)")


def annotate_frame(frame, phase, best=None, yaw_error_deg=None, alt=None):
    """Draw detection overlay and status text onto a copy of the frame."""
    out = frame.copy()
    if best is not None:
        pts = best.corners.astype(int)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        cx_tag, cy_tag = int(best.center[0]), int(best.center[1])
        cv2.circle(out, (cx_tag, cy_tag), 6, (0, 255, 0), -1)
        info = f"dm={best.decision_margin:.0f}"
        if yaw_error_deg is not None:
            info += f"  yaw={yaw_error_deg:+.1f}deg"
        if alt is not None:
            info += f"  alt={alt:.2f}m"
        cv2.putText(out, info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    else:
        cv2.putText(out, "NO TAG", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(out, f"phase: {phase}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    return out


# ==============================================================================
# MAVLINK HELPERS
# ==============================================================================

def connect():
    print(f"Connecting to FC on {SERIAL_PORT} at {BAUD_RATE} baud...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")
    return m

def set_param(m, name, value):
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("ascii"), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)

def set_mode(m, mode_str):
    mode_str = mode_str.upper()
    mm = m.mode_mapping()
    if mode_str not in mm:
        raise RuntimeError(f"Mode '{mode_str}' not in FC mode list")
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mm[mode_str]
    )
    time.sleep(0.5)

def arm(m, timeout=10.0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Armed")
            return True
    return False

def is_armed(m):
    hb = m.recv_match(type="HEARTBEAT", blocking=False)
    if hb is None:
        return True  # No heartbeat yet — assume still armed
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def get_rel_alt(m):
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    return msg.relative_alt / 1000.0 if msg else None

def send_rc(m, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    chans = [0] * 18
    chans[0] = roll
    chans[1] = pitch
    chans[2] = throttle
    chans[3] = yaw
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chans)

def release_rc(m):
    """Stop all RC overrides so the FC reverts to its own control."""
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *([0] * 18))

def landing_target_send(m, ax, ay):
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        0,
        12,           # MAV_FRAME_BODY_FRD
        float(ax), float(ay),
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        [1.0, 0.0, 0.0, 0.0],
        2,            # MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL
        0
    )


# ==============================================================================
# CAMERA
# ==============================================================================

def start_camera():
    cam = Picamera2()
    cfg = cam.create_video_configuration(
        main={"format": "RGB888", "size": (W, H)},
        controls={"FrameRate": 30}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(1.0)  # Allow auto-exposure to settle
    return cam

def grab_frame(cam):
    """Capture a frame and return it as a BGR numpy array."""
    frame = cam.capture_array()            # picamera2 gives RGB
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    m = connect()

    print("Writing PLND params...")
    set_param(m, "PLND_ENABLED",  1)
    set_param(m, "PLND_TYPE",     1)
    set_param(m, "PLND_EST_TYPE", 1)
    set_param(m, "PLND_STRICT",   0)
    set_param(m, "FS_GCS_ENABLE", 0)

    print("Starting camera...")
    cam = start_camera()
    if STREAM_VIDEO:
        start_mjpeg_stream()

    # Camera intrinsics
    cx, cy = W / 2.0, H / 2.0
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    print(f"Camera intrinsics: {W}x{H}, fx=fy={fx:.1f}, cx={cx:.1f}, cy={cy:.1f}")

    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0, refine_edges=True)

    send_dt  = 1.0 / SEND_RATE_HZ
    last_send = 0.0

    # ------------------------------------------------------------------
    # PHASE 1: TAKEOFF
    # ------------------------------------------------------------------
    if MANUAL_TAKEOFF:
        print(f"MANUAL mode — waiting for altitude > {TARGET_ALT_M}m...")
        while True:
            alt = get_rel_alt(m)
            if alt is not None and alt >= TARGET_ALT_M:
                print(f"Airborne at {alt:.1f}m — proceeding.")
                break
            time.sleep(0.2)
    else:
        print("AUTO mode — arming in ALT_HOLD and climbing...")
        set_mode(m, "ALT_HOLD")
        if not arm(m):
            raise RuntimeError("Arm failed — check ARMING_CHECK and prearm messages")

        climb_start = time.time()
        last_rc = 0.0
        while True:
            now = time.time()
            if now - climb_start > 25.0:
                raise RuntimeError("Climb timeout — check throttle trim and barometer")
            alt = get_rel_alt(m)
            if alt is not None and alt >= TARGET_ALT_M:
                print(f"Reached {alt:.1f}m")
                break
            if now - last_rc >= 0.05:
                send_rc(m, throttle=THROTTLE_CLIMB)
                last_rc = now
        # Hold altitude
        send_rc(m, throttle=THROTTLE_HOVER)

    # ------------------------------------------------------------------
    # PHASE 2: HOVER + YAW ALIGN (over tag, before landing)
    # ------------------------------------------------------------------
    print("Phase: HOVER — acquiring tag and aligning yaw...")
    phase         = "HOVER"
    hover_start   = None   # starts when tag is first acquired
    yaw_ok_since  = None
    yaw_pwm       = 1500
    last_rc       = 0.0

    while phase == "HOVER":
        now = time.time()

        if not is_armed(m):
            print("Disarmed unexpectedly.")
            cam.stop()
            return

        # In autonomous mode, keep altitude held and apply yaw correction
        if not MANUAL_TAKEOFF and now - last_rc >= 0.1:
            send_rc(m, throttle=THROTTLE_HOVER, yaw=yaw_pwm)
            last_rc = now

        frame = grab_frame(cam)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags  = det.detect(gray, estimate_tag_pose=True,
                           camera_params=[fx, fy, cx, cy], tag_size=TAG_SIZE_M)
        best  = max(tags, key=lambda t: t.decision_margin) if tags else None

        if best and best.decision_margin >= DM_MIN:
            u, v = best.center
            ax = math.atan((u - cx) / fx)
            ay = math.atan((v - cy) / fy)

            R = best.pose_R
            yaw_error_deg = math.degrees(math.atan2(R[1][0], R[0][0]))

            # Send landing target to FC
            if now - last_send >= send_dt:
                landing_target_send(m, ax, ay)
                last_send = now

            # Start hover timer on first tag acquisition
            if hover_start is None:
                hover_start = now
                print("Tag acquired — hovering...")

            # Yaw correction: RC override in auto mode, advisory print in manual mode
            if abs(yaw_error_deg) > YAW_DEADBAND_DEG:
                yaw_pwm = int(max(1300, min(1700, 1500 + YAW_KP * yaw_error_deg)))
                yaw_ok_since = None
                if MANUAL_TAKEOFF:
                    print(f"  Yaw error {yaw_error_deg:+.1f}° — please rotate drone")
            else:
                yaw_pwm = 1500
                if yaw_ok_since is None:
                    yaw_ok_since = now

            # Transition to LAND once hovered long enough AND yaw aligned
            yaw_aligned = (yaw_ok_since is not None and
                           now - yaw_ok_since >= YAW_HOLD_SEC)
            hovered_long_enough = (hover_start is not None and
                                   now - hover_start >= HOVER_SEC)

            if hovered_long_enough and yaw_aligned:
                print(f"Hover complete, yaw aligned ({yaw_error_deg:+.1f}°) — triggering LAND")
                phase = "LAND"

            print(f"  ax={math.degrees(ax):+.1f}° ay={math.degrees(ay):+.1f}° "
                  f"yaw={yaw_error_deg:+.1f}° alt={get_rel_alt(m) or '?':.2f}m", end="\r")

            if STREAM_VIDEO:
                push_stream_frame(annotate_frame(frame, "HOVER", best, yaw_error_deg, get_rel_alt(m)))
        else:
            yaw_pwm = 1500
            yaw_ok_since = None  # Reset yaw hold if tag is lost
            if STREAM_VIDEO:
                push_stream_frame(annotate_frame(frame, "HOVER"))

    # ------------------------------------------------------------------
    # PHASE 3: PRECISION LAND
    # ------------------------------------------------------------------
    if not MANUAL_TAKEOFF:
        release_rc(m)  # Let LAND mode control throttle

    set_mode(m, "LAND")
    print("\nPhase: PRECISION LAND")

    last_send = 0.0
    last_print = 0.0

    while True:
        now = time.time()

        if not is_armed(m):
            print("\nDisarmed — landed successfully.")
            break

        frame = grab_frame(cam)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags  = det.detect(gray, estimate_tag_pose=True,
                           camera_params=[fx, fy, cx, cy], tag_size=TAG_SIZE_M)
        best  = max(tags, key=lambda t: t.decision_margin) if tags else None

        if best and best.decision_margin >= DM_MIN:
            u, v = best.center
            ax = math.atan((u - cx) / fx)
            ay = math.atan((v - cy) / fy)
            if now - last_send >= send_dt:
                landing_target_send(m, ax, ay)
                last_send = now
            if now - last_print >= 1.0:
                alt = get_rel_alt(m)
                print(f"  LAND: ax={math.degrees(ax):+.1f}° ay={math.degrees(ay):+.1f}° "
                      f"dm={best.decision_margin:.0f} alt={alt:.2f}m" if alt else
                      f"  LAND: ax={math.degrees(ax):+.1f}° ay={math.degrees(ay):+.1f}° "
                      f"dm={best.decision_margin:.0f}")
                last_print = now
            if STREAM_VIDEO:
                push_stream_frame(annotate_frame(frame, "LAND", best, alt=get_rel_alt(m)))
        else:
            if now - last_print >= 1.0:
                print("  LAND: tag not visible")
                last_print = now
            if STREAM_VIDEO:
                push_stream_frame(annotate_frame(frame, "LAND"))

    cam.stop()
    print("Done.")


if __name__ == "__main__":
    main()
