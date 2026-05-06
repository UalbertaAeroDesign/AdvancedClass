"""
tag_streamer.py
Passive AprilTag detector — streams LANDING_TARGET messages to the FC
whenever a tag is visible. Does NOT touch modes, RC overrides, or flight logic.
Designed to run in the background while a pilot flies manually.

Hardware:
  - Raspberry Pi 5 + Raspberry Pi Camera Module 3 (downward-facing)
  - Matek H743 connected via UART (/dev/ttyAMA0)

Usage:
  python tag_streamer.py

------------------------------------------------------------
MONITORING LANDING_TARGET MESSAGES IN A MAVLINK TERMINAL
------------------------------------------------------------
On any machine that can reach the FC (e.g. via MAVProxy or direct serial):

  Option A — MAVProxy (recommended):
    mavproxy.py --master=/dev/ttyAMA0 --baudrate=921600
    Then in the MAVProxy console:
    #   module load messagerate
    #   set streamrate -1
      message LANDING_TARGET

  Option B — pymavlink one-liner (run on the Pi):
    python3 - <<'EOF'
    from pymavlink import mavutil
    m = mavutil.mavlink_connection("/dev/ttyAMA0", baud=921600)
    m.wait_heartbeat()
    while True:
        msg = m.recv_match(type="LANDING_TARGET", blocking=True, timeout=2.0)
        if msg:
            print(msg)
    EOF

  Option C — MAVProxy over UDP (if FC is forwarding telemetry):
    mavproxy.py --master=udp:0.0.0.0:14550
    Then use the same 'message LANDING_TARGET' command above.
------------------------------------------------------------
"""

import time
import math
import os
from datetime import datetime
import numpy as np
import cv2
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
from pupil_apriltags import Detector
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
SERIAL_PORT  = "/dev/ttyAMA0"
BAUD_RATE    = 921600

W, H         = 1280, 720
HFOV_DEG     = 66.0        # RPi Camera Module 3 standard lens (~66 deg HFOV)
TAG_FAMILY   = "tag36h11"
TAG_SIZE_M   = 0.16        # Physical side length of your AprilTag in metres
DM_MIN       = 15.0        # Minimum detection confidence to send a message

SEND_RATE_HZ = 15          # Max rate to send LANDING_TARGET messages

VIDEO_DIR     = "/home/aerodesign/flights"   # Where recorded video files are saved
VIDEO_BITRATE = 10_000_000                   # H.264 bitrate (bps) — ~10 Mbps


# ==============================================================================
# MAVLINK
# ==============================================================================
def connect():
    print(f"Connecting to FC on {SERIAL_PORT} at {BAUD_RATE} baud...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")
    return m

def landing_target_send(m, ax, ay):
    m.mav.landing_target_send(
        int(time.time() * 1e6),  # time_usec
        0,                        # target_num
        12,                       # MAV_FRAME_BODY_FRD
        float(ax), float(ay),     # angle_x, angle_y (radians)
        0.0, 0.0, 0.0,            # distance, size_x, size_y
        0.0, 0.0, 0.0,            # x, y, z
        [1.0, 0.0, 0.0, 0.0],    # q
        2,                        # MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL
        0                         # position_valid = 0 (angles only)
    )


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    m = connect()

    print("Starting camera...")
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"format": "RGB888", "size": (W, H)},
        controls={"FrameRate": 30, "ExposureTime": 4000}
    ))

    os.makedirs(VIDEO_DIR, exist_ok=True)
    video_path = os.path.join(
        VIDEO_DIR, f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"
    )
    encoder = H264Encoder(bitrate=VIDEO_BITRATE)
    cam.start_recording(encoder, FileOutput(video_path))
    print(f"Recording video to: {video_path}")
    time.sleep(1.0)  # Let auto-exposure settle

    cx = W / 2.0
    cy = H / 2.0
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    print(f"Intrinsics: fx=fy={fx:.1f}  cx={cx:.1f} cy={cy:.1f}")

    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0, refine_edges=True)

    send_dt   = 1.0 / SEND_RATE_HZ
    last_send = 0.0
    last_log  = 0.0

    print("Running — Ctrl+C to stop.\n")

    try:
        while True:
            now   = time.time()
            frame = cam.capture_array()
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

                if now - last_log >= 1.0:
                    print(f"TAG ax={math.degrees(ax):+.1f}°  ay={math.degrees(ay):+.1f}°  dm={best.decision_margin:.0f}")
                    last_log = now
            else:
                if now - last_log >= 1.0:
                    print("NO TAG")
                    last_log = now

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.stop_recording()
        print(f"Video saved: {video_path}")


if __name__ == "__main__":
    main()
