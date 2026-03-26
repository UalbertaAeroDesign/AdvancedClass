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
WATCHING THE VIDEO FEED IN A BROWSER
------------------------------------------------------------
This script streams an annotated MJPEG feed over HTTP.
1. Make sure your laptop is on the same WiFi network as the Pi.
2. Find the Pi's IP address:
     hostname -I          (run on the Pi)
3. Open a browser and go to:
     http://<PI_IP_ADDRESS>:8080/
   Example: http://192.168.1.42:8080/
The feed shows the camera view with the tag outlined in green when detected,
and a "NO TAG" message in red when not detected.

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
    Then use the same 'message LANDING_TARGETd' command above.
------------------------------------------------------------
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

STREAM_PORT  = 8080        # Browser stream port
STREAM_FPS   = 15
STREAM_W     = 640
STREAM_H     = 360


# ==============================================================================
# MJPEG STREAM
# ==============================================================================
_stream_frame = None
_stream_lock  = threading.Lock()

def push_stream_frame(frame):
    global _stream_frame
    small = cv2.resize(frame, (STREAM_W, STREAM_H))
    _, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 65])
    with _stream_lock:
        _stream_frame = jpg.tobytes()

class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # Suppress per-request logs

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _stream_lock:
                    jpg = _stream_frame
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / STREAM_FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass

def start_stream():
    server = HTTPServer(("0.0.0.0", STREAM_PORT), _MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    print(f"Video stream: http://<PI_IP>:{STREAM_PORT}/")


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
    cam.start()
    time.sleep(1.0)  # Let auto-exposure settle

    cx = W / 2.0
    cy = H / 2.0
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    print(f"Intrinsics: fx=fy={fx:.1f}  cx={cx:.1f} cy={cy:.1f}")

    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0, refine_edges=True)

    start_stream()

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

                # Annotate frame
                pts = best.corners.astype(int)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
                cv2.putText(frame,
                            f"ax={math.degrees(ax):+.1f} ay={math.degrees(ay):+.1f} dm={best.decision_margin:.0f}",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            else:
                if now - last_log >= 1.0:
                    print("NO TAG")
                    last_log = now
                cv2.putText(frame, "NO TAG", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

            push_stream_frame(frame)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
