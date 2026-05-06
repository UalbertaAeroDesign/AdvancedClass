"""
ov9281_tag_detect.py
Standalone AprilTag detection test for the OV9281 camera.
Opens a live window on the Pi's attached monitor and draws a HUGE, obvious
overlay when a tag is detected. No MAVLink, no recording — pure detection test.

Use this to:
  - Verify the OV9281 is wired up and libcamera sees it
  - Confirm AprilTag decoding is working with your tag size/family
  - Quick check HFOV / exposure / gain / lens focus before a flight

Hardware:
  - Raspberry Pi 5 + Arducam OV9281 (monochrome global shutter)
  - Monitor attached to the Pi's HDMI (or VNC session with display forwarding)

Prerequisites:
  - dtoverlay=ov9281 in /boot/firmware/config.txt
  - Picamera2 + pupil_apriltags installed
  - Run from a desktop session (not headless SSH) so OpenCV can open a window
    — or SSH with -X / -Y for X11 forwarding

Usage:
  python ov9281_tag_detect.py

Press 'q' or ESC in the window to quit.
"""

import time
import math
import cv2
from picamera2 import Picamera2
from pupil_apriltags import Detector

# ==============================================================================
# CONFIG
# ==============================================================================
W, H          = 1280, 800    # OV9281 native resolution
HFOV_DEG      = 70.0         # Stock Arducam OV9281 M12 lens — CHECK YOUR LENS!
TAG_FAMILY    = "tag36h11"
TAG_SIZE_M    = 0.16         # Physical side length of your AprilTag in metres
DM_MIN        = 15.0         # Minimum decision margin to consider "detected"

EXPOSURE_US   = 2000         # Short exposure — global shutter kills motion blur
ANALOG_GAIN   = 4.0
FRAME_RATE    = 60

WINDOW_NAME   = "OV9281 AprilTag Detect"


def main():
    print("Starting OV9281...")
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"format": "YUV420", "size": (W, H)},
        controls={
            "FrameRate":    FRAME_RATE,
            "ExposureTime": EXPOSURE_US,
            "AnalogueGain": ANALOG_GAIN,
        }
    ))
    cam.start()
    time.sleep(1.0)

    cx = W / 2.0
    cy = H / 2.0
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    print(f"Intrinsics: fx=fy={fx:.1f}  cx={cx:.1f} cy={cy:.1f}")

    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0, refine_edges=True)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 800)

    print("Running — press 'q' or ESC in the window to quit.\n")

    last_fps_t = time.time()
    fps_count  = 0
    fps        = 0.0

    try:
        while True:
            # YUV420: first H rows are the Y (luma) plane = our mono image
            frame = cam.capture_array()
            gray  = frame[:H, :]

            tags = det.detect(gray, estimate_tag_pose=True,
                              camera_params=[fx, fy, cx, cy], tag_size=TAG_SIZE_M)
            best = max(tags, key=lambda t: t.decision_margin) if tags else None

            # Convert to BGR so we can draw colored overlays
            disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            if best and best.decision_margin >= DM_MIN:
                # ---- TAG DETECTED: big obvious overlay ----
                pts = best.corners.astype(int)
                u, v = best.center
                ax = math.degrees(math.atan((u - cx) / fx))
                ay = math.degrees(math.atan((v - cy) / fy))

                # Thick green outline around the tag
                cv2.polylines(disp, [pts], True, (0, 255, 0), 6)
                # Center dot
                cv2.circle(disp, (int(u), int(v)), 12, (0, 255, 0), -1)
                # Crosshair from image center to tag center
                cv2.line(disp, (int(cx), int(cy)), (int(u), int(v)), (0, 255, 255), 2)

                # Giant green banner across the top
                cv2.rectangle(disp, (0, 0), (W, 90), (0, 200, 0), -1)
                cv2.putText(disp, "TAG DETECTED", (30, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.2, (255, 255, 255), 5)

                # Details below the banner
                cv2.putText(disp,
                            f"id={best.tag_id}  ax={ax:+.1f}  ay={ay:+.1f}  dm={best.decision_margin:.0f}",
                            (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            else:
                # ---- NO TAG: big red banner ----
                cv2.rectangle(disp, (0, 0), (W, 90), (0, 0, 200), -1)
                cv2.putText(disp, "NO TAG", (30, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.2, (255, 255, 255), 5)

            # Image-center crosshair
            cv2.drawMarker(disp, (int(cx), int(cy)), (255, 255, 255),
                           cv2.MARKER_CROSS, 30, 2)

            # FPS counter (bottom-left)
            fps_count += 1
            if time.time() - last_fps_t >= 1.0:
                fps = fps_count / (time.time() - last_fps_t)
                fps_count = 0
                last_fps_t = time.time()
            cv2.putText(disp, f"{fps:.1f} fps", (20, H - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.imshow(WINDOW_NAME, disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
