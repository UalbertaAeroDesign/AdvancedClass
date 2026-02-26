import time
import math
import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2
from pymavlink import mavutil

# ==============================================================================
# CONFIG
# ==============================================================================
SERIAL_PORT  = "/dev/serial0" 
BAUD_RATE    = 57600

W, H         = 640, 480
HFOV_DEG     = 66.0      # RPi Camera Module 3 standard (approximate)
TAG_SIZE_M   = 0.16
DM_MIN       = 15.0
SEND_RATE_HZ = 15


# ==============================================================================
# MAVLINK
# ==============================================================================
def connect():
    print(f"Connecting to FC on {SERIAL_PORT} @ {BAUD_RATE}...")
    m = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
    m.wait_heartbeat()
    print(f"Heartbeat OK (sys={m.target_system} comp={m.target_component})")
    return m

def landing_target_send(m, ax, ay):
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        0,
        12,           # MAV_FRAME_BODY_FRD
        float(ax), float(ay),
        0.0,
        0.0, 0.0,
        0.0, 0.0, 0.0,
        [1.0, 0.0, 0.0, 0.0],
        2,            # MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL
        0
    )


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    m = connect()

    # Camera intrinsics
    cx, cy = W / 2.0, H / 2.0
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx
    print(f"Intrinsics: {W}x{H} fx=fy={fx:.1f}")

    # Camera
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (W, H)}
    )
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2})  # Continuous autofocus
    picam2.start()

    det = Detector(families="tag36h11", nthreads=2, quad_decimate=1.0, refine_edges=True)

    send_dt   = 1.0 / SEND_RATE_HZ
    last_send = 0.0

    print("Running — watching for tags and sending LANDING_TARGET to FC.")
    print("In MAVProxy run:  watch LANDING_TARGET")
    print("Press Ctrl+C to exit.\n")

    try:
        while True:
            now = time.time()

            frame = picam2.capture_array()          # RGB
            gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            tags = det.detect(gray, estimate_tag_pose=True,
                              camera_params=[fx, fy, cx, cy], tag_size=TAG_SIZE_M)
            best = max(tags, key=lambda t: t.decision_margin) if tags else None

            if best and best.decision_margin >= DM_MIN:
                u, v = best.center
                ax = math.atan((u - cx) / fx)
                ay = math.atan((v - cy) / fy)

                if now - last_send >= send_dt:
                    landing_target_send(m, ax, ay)
                    last_send = now
                    print(f"LANDING_TARGET sent — ax={math.degrees(ax):+.2f}° "
                          f"ay={math.degrees(ay):+.2f}°  dm={best.decision_margin:.1f}")

                # Draw overlay
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.polylines(bgr, [best.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(bgr, (int(u), int(v)), 5, (0, 255, 0), -1)
                cv2.putText(bgr, f"ID:{best.tag_id}  ax={math.degrees(ax):+.1f} ay={math.degrees(ay):+.1f}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.imshow("AeroDesign AprilTag Detection", bgr)
            else:
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.putText(bgr, "NO TAG", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("AeroDesign AprilTag Detection", bgr)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
