import time, math, sys
import numpy as np
import cv2
from pupil_apriltags import Detector
from pymavlink import mavutil

# Config
MAVLINK_CONNECTION = "udp:127.0.0.1:14551"
BAUD_RATE = 56700

# These are not real intrinsics YET
W, H = 640, 480
FX, FY = 534.21, 534.15
CX, CY = 320.0, 240.0

TAG_FAMILY = "tag36h11"
DECISION_MARGIN_THRESHOLD = 10.0 

def connect_drone():
    m = mavutil.mavlink_connection(MAVLINK_CONNECTION, baud=BAUD_RATE)
    m.wait_heartbeat()
    return m

def landing_target_send(m, ax, ay, target_id):
    """Sends the angle offset. Note the use of [1.0, 0.0, 0.0, 0.0] for Python 3.14 fix."""
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        target_id,           # Now using the actual ID of the tag
        12,                  # MAV_FRAME_BODY_FRD
        float(ax), float(ay),
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        [1.0, 0.0, 0.0, 0.0], # Fixed list for quaternion
        2,                   # MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL
        1                    # position_valid
    )

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    
    # quad_decimate > 1.0 speeds up detection but reduces range
    det = Detector(families=TAG_FAMILY, nthreads=4, quad_decimate=1.0)
    
    try:
        m = connect_drone()
    except Exception as e:
        print(f"Connection failed: {e}")
        return

    print("System Ready. Multi-tag tracking active...")
    last_send = 0

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = det.detect(gray)
        
        now = time.time()

        # LOOP THROUGH ALL SUSPECTED TAGS
        for tag in tags:
            if tag.decision_margin < DECISION_MARGIN_THRESHOLD:
                continue

            u, v = tag.center
            ax = math.atan((u - CX) / FX)
            ay = math.atan((v - CY) / FY)

            # Only send MAVLink data for the tag closest to the center 
            # OR you can send all, but ArduPilot usually tracks one target_num.
            if now - last_send >= (1.0 / 20.0):
                landing_target_send(m, ax, ay, tag.tag_id)
                last_send = now

            # Visual feedback for ALL detected tags
            pts = tag.corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (int(u), int(v)), 5, (0, 255, 0), -1)
            # cv2.putText(frame, f"ID:{tag.tag_id} Conf:{int(tag.decision_margin)}", 
            #             (int(u)+10, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Multi-Tag Tracker", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()