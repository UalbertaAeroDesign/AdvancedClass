#!/usr/bin/env python3
"""
Full mission for quadcopter test platform. Takeoff -> waypoint -> detect white square -> initiate PL -> Land -> Takeoff -> RTL
"""

import time
import cv2
import numpy as np
from pymavlink import mavutil

from gz.transport14 import Node
from gz.msgs11.image_pb2 import Image


# ==============================
# Config
# ==============================
PORT = 14551
GZ_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

HORIZONTAL_FOV = np.radians(60)
VERTICAL_FOV = np.radians(45)


# ==============================
# Gazebo image -> OpenCV BGR
# ==============================
def gz_image_to_bgr(msg: Image):
    w, h = msg.width, msg.height
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if w == 0 or h == 0:
        raise RuntimeError("Bad image dimensions")

    channels = arr.size // (w * h)
    img = arr.reshape((h, w, channels))

    if channels == 1:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if channels == 3:
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if channels == 4:
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

    raise RuntimeError(f"Unexpected channels={channels}")


# ==============================
# White square detector
# ==============================
def get_coords_from_image(bgr):
    """
    Detect a white-ish square-ish blob.

    Returns:
      (distance, angle_x, angle_y, size_x, size_y), mask, vis
      OR (None, mask, vis)
    """
    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # White-ish: low saturation, high value
    mask = cv2.inRange(hsv, (0, 0, 210), (179, 60, 255))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    vis = bgr.copy()
    if not contours:
        return None, mask, vis

    best = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < 800:
            continue

        x, y, w, h = cv2.boundingRect(c)
        ar = w / float(h) if h else 999
        if not (0.8 <= ar <= 1.2):
            continue

        extent = area / float(w * h)
        if extent < 0.6:
            continue

        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.03 * peri, True)
        if len(approx) < 4 or len(approx) > 6:
            continue

        if area > best_area:
            best_area = area
            best = (x, y, w, h)

    if best is None:
        return None, mask, vis

    x, y, w, h = best
    cx = x + w // 2
    cy = y + h // 2

    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)

    # FOV -> focal length in pixels
    fx = bgr.shape[1] / (2.0 * np.tan(HORIZONTAL_FOV / 2.0))
    fy = bgr.shape[0] / (2.0 * np.tan(VERTICAL_FOV / 2.0))

    img_cx = bgr.shape[1] / 2.0
    img_cy = bgr.shape[0] / 2.0

    dx = (cx - img_cx)      # +right
    dy = (img_cy - cy)      # +forward (invert y)

    angle_x = np.arctan2(dx, fx)
    angle_y = np.arctan2(dy, fy)

    size_x = 2.0 * np.arctan2(w / 2.0, fx)
    size_y = 2.0 * np.arctan2(h / 2.0, fy)

    # Simple distance heuristic from apparent width (tune later)
    norm_size = w / float(bgr.shape[1])
    if norm_size < 0.05:
        distance = 4.0
    elif norm_size > 0.5:
        distance = 0.5
    else:
        distance = 4.0 - 7.0 * (norm_size - 0.05) / (0.5 - 0.05)
        distance = float(np.clip(distance, 0.5, 4.0))

    return (distance, angle_x, angle_y, size_x, size_y), mask, vis


# ==============================
# MAVLink helpers
# ==============================
def set_mode(conn, name):
    mode_id = conn.mode_mapping()[name]
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

def arm(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

def takeoff(conn, alt_m=4):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt_m
    )

def set_mode(conn, name):
    mode_id = conn.mode_mapping()[name]
    conn.mav.set_mode_send(
        conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
)


def rtl(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def recv_latest_local_pos(conn, last=None):
    """Drain queue and keep newest LOCAL_POSITION_NED (non-blocking)."""
    while True:
        m = conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if not m:
            return last
        last = m

def recv_ack_nonblocking(conn):
    """Drain any COMMAND_ACK messages (non-blocking), return last one if present."""
    last = None
    while True:
        m = conn.recv_match(type="COMMAND_ACK", blocking=False)
        if not m:
            return last
        last = m

def send_setpoint_local_ned(conn, frame, x, y, z, type_mask=int(0b110111111000)):
    conn.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            int(time.time() * 1000) & 0xFFFFFFFF,
            conn.target_system,
            conn.target_component,
            frame,
            type_mask,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
    )

def send_landing_target(conn, distance, angle_x, angle_y, size_x, size_y):
    msg = conn.mav.landing_target_encode(
        int(time.time() * 1000) & 0xFFFFFFFF,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        float(angle_x),
        float(angle_y),
        float(distance),
        float(size_x),
        float(size_y),
    )
    conn.mav.send(msg)

def recv_latest_heartbeat(conn, last=None):
    while True:
        m = conn.recv_match(type="HEARTBEAT", blocking=False)
        if not m:
            return last
        last = m

def is_armed_from_hb(hb):
    if hb is None:
        return None
    # base_mode bit tells armed state
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

def do_set_home_here(conn):
    print("Setting HOME to current location...")
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1, 0, 0, 0,   # param1=1 → use current position
        0, 0, 0
    )

def wait_armed(conn, want=True, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        m = conn.recv_match(type="HEARTBEAT", blocking=False)
        if m:
            armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed == want:
                return True
        time.sleep(0.05)
    return False

def arm_and_wait(conn, timeout=5.0):
    arm(conn)
    return wait_armed(conn, True, timeout=timeout)

def takeoff_and_wait_alt(conn, target_alt_m=4.0, timeout=15.0):
    # In LOCAL_POSITION_NED, altitude up is negative z
    target_z = -abs(target_alt_m)

    takeoff(conn, alt_m=target_alt_m)

    pos = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        pos = recv_latest_local_pos(conn, pos)
        if pos and pos.z <= target_z * 0.85:
            return True
        time.sleep(0.05)
    return False

# ==============================
# Vision tick (always called)
# ==============================
def vision_tick(latest, conn=None):
    frame = latest["frame"]
    if frame is None:
        return

    coords, mask, vis = get_coords_from_image(frame)

    # show age so you can see if frames go stale
    age_ms = (time.time() - latest["t"]) * 1000.0 if latest["t"] else 0.0
    cv2.putText(vis, f"{age_ms:.0f} ms", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    # send LANDING_TARGET if detected
    if coords and conn is not None:
        distance, ax, ay, sx, sy = coords
        send_landing_target(conn, distance, ax, ay, sx, sy)
        cv2.putText(vis, "LANDING_TARGET sent", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("gz camera", vis)
    cv2.imshow("mask", mask)
    cv2.waitKey(1)



# ==============================
# Main flight
# ==============================
def main():
    # --- Connect MAVLink ---
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{PORT}")
    print("waiting for heartbeat")
    conn.wait_heartbeat()
    print(f"Heartbeat from system {conn.target_system} component {conn.target_component}")

    set_mode(conn, "GUIDED")
    time.sleep(0.5)

    # --- Subscribe camera ---
    node = Node()
    latest = {"frame": None, "t": 0.0}

    def cb(msg: Image):
        try:
            latest["frame"] = gz_image_to_bgr(msg)
            latest["t"] = time.time()
        except Exception as e:
            print("Decode error:", e)

    if not node.subscribe(Image, GZ_TOPIC, cb):
        raise RuntimeError(f"Failed to subscribe: {GZ_TOPIC}")
    print("Subscribed camera:", GZ_TOPIC)
    print("Press q in the camera window to quit (won't stop flight).")

    target_lock = 0
    TARGET_LOCK_N = 20   # ~2 seconds if your loop is ~10 Hz


    # --- Arm ---
    arm(conn)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        vision_tick(latest, conn)
        ack = recv_ack_nonblocking(conn)
        if ack:
            print("ARM ACK:", ack)
        time.sleep(0.01)

    # --- Takeoff to 4m (NED z ~ -4) ---
    takeoff(conn, alt_m=4)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        vision_tick(latest, conn)
        ack = recv_ack_nonblocking(conn)
        if ack:
            print("TAKEOFF ACK:", ack)
        time.sleep(0.01)

    target_alt = -4.0
    print("Waiting until altitude reached...")
    do_set_home_here(conn)
    time.sleep(2.0)   # allow FC to accept it

    pos = None
    t_last_print = 0.0
    while True:
        vision_tick(latest, conn)
        pos = recv_latest_local_pos(conn, pos)

        if pos and (time.time() - t_last_print) > 0.5:
            print(pos)
            t_last_print = time.time()

        if pos and pos.z <= target_alt * 0.85:
            break

        time.sleep(0.01)

    print("Aircraft has reached cruising altitude")

    # --- Waypoint 1: (5,0,-4) in LOCAL_NED ---
    target_x, target_y, target_z = 3, 0, -4
    print("Flying to waypoint 1...")
    pos = None
    while True:
        send_setpoint_local_ned(conn, mavutil.mavlink.MAV_FRAME_LOCAL_NED, target_x, target_y, target_z)

        vision_tick(latest, conn)
        pos = recv_latest_local_pos(conn, pos)

        if pos and pos.z <= -3.5 and pos.x >= 2.5:
            break

        time.sleep(0.1)

    print("Aircraft has reached first waypoint")

    # --- Waypoint 2: (5,2,-4) in LOCAL_OFFSET_NED ---
    target_x, target_y, target_z = 6, 1, -4
    print("Flying to waypoint 2...")
    pos = None
    while True:
        send_setpoint_local_ned(conn, mavutil.mavlink.MAV_FRAME_LOCAL_NED, target_x, target_y, target_z)

        vision_tick(latest, conn)
        pos = recv_latest_local_pos(conn, pos)

        if pos and pos.x >= target_x * 0.9 and pos.y >= target_y * 0.9 and pos.z <= target_z * 0.9:
            break

        time.sleep(0.1)

    print("Aircraft has reached second waypoint")

    # --- Precision landing trigger: require stable detection, then switch to LAND ---
    print("Acquiring target lock before LAND...")

    pos = None
    target_lock = 0
    t_last_print = 0.0

    while True:
        # Run vision + send LANDING_TARGET inside vision_tick (you already do)
        # BUT we also need to know if a target is currently d
        frame = latest["frame"]
        if frame is not None:
            coords, mask, vis = get_coords_from_image(frame)
            if coords:
                # send LANDING_TARGET
                distance, ax, ay, sx, sy = coords
                send_landing_target(conn, distance, ax, ay, sx, sy)
                target_lock += 1
                cv2.putText(vis, f"LOCK {target_lock}/{TARGET_LOCK_N}", (20, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                target_lock = max(0, target_lock - 1)

            age_ms = (time.time() - latest["t"]) * 1000.0 if latest["t"] else 0.0
            cv2.putText(vis, f"{age_ms:.0f} ms", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            cv2.imshow("gz camera", vis)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)

        # Keep position updated (non-blocking)
        pos = recv_latest_local_pos(conn, pos)
        if pos and (time.time() - t_last_print) > 0.5:
            print(pos)
            t_last_print = time.time()

        # Once we’ve got stable target, start LAND
        if target_lock >= TARGET_LOCK_N:
            print("Target locked. Switching to LAND for precision landing.")
            set_mode(conn, "LAND")
            break

        time.sleep(0.1)


    # --- During LAND: keep feeding LANDING_TARGET until touchdown ---
    print("In LAND. Feeding LANDING_TARGET until touchdown, then wait 20s, then RTL...")

    LANDED_Z_THRESH = -0.10
    LANDED_VEL_THRESH = 0.20
    HOLD_ON_GROUND_S = 20.0

    landed_since = None
    t_last_print = time.monotonic()
    pos = None
    hb = None

    while True:
        now = time.monotonic()


        # Vision + landing target feed
        frame = latest["frame"]
        if frame is not None:
            coords, mask, vis = get_coords_from_image(frame)
            if coords:
                distance, ax, ay, sx, sy = coords
                send_landing_target(conn, distance, ax, ay, sx, sy)
            cv2.imshow("gz camera", vis)
            cv2.imshow("mask", mask)
            cv2.waitKey(1)

        # State updates
        pos = recv_latest_local_pos(conn, pos)
        hb = recv_latest_heartbeat(conn, hb)
        armed = is_armed_from_hb(hb) if hb else None

        if pos and (now - t_last_print) > 0.5:
            print(pos, "armed=", armed)
            t_last_print = now

        # touchdown detection: z near ground and low speed
        if pos:
            vel_ok = True
            if hasattr(pos, "vx"):
                speed = (pos.vx**2 + pos.vy**2 + pos.vz**2) ** 0.5
                vel_ok = speed < LANDED_VEL_THRESH

            if pos.z >= LANDED_Z_THRESH and vel_ok:
                if landed_since is None:
                    landed_since = now
                    print("Touchdown detected. Holding on ground for 20s...")
            else:
                landed_since = None

        # after hold: rearm/takeoff then RTL
        if landed_since is not None and (now - landed_since) >= HOLD_ON_GROUND_S:
            print("Hold complete. Preparing relaunch -> RTL...")

            # Get out of LAND before arming/takeoff
            set_mode(conn, "GUIDED")
            time.sleep(1.0)

            # If disarmed, re-arm
            hb = recv_latest_heartbeat(conn, hb)
            armed = is_armed_from_hb(hb) if hb else False
            if not armed:
                print("Vehicle disarmed; re-arming...")
                if not arm_and_wait(conn, timeout=5.0):
                    print("Re-arm failed. Check STATUSTEXT above.")
                    break

            # Re-takeoff
            print("Re-takeoff to 4m...")
            if not takeoff_and_wait_alt(conn, target_alt_m=4.0, timeout=20.0):
                print("Takeoff did not reach altitude in time.")
                break

            # RTL
            print("Switching to RTL...")
            set_mode(conn, "RTL")
            break

        time.sleep(0.05)




if __name__ == "__main__":
    main()
