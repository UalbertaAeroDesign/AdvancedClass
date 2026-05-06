import time, math, subprocess
import numpy as np
import cv2
from pymavlink import mavutil
from pupil_apriltags import Detector

# Config
CONNECTION_STRING = "udp:127.0.0.1:14551"
SDP_FILE = "gazebo5601.sdp"

TARGET_ALTITUDE = 10.0
SEND_RATE_HZ = 20

W, H = 1280, 720
HFOV_DEG = 60.0
DEADBAND_DEG = 0.8

# Camera intrinsics
CAM_CX, CAM_CY = W / 2.0, H / 2.0
FX = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
FY = FX

FFMPEG_COMMAND = [
    "ffmpeg",
    "-hide_banner", "-loglevel", "error",
    "-protocol_whitelist", "file,udp,rtp",
    "-fflags", "nobuffer",
    "-flags", "low_delay",
    "-strict", "experimental",
    "-i", SDP_FILE,
    "-f", "image2pipe",
    "-pix_fmt", "bgr24",
    "-vcodec", "rawvideo",
    "-an", "-"
]

at_detector = Detector(families="tag36h11", nthreads=2, quad_decimate=2.0)

def apply_deadband(angle_rad):
    if abs(math.degrees(angle_rad)) < DEADBAND_DEG:
        return 0.0
    return angle_rad

def clamp_angle(limit, val):
    if val < 0:
        return max(-limit, val)
    return min(limit, val)

def set_param(m, name, value):
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("ascii"), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

def connect_drone():
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print("Heartbeat received!")
    time.sleep(1)

    print("Setting Parameters...")
    set_param(master, "PLND_ENABLED", 1)
    set_param(master, "PLND_TYPE", 1)
    set_param(master, "PLND_STRICT", 0)
    set_param(master, "PLND_ACC_P_NSE", 0.8)
    return master

def set_mode(master, mode):
    mode = mode.upper()
    mm = master.mode_mapping()
    if mode not in mm:
        print(f"Mode {mode} not supported.")
        return False
    master.mav.set_mode_send(master.target_system, 1, mm[mode])
    return True

def arm_drone(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed.")

def get_latest_msg(master, msg_type):
    latest = None
    while True:
        m = master.recv_match(type=msg_type, blocking=False)
        if m is None:
            break
        latest = m
    return latest

def get_height_agl_m(master):
    tr = get_latest_msg(master, "TERRAIN_REPORT")
    if tr is not None:
        return float(tr.current_height)
    gp = get_latest_msg(master, "GLOBAL_POSITION_INT")
    if gp is not None:
        return gp.relative_alt / 1000.0
    return None

QCLIMB_THROTTLE = 1700
QHOVER_THROTTLE = 1500
QTHROTTLE_MIN = 1300
QTHROTTLE_MAX = 1900

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def landing_target_send(m, ax, ay, dist_m):
    safe_dist = float(dist_m) if dist_m is not None else TARGET_ALTITUDE
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        0, 12,
        float(ax), float(ay),
        safe_dist,
        0.0, 0.0,
        0.0, 0.0, 0.0,
        [1.0, 0.0, 0.0, 0.0],
        2, 0
    )

def send_rc_throttle(master, throttle_pwm):
    throttle_pwm = clamp(int(throttle_pwm), QTHROTTLE_MIN, QTHROTTLE_MAX)
    chans = [65535] * 18
    chans[2] = throttle_pwm
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *chans
    )

def release_rc_overrides(master):
    chans = [0] * 18
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *chans
    )
    print("RC overrides released.")

def vtol_climb_to_alt(master, target_alt_m, timeout=25.0):
    print(f"Climbing to {target_alt_m:.1f}m...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        gp = get_latest_msg(master, "GLOBAL_POSITION_INT")
        if gp and (gp.relative_alt / 1000.0) >= target_alt_m:
            print(f"Reached {gp.relative_alt / 1000.0:.1f}m")
            return True
        send_rc_throttle(master, QCLIMB_THROTTLE)
        time.sleep(0.1)
    print("Climb timeout.")
    return False

def main():
    drone = connect_drone()

    set_mode(drone, "QLOITER")
    arm_drone(drone)
    vtol_climb_to_alt(drone, TARGET_ALTITUDE)
    release_rc_overrides(drone)

    # Immediately enter QLAND
    print("Switching to QLAND...")
    set_mode(drone, "QLAND")

    print("\nSending LANDING_TARGET during descent\n")
    pipe = subprocess.Popen(FFMPEG_COMMAND, stdout=subprocess.PIPE, bufsize=W * H * 3 * 10)

    last_send = 0.0
    last_dist = TARGET_ALTITUDE
    send_dt = 1.0 / SEND_RATE_HZ
    filt_ax, filt_ay = 0.0, 0.0
    alpha = 1

    try:
        while True:
            now = time.time()

            # Check for disarm (touchdown)
            hb = get_latest_msg(drone, "HEARTBEAT")
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("Disarmed — touchdown complete.")
                break

            raw = pipe.stdout.read(W * H * 3)
            if len(raw) != W * H * 3:
                continue

            frame = np.frombuffer(raw, dtype="uint8").reshape((H, W, 3)).copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[FX, FY, CAM_CX, CAM_CY],
                tag_size=0.40
            )

            if detections:
                best = max(detections, key=lambda t: t.decision_margin)
                if best.decision_margin > 15:
                    u, v = best.center

                    ax = clamp_angle(0.1, math.atan((u - CAM_CX) / FX))
                    ay = clamp_angle(0.4, math.atan((v - CAM_CY) / FY))

                    filt_ax = apply_deadband((alpha * ax) + (1.0 - alpha) * filt_ax)
                    filt_ay = apply_deadband((alpha * ay) + (1.0 - alpha) * filt_ay)

                    if now - last_send >= send_dt:
                        dist = get_height_agl_m(drone)
                        if dist is not None and dist > 0.1:
                            last_dist = dist

                        print(f"ax={math.degrees(filt_ax):.2f} deg | ay={math.degrees(filt_ay):.2f} deg | dist={last_dist:.2f}m")
                        landing_target_send(drone, filt_ax, filt_ay, last_dist)
                        last_send = now

                    tag_x, tag_y = int(u), int(v)
                    pts = best.corners.astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                    cv2.circle(frame, (tag_x, tag_y), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID:{best.tag_id}", (tag_x+10, tag_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"ax={math.degrees(filt_ax):+.2f} ay={math.degrees(filt_ay):+.2f}",
                                (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, "QLAND - Precision Landing", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            cv2.imshow("AeroDesign Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        pipe.terminate()
        cv2.destroyAllWindows()
        set_mode(drone, "QLAND")

if __name__ == "__main__":
    main()
