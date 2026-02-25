# DISCLAIMER: In order for this script to work, you MUST have the plane_precland.lua script inside your ardupilot scripts directory!
# To do this, follow these steps:
# 1. In ardupilot root directory, do a git pull to make sure your version is current. 
# 2. Navigate to libraries/AP_Scripting/applets/plane_precland.lua from ardupilot root dir to confirm the existence of plane_precland.lua.
# 3. Run "cp libraries/AP_Scripting/applets/plane_precland.lua scripts/" from inside ardupilots root directory. This will copy plane_precland.lua from libraries/AP_Scripting/applets into scripts.

import time, math, subprocess
import numpy as np
import cv2
from pymavlink import mavutil
from pupil_apriltags import Detector

# Config
CONNECTION_STRING = "udp:127.0.0.1:14551"
SDP_FILE = "gazebo5601.sdp"

TARGET_ALTITUDE = 7.0
HOLD_DURATION = 120
RC_HOLD_HZ = 10

W, H = 1280, 720
SEND_RATE_HZ = 20
HFOV_DEG = 60.0

# Throttle tuning (important for QLOITER altitude hold behavior)
QCLIMB_THROTTLE = 1700     # used to climb
QHOVER_THROTTLE = 1500     # used to "hold" altitude in QLOITER.;
QTHROTTLE_MIN = 1300
QTHROTTLE_MAX = 1900

# Camera intrinsics
CAM_CX, CAM_CY = W / 2.0, H / 2.0
FX = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
FY = FX

FFMPEG_COMMAND = [
    "ffmpeg", "-loglevel", "quiet",
    "-protocol_whitelist", "file,udp,rtp",
    "-i", SDP_FILE,
    "-f", "image2pipe", "-pix_fmt", "bgr24",
    "-vcodec", "rawvideo", "-"
]

at_detector = Detector(families="tag36h11", nthreads=2, quad_decimate=2.0)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_param(m, name, value):
    m.mav.param_set_send(
        m.target_system, m.target_component,
        name.encode("ascii"), float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

def connect_drone():
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print("Heartbeat received! Waiting for EKF3 to initialize...")

    # Listen to the statustext messages from the drone to know ahrs is stable
    # while True:
    #     msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
    #     if msg:
    #         text = msg.text
    #         if isinstance(text, bytes):
    #             text = text.decode('utf-8')
            
    #         # Break the loop once the EKF is active
    #         if "EKF3 active" in text or "Origin set" in text:
    #             print("EKF3 healthy, navigation is online.")
    #             break
                
    time.sleep(1)

    print("Setting Parameters...")
    set_param(master, "RC7_OPTION", 39)   # PrecLoiter enable
    set_param(master, "PLND_ENABLED", 1)
    set_param(master, "PLND_TYPE", 1)     # MAVLink LANDING_TARGET
    set_param(master, "PLND_STRICT", 0)
    set_param(master, "PLND_ACC_P_NSE", 0.8)
    set_param(master, "Q_LOIT_SPEED_MS", 0.5)
    set_param(master, "Q_LOIT_ACC_MAX_M", 1)
    set_param(master, "Q_LOIT_ACC_MAX_M", 1)

    set_param(master, "Q_P_NE_POS_P", 0.5)
    set_param(master, "Q_P_JERK_NE", 1)
    # 1. Soften the position logic - Doesnt seem to make much a difference
    set_param(master, "Q_P_POS_XY_P", 0.5)   # Lower: Drone won't rush to the center
    set_param(master, "Q_V_VEL_XY_P", 0.3)   # Lower: Softens the braking/acceleration
    set_param(master, "Q_V_VEL_XY_I", 0.05)  # Very low: Prevents "pendulum" wind-up

    # Limit the physical energy
    set_param(master, "Q_LOIT_ACC_MAX", 100) # Max 1m/s^2 acceleration
    set_param(master, "Q_LOIT_BRK_JERK", 250)# Slow down the snappiness when stopping
    # Precision Landing Gain
    set_param(master, "PLND_GAIN", 0.5)
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

def get_rel_alt_m(master):
    msg = get_latest_msg(master, "GLOBAL_POSITION_INT")
    if msg is None:
        return None
    return msg.relative_alt / 1000.0

def get_height_agl_m(master):
    # Prefer TERRAIN_REPORT.current_height (best estimate of height above terrain)
    tr = get_latest_msg(master, "TERRAIN_REPORT")
    if tr is not None:
        return float(tr.current_height)

    # GLOBAL_POSITION_INT.relative_alt (height above home) is less reliable but need a fallback
    gp = get_latest_msg(master, "GLOBAL_POSITION_INT")
    if gp is not None:
        return gp.relative_alt / 1000.0

    return None

def landing_target_send_angles_only(m, ax, ay, dist_m):
    """
    In the same way as with the quadcopter, send angle offsets to FC
    which consumes them along with current heigh in order to adjust its position safely 
    """
    # If we lose distance tracking momentarily, guess the target alt.
    safe_dist = float(dist_m) if dist_m is not None else TARGET_ALTITUDE

    m.mav.landing_target_send(
        int(time.time() * 1e6),      # time_usec
        0,                           # target_num
        12,                          # frame (dont change this)
        float(ax), float(ay),        # angle_x, angle_y
        safe_dist,                   # distance (used for scaling)
        0.0, 0.0,                    # size_x, size_y  
        0.0, 0.0, 0.0,               # x, y, z (Ignored by position_valid=0)
        [1.0, 0.0, 0.0, 0.0],        # q (Quaternion)
        2,                           # type (april tag/fudicial marker)
        0                            # position_valid = 0 (Forces angles only!)
    )

def send_rc_hold(master, throttle_pwm, precloiter_switch=2000):
    throttle_pwm = clamp(int(throttle_pwm), QTHROTTLE_MIN, QTHROTTLE_MAX)

    chans = [65535] * 18
    chans[0] = 1500  # roll
    chans[1] = 1500  # pitch
    chans[2] = throttle_pwm
    chans[3] = 1500  # yaw
    chans[6] = int(precloiter_switch)  # ch7 (PrecLoiter)

    master.mav.rc_channels_override_send(
        master.target_system, master.target_component, *chans
    )

def vtol_climb_to_alt(master, target_alt_m, timeout=25.0):
    print(f"Climbing to {target_alt_m:.1f}m...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        alt = get_rel_alt_m(master)
        if alt is not None and alt >= target_alt_m:
            print(f"Reached {alt:.1f}m")
            return True
        send_rc_hold(master, throttle_pwm=QCLIMB_THROTTLE, precloiter_switch=1000)
        time.sleep(0.1)
    print("Climb timeout.")
    return False

def main():
    drone = connect_drone()

    set_mode(drone, "QLOITER")
    arm_drone(drone)

    # climb
    vtol_climb_to_alt(drone, TARGET_ALTITUDE, timeout=25.0)

    print("\nEntering QLOITER AprilTag tracking loop\n")
    pipe = subprocess.Popen(FFMPEG_COMMAND, stdout=subprocess.PIPE, bufsize=W * H * 3 * 10)

    start_hold = time.time()
    last_rc_send = 0.0
    last_send = 0.0

    filt_ax, filt_ay = 0.0, 0.0
    alpha = 1
    send_dt = 1.0 / SEND_RATE_HZ

    try:
        last_dist = TARGET_ALTITUDE   # fallback distance
        while time.time() - start_hold < HOLD_DURATION:
            now = time.time()

            # Keep throttle + aux asserted at 10Hz (prevents descent in QLOITER)
            if now - last_rc_send >= (1.0 / RC_HOLD_HZ):
                send_rc_hold(drone, throttle_pwm=QHOVER_THROTTLE, precloiter_switch=2000)
                last_rc_send = now

            raw = pipe.stdout.read(W * H * 3)
            if len(raw) != W * H * 3:
                continue

            frame = np.frombuffer(raw, dtype="uint8").reshape((H, W, 3)).copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            if detections:
                best = max(detections, key=lambda t: t.decision_margin)
                if best.decision_margin > 15:
                    u, v = best.center

                    ax = math.atan((u - CAM_CX) / FX)
                    ay = math.atan((v - CAM_CY) / FY)

                    filt_ax = (alpha * ax) + (1.0 - alpha) * filt_ax
                    filt_ay = (alpha * ay) + (1.0 - alpha) * filt_ay
                    
                    if now - last_send >= send_dt:
                        dist = get_height_agl_m(drone) 
                        if dist is not None and dist > 0.1:
                            last_dist = dist
                        
                        print(f"ax={math.degrees(filt_ax):.2f} deg | ay={math.degrees(filt_ay):.2f} deg | dist={last_dist:.2f} m")
                        landing_target_send_angles_only(drone, filt_ax, filt_ay, last_dist)
                        last_send = now

                    tag_x, tag_y = int(u), int(v)
                    pts = best.corners.astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                    cv2.circle(frame, (tag_x, tag_y), 5, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID:{best.tag_id}", (tag_x+10, tag_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.imshow("AeroDesign Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        pipe.terminate()
        cv2.destroyAllWindows()
        set_mode(drone, "QLAND")

if __name__ == "__main__":
    main()