import time, math, subprocess
import numpy as np
import cv2
from pupil_apriltags import Detector
from pymavlink import mavutil

# CONFIGURATION PARAMS
MAVLINK_UDP = "udp:127.0.0.1:14550" # Link between python and our drone

SDP_PATH = "gazebo5601.sdp"     # RTP/H264 SDP file (UDP port inside should match your GstCameraPlugin)
W, H = 1280, 720                # Pixel grid size of down facing camera in gazebo. 
HFOV_DEG = 60.0                 # match your <horizontal_fov> in the SDF

TAG_FAMILY = "tag36h11"
DM_MIN = 18.0                   # decision margin threshold aka confidence score. Apriltag confidence should be very high, this is low.

TAKEOFF_ALT_M = 8.0
HOVER_SEC = 2.0 # Hover for this long before entering precision loiter (how long after reaching final waypoint we wait to enter precision loiter)

SEND_RATE_HZ = 15               # Send target coords to drone 15 times a second. Should be low enought to avoid overloading connection
CENTER_AX_OK = 0.03             # Radian measurement of tilt. If we are tilted less than this amount over target, we can consider ourselves to be on top of the target
CENTER_AY_OK = 0.03
CENTER_HOLD_SEC = 2.0           # Must be centered this long before landing (tilt below CENTER_AX_OK and CENTER_AY_OK for this duraiton before entering Precision Landing )

SHOW_VIDEO = True # If you want the downwards camera feed

YAW_KP = 5.0          # P-gain: PWM offset per degree of yaw error (e.g. 24deg * 5 = +120 PWM)
YAW_DEADBAND_DEG = 3.0  # Ignore yaw errors smaller than this

# Precision Loiter enable (Aux function 39)
# IMPORTANT: pick a channel that is NOT already assigned to another Aux function,
# otherwise you can get "Duplicate Aux Switch Options" and arming failures.
#
# Ardupilot is designed to listen to RC channels to enable or disable specific features/params.
# Channel 1-4 usually are for Roll, Pitch, Throttle, Yaw, and ch5 and aboive are for switches.
# We will use channel 7 as our virtual switch to control precision loiter feature. 
# When entering centering phase, we send an rc_channels_override command on channel 7 with a value of 2000 (ON)
# which will activate precision loiter (channel 7 should be mapped to option 39 if you check the param list in QGC/MissionPlanner)
PRECLOITER_CH = 7
PWM_ON  = 2000
PWM_OFF = 1000

# ----------------------------
# MAVLink helpers
# ----------------------------
def set_mode(m, mode_str: str, timeout=5.0):
    mode_str = mode_str.upper() # Pymavlink is picky about this
    mode_mapping = m.mode_mapping() # Name to number map for modes
    if mode_mapping is None or mode_str not in mode_mapping:
        raise RuntimeError(f"Mode {mode_str} not in mode mapping. Available: {list(mode_mapping.keys()) if mode_mapping else 'None'}")

    mode_id = mode_mapping[mode_str]
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(0.5)

# Boilerplate
def set_param(m, name: str, value: float):
    m.mav.param_set_send(
        m.target_system, m.target_component, 
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)

def arm(m, timeout=10.0):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is None:
            continue
        armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        if armed:
            print("ARMED")
            return True
    return False

def takeoff(m, alt_m: float):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        float(alt_m)
    )

def rc_override_one(m, ch: int, pwm: int): # Update only one channel 
    chans = [0]*18
    chans[ch-1] = int(pwm)
    m.mav.rc_channels_override_send(
        m.target_system, m.target_component, *chans
    )

def is_armed(m):
    heartbeat = m.recv_match(type="HEARTBEAT", blocking=False) # Catch the heatbeat... 
    if heartbeat is None: # No news is good news, apparently
        return True
    return (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


# We must simulate the controller stick at neutral in order to properly loiter
RC_HOLD_HZ = 10.0 # If ardupilot doesnt get any controller input for for more than a few seconds it will failsafe. We will send 10 signals a second
_last_rc = 0.0

def send_rc_hold(m, roll=1500, pitch=1500, throttle=1500, yaw=1500, aux_ch=PRECLOITER_CH, aux_pwm=PWM_ON):
    """
    Hold pilot sticks neutral and throttle slightly above mid so LOITER doesn't sink.
    Also keeps PrecisionLoiter aux channel held HIGH.
    """
    chans = [0]*18
    chans[0] = int(roll)       # ch1
    chans[1] = int(pitch)      # ch2
    chans[2] = int(throttle)   # ch3  <-- CRITICAL. This needs to be 1500 or we will descend in loiter mode
    chans[3] = int(yaw)        # ch4
    chans[aux_ch-1] = int(aux_pwm)  # Chosen aux channel (I chose 7)
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *chans)

def maintain_rc_hold(m, now, enable_precloiter=True, yaw=1500):
    global _last_rc
    if now - _last_rc >= 1.0 / RC_HOLD_HZ: # Make sure we send AT MOST 10 commands a second so we dont overwhelm the link
        send_rc_hold(
            m,
            throttle=1500,
            yaw=yaw,  # 0 = passthrough (don't override ch4), 1500 = neutral
            aux_pwm=(PWM_ON if enable_precloiter else PWM_OFF)
        )
        _last_rc = now


def stop_rc_overrides(m):
    """
    Send one neutral override (safest), turn OFF precision loiter then stop sending overrides so that we can actually descend.
    Once the drone notices that it is no longer receiving overrides on throttle (and any other overriden RC), it will revert back to
    its internal landing logic (which for land mode is throttle = 1000 (descending))
    """
    send_rc_hold(m, roll=1500, pitch=1500, throttle=1500, yaw=1500, aux_pwm=PWM_OFF)


# ----------------------------
# Vision / LANDING_TARGET
# ----------------------------
def landing_target_send(m, ax, ay):
    m.mav.landing_target_send(
        int(time.time() * 1e6),
        0,
        12,  # MAV_FRAME_BODY_FRD, this tells drone that angles are al relative ot its nose and belly
        float(ax), float(ay),
        0.0, # Distance in meters... Works fine in sim without, might want to set for actual physical drone
        0.0, 0.0, # Size of target IN RADIANS! This seems counterintuitive but it allows for some weird math that helps us determine our altitude.
        0.0, 0.0, 0.0,
        (0.0, 0.0, 0.0, 1.0),
        2, # # Type: MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL aka AprilTag
        0
    )

def send_condition_yaw(m, yaw_angle_deg, speed_deg_s=10, is_relative=True):
    """
    Commands the drone to change its yaw.
    is_relative=True means 'rotate this many degrees from current heading'
    """
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,                  # confirmation
        abs(yaw_angle_deg), # param 1: target angle (must be absolute value for this command)
        speed_deg_s,        # param 2: angular speed in deg/s
        1 if yaw_angle_deg > 0 else -1, # param 3: direction (-1 CCW, 1 CW)
        1 if is_relative else 0,        # param 4: 1 for relative, 0 for absolute
        0, 0, 0             # param 5-7 not used
    )

# ----------------------------
# FFmpeg pipe reader - Super low latency video feed from gazebo down facing camera
# ----------------------------
def start_ffmpeg_pipe(sdp_path: str):
    cmd = [
    "ffmpeg",
    "-loglevel", "error",
    "-avoid_negative_ts", "make_zero", # Add this
    "-fflags", "nobuffer+discardcorrupt", # Add discardcorrupt
    "-flags", "low_delay",
    "-strict", "experimental",
    "-protocol_whitelist", "file,udp,rtp",
    "-i", sdp_path,
    "-f", "rawvideo",
    "-pix_fmt", "bgr24",
    "-"
]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=10**8)

# ----------------------------
# Main
# ----------------------------
def main():
    # MavLink connection
    m = mavutil.mavlink_connection(MAVLINK_UDP)
    m.wait_heartbeat()
    print("MAVLink heartbeat OK")


    print("Applying param tuning, optimized for SMOOTHNESS (slow, low jitter)")
    # Precision Landing specific (from ArduPilot Docs)
    set_param(m, "PLND_ENABLED", 1)
    set_param(m, "PLND_TYPE", 1)      # MAVLink
    set_param(m, "PLND_EST_TYPE", 1)  # 1 = Kalman Filter (Smooth)
    set_param(m, "PLND_XY_DIST_MAX", 1.0) # Limit correction 'reach'
    set_param(m, "PSC_JERK_NE", 2.0) # THIS IS CRUCIAL FOR LOITER JERKINESS vs SMOOTHNESS
    # Loiter movement limits (Controls the 'Jerk' and 'Aggression')
    set_param(m, "LOIT_ACC_MAX", 50)   # Max acceleration 0.5m/s/s
    set_param(m, "LOIT_SPEED", 20)     # VERY slow loiter speed
    set_param(m, "LOIT_BRK_ACCEL", 25) # Gentle braking


    # April tag detector
    det = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0, refine_edges=True)

    # Intrinsics from HFOV
    cx, cy = W / 2.0, H / 2.0 # Optical center points of image, perfectly center for this simulated camera
    fx = (W / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)
    fy = fx # Again, assuming perfectly square pixels. This wont be true for a real camera
    print(f"Frame {W}x{H}; fx~{fx:.1f}; cx={cx:.1f} cy={cy:.1f}")

    # Start ffmpeg
    p = start_ffmpeg_pipe(SDP_PATH)
    frame_size = W * H * 3

    # SET MODE GUIDED
    print("Setting GUIDED...")
    set_mode(m, "GUIDED")

    print("Arming...")
    if not arm(m):
        raise RuntimeError("Failed to arm (timeout).")

    print(f"Taking off to {TAKEOFF_ALT_M:.1f}m...")
    takeoff(m, TAKEOFF_ALT_M)
    time.sleep(6.0)

    print(f"Hovering {HOVER_SEC:.1f}s...")
    time.sleep(HOVER_SEC)

    # PHASE 1: LOITER + PRECISION LOITER
    print("Priming RC overrides...")
    # Send the override BEFORE the mode change to prevent the initial dip
    send_rc_hold(m, throttle=1500, aux_pwm=PWM_ON)
    time.sleep(0.1) 

    print("Switching to LOITER...")
    set_mode(m, "LOITER")
    
    # Send it again immediately after the mode change
    send_rc_hold(m, throttle=1500, aux_pwm=PWM_ON)
    time.sleep(0.3)


    filt_ax, filt_ay = 0.0, 0.0
    alpha = 1  # [0,1], Lower is heavier / slower but in my experience its best to leave at 1


    centered_since = None
    send_dt = 1.0 / SEND_RATE_HZ
    last_send = 0.0
    last_print = 0.0
    yaw_pwm = 1500  # Neutral until we see the tag

    phase = "PREC_LOITER"
    print("Phase: Precision Loiter centering...")

    while True:
        now = time.time()

        # CRITICAL: keep RC3 + aux held while centering or aligning
        if phase in ("PREC_LOITER", "YAW_ALIGN"):
            maintain_rc_hold(m, now, enable_precloiter=True, yaw=yaw_pwm)

        if not is_armed(m):
            print("Disarmed. Done.")
            break

        # THIS IS A BLOCKING CALL    
        raw = p.stdout.read(frame_size)
        if len(raw) != frame_size:
            print("Frame read failed (ffmpeg ended or stream stalled).")
            break

        # Copy so OpenCV can draw without readonly errors
        frame = np.frombuffer(raw, np.uint8).reshape((H, W, 3)).copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = det.detect(gray, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=0.40)
        best = None
        if tags:
            best = max(tags, key=lambda t: t.decision_margin)

        

        if best and best.decision_margin >= DM_MIN:
            u, v = best.center
            ax = math.atan((u - cx) / fx)
            ay = math.atan((v - cy) / fy)

            R = best.pose_R 
            yaw_error_rad = math.atan2(R[1][0], R[0][0])
            yaw_error_deg = math.degrees(yaw_error_rad)

            # APPLY FILTER
            filt_ax = (alpha * ax) + (1.0 - alpha) * filt_ax
            filt_ay = (alpha * ay) + (1.0 - alpha) * filt_ay

            # 1. SPAM THIS: Send the X/Y precision landing targets at high frequency
            if now - last_send >= send_dt:
                landing_target_send(m, filt_ax, filt_ay) 
                last_send = now

            # 2. Proportional yaw correction via RC ch4 — only active in YAW_ALIGN phase
            if phase == "YAW_ALIGN" and abs(yaw_error_deg) > YAW_DEADBAND_DEG:
                yaw_pwm = int(max(1300, min(1700, 1500 + YAW_KP * yaw_error_deg)))
            else:
                yaw_pwm = 1500

            # Phase transitions
            if phase == "PREC_LOITER":
                # Step 1: center XY only
                if abs(filt_ax) < CENTER_AX_OK and abs(filt_ay) < CENTER_AY_OK:
                    if centered_since is None:
                        centered_since = now
                    elif now - centered_since >= CENTER_HOLD_SEC:
                        print("Centered -> aligning yaw...")
                        phase = "YAW_ALIGN"
                        centered_since = None
                else:
                    centered_since = None

            elif phase == "YAW_ALIGN":
                # Step 2: rotate to align yaw
                if abs(yaw_error_deg) < YAW_DEADBAND_DEG:
                    if centered_since is None:
                        centered_since = now
                    elif now - centered_since >= CENTER_HOLD_SEC:
                        print("Yaw aligned -> switching to LAND")
                        stop_rc_overrides(m)
                        set_mode(m, "LAND")
                        phase = "LAND"
                        centered_since = None
                else:
                    centered_since = None

            if SHOW_VIDEO:
                pts = best.corners.astype(int)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                cv2.circle(frame, (int(u), int(v)), 5, (255, 0, 0), -1)
                cv2.putText(frame, f"dm={best.decision_margin:.1f} ax={ax:+.3f} ay={ay:+.3f}",
                            (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        else:
            centered_since = None
            yaw_pwm = 1500  # No tag — stop any yaw correction

        if SHOW_VIDEO:
            cv2.putText(frame, f"phase: {phase}", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
            cv2.imshow("PrecLoiter -> LAND (pipe)", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                print("ESC pressed, exiting.")
                break

        # Optional: light debug
        if now - last_print > 1.0:
            if best:
                print(f"phase={phase} dm={best.decision_margin:.1f} yaw_err={yaw_error_deg:+.1f}deg")
            else:
                print(f"phase={phase} dm=None")
            last_print = now

    try:
        p.terminate()
    except Exception:
        pass
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
