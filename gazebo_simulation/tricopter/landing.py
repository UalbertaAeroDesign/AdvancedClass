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
SEND_RATE_HZ = 15

W, H = 1280, 720
HFOV_DEG = 60.0

QCLIMB_THROTTLE = 1700     
QHOVER_THROTTLE = 1500     
QTHROTTLE_MIN = 1300
QTHROTTLE_MAX = 1900

# --- PID GAINS (For Alignment Phase) ---
KP_ROLL = 220.0    
KP_PITCH = 220.0   
KI_ROLL = 35.0     
KI_PITCH = 35.0    
KD_ROLL = 70.0     
KD_PITCH = 70.0    
ALPHA = 0.4  

# --- ALIGNMENT LOGIC ---
REQUIRED_ALIGN_TIME = 3.0      # Seconds the tag must be centered before descending
ALIGN_THRESHOLD_RAD = 0.035     # ~4.5 degrees of allowable error to be considered "centered"
STABILITY_THRESHOLD = 0.05     # Maximum allowable angular velocity (ensures it is hovering still)

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
    print("Heartbeat received!")
    time.sleep(1)

    print("Configuring PLND Parameters...")
    set_param(master, "PLND_ENABLED", 1)
    set_param(master, "PLND_TYPE", 1)     
    set_param(master, "PLND_STRICT", 0)   
    
    # --- NEW: QUADPLANE DESCENT SPEEDS ---
    print("Configuring Descent Speeds...")
    set_param(master, "Q_WP_SPEED_DN", 70)       # Drop from 7m at a slow 70 cm/s
    set_param(master, "Q_LAND_FINAL_SPD", 0.3)   # Final touchdown at 0.3 m/s
    set_param(master, "Q_LAND_FINAL_ALT", 2.5)   # Start the final slow touchdown phase at 2.5 meters high
    
    return master

def set_mode(master, mode):
    mode = mode.upper()
    mm = master.mode_mapping()
    if mode not in mm: return False
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

def send_rc_movement(master, roll_pwm, pitch_pwm, throttle_pwm):
    chans = [65535] * 18
    chans[0] = clamp(int(roll_pwm), 1100, 1900)
    chans[1] = clamp(int(pitch_pwm), 1100, 1900)
    chans[2] = clamp(int(throttle_pwm), QTHROTTLE_MIN, QTHROTTLE_MAX)
    chans[3] = 1500
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)

def release_rc_controls(master):
    chans = [0] * 18
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)
    print("RC Overrides Released.")

def send_landing_target(master, ax, ay, dist_m):
    safe_dist = float(dist_m) if dist_m is not None else TARGET_ALTITUDE
    master.mav.landing_target_send(
        0, 0, 12, float(ax), float(ay), safe_dist, 
        0.0, 0.0, 0.0, 0.0, 0.0, [1.0, 0.0, 0.0, 0.0], 2, 0
    )

def vtol_climb_to_alt(master, target_alt_m):
    print(f"Climbing to {target_alt_m}m...")
    while True:
        m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if m and (m.relative_alt / 1000.0) >= target_alt_m: break
        send_rc_movement(master, 1500, 1500, QCLIMB_THROTTLE)
        time.sleep(0.1)
    return True

def main():
    drone = connect_drone()
    set_mode(drone, "QLOITER")
    arm_drone(drone)
    vtol_climb_to_alt(drone, TARGET_ALTITUDE)

    print("\nEntering Hybrid Tracking Loop...")
    pipe = subprocess.Popen(FFMPEG_COMMAND, stdout=subprocess.PIPE, bufsize=W * H * 3 * 10)

    # State Variables
    flight_state = "ALIGNING"
    aligned_time_start = None
    land_start_time = None
    
    filt_ax, filt_ay = 0.0, 0.0
    int_ax, int_ay, prev_ax, prev_ay = 0.0, 0.0, 0.0, 0.0
    current_alt_m = TARGET_ALTITUDE
    
    last_loop_time = time.time()
    last_rc_send = 0.0
    last_plnd_send = 0.0

    try:
        while True:
            now = time.time()
            dt = now - last_loop_time
            if dt <= 0: dt = 0.01

            # Drain altitude messages to keep current_alt_m fresh
            while True:
                msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if not msg: break
                current_alt_m = msg.relative_alt / 1000.0

            # --- Landing Completion Check ---
            if flight_state == "LANDING" and (now - land_start_time > 5.0):
                hb = drone.recv_match(type='HEARTBEAT', blocking=False)
                if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print("Disarm detected! Touchdown successful.")
                    break

            raw = pipe.stdout.read(W * H * 3)
            if len(raw) != W * H * 3:
                # Keep QLOITER alive if waiting for frames
                if flight_state == "ALIGNING" and now - last_rc_send >= (1.0 / RC_HOLD_HZ):
                    send_rc_movement(drone, 1500, 1500, QHOVER_THROTTLE)
                    last_rc_send = now
                continue

            frame = np.frombuffer(raw, dtype="uint8").reshape((H, W, 3)).copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            cmd_roll, cmd_pitch = 1500, 1500
            target_seen = False

            if detections:
                best = max(detections, key=lambda t: t.decision_margin)
                if best.decision_margin > 15:
                    target_seen = True
                    u, v = best.center
                    
                    ax = math.atan((u - CAM_CX) / FX)
                    ay = math.atan((v - CAM_CY) / FY)

                    filt_ax = (ALPHA * ax) + (1.0 - ALPHA) * filt_ax
                    filt_ay = (ALPHA * ay) + (1.0 - ALPHA) * filt_ay

                    tag_x, tag_y = int(u), int(v)
                    cv2.polylines(frame, [best.corners.astype(int)], True, (0, 255, 0), 3)
                    cv2.arrowedLine(frame, (int(CAM_CX), int(CAM_CY)), (tag_x, tag_y), (0, 0, 255), 3, tipLength=0.1)

            # ==========================================
            # STATE 1: ALIGNING (PID Control)
            # ==========================================
            if flight_state == "ALIGNING":
                if target_seen:
                    # PID Math
                    int_ax = clamp(int_ax + filt_ax * dt, -0.4, 0.4)
                    int_ay = clamp(int_ay + filt_ay * dt, -0.4, 0.4)
                    d_ax = (filt_ax - prev_ax) / dt
                    d_ay = (filt_ay - prev_ay) / dt
                    prev_ax, prev_ay = filt_ax, filt_ay

                    cmd_roll = 1500 + (filt_ax * KP_ROLL) + (int_ax * KI_ROLL) + (d_ax * KD_ROLL)
                    cmd_pitch = 1500 + (filt_ay * KP_PITCH) + (int_ay * KI_PITCH) + (d_ay * KD_PITCH)

                    # Check if we are close to the center AND not swinging/drifting
                    if (abs(filt_ax) < ALIGN_THRESHOLD_RAD and abs(filt_ay) < ALIGN_THRESHOLD_RAD and
                        abs(d_ax) < STABILITY_THRESHOLD and abs(d_ay) < STABILITY_THRESHOLD):
                        
                        if aligned_time_start is None:
                            aligned_time_start = now
                        else:
                            # If aligned for long enough, transition to LANDING
                            align_duration = now - aligned_time_start
                            cv2.putText(frame, f"ALIGNING: {align_duration:.1f}s / {REQUIRED_ALIGN_TIME}s", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                            
                            if align_duration >= REQUIRED_ALIGN_TIME:
                                print("\n>>> TARGET ALIGNED & STABLE! Initiating ArduPilot QLAND... <<<")
                                flight_state = "LANDING"
                                release_rc_controls(drone)
                                set_mode(drone, "QLAND")
                                land_start_time = now
                    else:
                        aligned_time_start = None # Reset timer if it drifts OR swings too fast
                else:
                    aligned_time_start = None
                    int_ax, int_ay, prev_ax, prev_ay = 0.0, 0.0, 0.0, 0.0

                if now - last_rc_send >= (1.0 / RC_HOLD_HZ):
                    send_rc_movement(drone, cmd_roll, cmd_pitch, QHOVER_THROTTLE)
                    last_rc_send = now

                cv2.rectangle(frame, (10, 10), (400, 80), (0,0,0), -1) 
                cv2.putText(frame, f"STATE: ALIGNING", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                cv2.putText(frame, f"Alt: {current_alt_m:.1f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # ==========================================
            # STATE 2: LANDING (ArduPilot PLND)
            # ==========================================
            elif flight_state == "LANDING":
                if target_seen and (now - last_plnd_send >= (1.0 / SEND_RATE_HZ)):
                    send_landing_target(drone, filt_ax, filt_ay, current_alt_m)
                    last_plnd_send = now

                cv2.rectangle(frame, (10, 10), (450, 80), (0,0,0), -1) 
                cv2.putText(frame, f"STATE: DESCENDING (PLND)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
                cv2.putText(frame, f"Alt: {current_alt_m:.1f}m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            last_loop_time = now
            cv2.imshow("Hybrid Precision Land", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"): break

    finally:
        pipe.terminate()
        cv2.destroyAllWindows()
        release_rc_controls(drone)

if __name__ == "__main__":
    main()