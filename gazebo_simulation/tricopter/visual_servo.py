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
HFOV_DEG = 60.0

# Throttle tuning
QCLIMB_THROTTLE = 1700     
QHOVER_THROTTLE = 1500     
QTHROTTLE_MIN = 1300
QTHROTTLE_MAX = 1900

# --- REFINED PID GAINS ---
KP_ROLL = 220.0    
KP_PITCH = 220.0   
KI_ROLL = 35.0     
KI_PITCH = 35.0    
KD_ROLL = 70.0     
KD_PITCH = 70.0    

ALPHA = 0.4  # Smoothing factor

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

def connect_drone():
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print("Heartbeat received!")
    time.sleep(1)
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

def get_rel_alt_m(master):
    m = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    return m.relative_alt / 1000.0 if m else None

def send_rc_movement(master, roll_pwm, pitch_pwm, throttle_pwm):
    roll_pwm = clamp(int(roll_pwm), 1100, 1900)
    pitch_pwm = clamp(int(pitch_pwm), 1100, 1900)
    throttle_pwm = clamp(int(throttle_pwm), QTHROTTLE_MIN, QTHROTTLE_MAX)
    chans = [65535] * 18
    chans[0], chans[1], chans[2], chans[3] = roll_pwm, pitch_pwm, throttle_pwm, 1500
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)

def vtol_climb_to_alt(master, target_alt_m):
    print(f"Climbing to {target_alt_m}m...")
    while True:
        alt = get_rel_alt_m(master)
        if alt and alt >= target_alt_m: break
        send_rc_movement(master, 1500, 1500, QCLIMB_THROTTLE)
        time.sleep(0.1)
    return True

def main():
    drone = connect_drone()
    set_mode(drone, "QLOITER")
    arm_drone(drone)
    vtol_climb_to_alt(drone, TARGET_ALTITUDE)

    print("\nEntering PID Tracking Loop with Visuals...")
    pipe = subprocess.Popen(FFMPEG_COMMAND, stdout=subprocess.PIPE, bufsize=W * H * 3 * 10)

    filt_ax, filt_ay = 0.0, 0.0
    int_ax, int_ay = 0.0, 0.0
    prev_ax, prev_ay = 0.0, 0.0
    
    last_loop_time = time.time()
    last_rc_send = 0.0
    start_time = time.time()

    try:
        while time.time() - start_time < HOLD_DURATION:
            now = time.time()
            dt = now - last_loop_time
            if dt <= 0: dt = 0.01

            raw = pipe.stdout.read(W * H * 3)
            if len(raw) != W * H * 3: continue

            frame = np.frombuffer(raw, dtype="uint8").reshape((H, W, 3)).copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = at_detector.detect(gray)

            cmd_roll, cmd_pitch = 1500, 1500

            if detections:
                best = max(detections, key=lambda t: t.decision_margin)
                if best.decision_margin > 15:
                    u, v = best.center
                    
                    # 1. Error calculation
                    ax = math.atan((u - CAM_CX) / FX)
                    ay = math.atan((v - CAM_CY) / FY)

                    # 2. Filter
                    filt_ax = (ALPHA * ax) + (1.0 - ALPHA) * filt_ax
                    filt_ay = (ALPHA * ay) + (1.0 - ALPHA) * filt_ay

                    # 3. Integral (I)
                    int_ax += filt_ax * dt
                    int_ay += filt_ay * dt
                    int_ax = clamp(int_ax, -0.4, 0.4)
                    int_ay = clamp(int_ay, -0.4, 0.4)

                    # 4. Derivative (D)
                    d_ax = (filt_ax - prev_ax) / dt
                    d_ay = (filt_ay - prev_ay) / dt
                    prev_ax, prev_ay = filt_ax, filt_ay

                    # 5. PID Sum
                    cmd_roll = 1500 + (filt_ax * KP_ROLL) + (int_ax * KI_ROLL) + (d_ax * KD_ROLL)
                    cmd_pitch = 1500 + (filt_ay * KP_PITCH) + (int_ay * KI_PITCH) + (d_ay * KD_PITCH)

                    # --- VISUALS ---
                    tag_x, tag_y = int(u), int(v)
                    # Green Bounding Box
                    pts = best.corners.astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    
                    # Tag ID text
                    cv2.putText(frame, f"ID: {best.tag_id}", (tag_x - 40, tag_y - 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    # Red Center-to-Target Arrow
                    cv2.arrowedLine(frame, (int(CAM_CX), int(CAM_CY)), (tag_x, tag_y), (0, 0, 255), 3, tipLength=0.1)
                    
                    # Output Overlay
                    cv2.rectangle(frame, (10, 10), (320, 100), (0,0,0), -1) # Background box
                    cv2.putText(frame, f"ROLL: {int(cmd_roll)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                    cv2.putText(frame, f"PITCH: {int(cmd_pitch)}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            else:
                int_ax, int_ay = 0.0, 0.0
                prev_ax, prev_ay = 0.0, 0.0

            if now - last_rc_send >= (1.0 / RC_HOLD_HZ):
                send_rc_movement(drone, cmd_roll, cmd_pitch, QHOVER_THROTTLE)
                last_rc_send = now
            
            last_loop_time = now
            cv2.imshow("AeroDesign Vision PID", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"): break

    finally:
        pipe.terminate()
        cv2.destroyAllWindows()
        set_mode(drone, "QLAND")

if __name__ == "__main__":
    main()