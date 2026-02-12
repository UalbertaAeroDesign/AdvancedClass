import time
import subprocess
import numpy as np
import cv2
from pymavlink import mavutil

# Config
CONNECTION_STRING = "udp:127.0.0.1:14551"
SDP_FILE = "gazebo5601.sdp"
TARGET_ALTITUDE = 10.0
HOLD_DURATION = 30
RC_HOLD_HZ = 10
PWM_NEUTRAL = 1500


WIDTH, HEIGHT = 1280, 720 

FFMPEG_COMMAND = [
    'ffmpeg',
    '-protocol_whitelist', 'file,udp,rtp',
    '-i', SDP_FILE,
    '-f', 'image2pipe',
    '-pix_fmt', 'bgr24',
    '-vcodec', 'rawvideo', '-'
]

def connect_drone():
    print(f"Connecting to {CONNECTION_STRING}...")
    master = mavutil.mavlink_connection(CONNECTION_STRING)
    master.wait_heartbeat()
    print("Heartbeat received!")
    return master

def set_mode(master, mode):
    mode = mode.upper()
    if mode not in master.mode_mapping():
        return
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system, 1, mode_id)
    print(f"Mode set to {mode}")

def arm_drone(master):
    print("Waiting for GPS fix...")
    master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print("GPS Fix confirmed. Arming...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print("Armed!")

def takeoff_vtol(master, altitude):
    print(f"Initiating VTOL Takeoff to {altitude}m...")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

# Need this to hold altitude in QLOITER
def send_rc_hold(master, throttle=1500):
    master.mav.rc_channels_override_send(master.target_system, master.target_component,
                                         65535, 65535, throttle, 65535, 65535, 65535, 65535, 65535)

def main():
    drone = connect_drone()
    set_mode(drone, 'GUIDED')
    arm_drone(drone)
    takeoff_vtol(drone, TARGET_ALTITUDE)

    print("Climbing...")
    current_alt = 0
    while current_alt < TARGET_ALTITUDE * 0.95:
        msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f" Current Alt: {current_alt:.2f}m", end='\r')
        time.sleep(0.1)

    print("\nOpening FFmpeg pipe and switching to QLOITER...")
    pipe = subprocess.Popen(FFMPEG_COMMAND, stdout=subprocess.PIPE, bufsize=WIDTH*HEIGHT*3*10)
    set_mode(drone, 'QLOITER')

    start_hold = time.time()
    last_rc_send = 0
    
    try:
        while time.time() - start_hold < HOLD_DURATION:
            now = time.time()
            if now - last_rc_send >= (1.0 / RC_HOLD_HZ):
                send_rc_hold(drone, throttle=PWM_NEUTRAL)
                last_rc_send = now

            n_bytes = WIDTH * HEIGHT * 3
            raw_image = pipe.stdout.read(n_bytes)
            
            if len(raw_image) != n_bytes:
                continue

            frame = np.frombuffer(raw_image, dtype='uint8').reshape((HEIGHT, WIDTH, 3)).copy()

            # Display and check for quit
            cv2.putText(frame, f"ALT: {current_alt:.1f}m", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('MiniHawk Downward Cam', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"Error in vision loop: {e}")
    finally:
        print("\nClosing video and landing...")
        pipe.terminate()
        cv2.destroyAllWindows()
        send_rc_hold(drone, throttle=0) 
        set_mode(drone, 'QLAND')

if __name__ == "__main__":
    main()