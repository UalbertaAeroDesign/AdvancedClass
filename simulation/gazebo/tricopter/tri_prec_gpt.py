import time
from pymavlink import mavutil

CONNECTION_STRING = "udp:127.0.0.1:14551"

PWM_NEUTRAL = 1500
QTHROTTLE_MIN = 1300
QTHROTTLE_MAX = 1900

CLIMB_THROTTLE = 1800
HOLD_THROTTLE  = 1550

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def connect_drone():
    m = mavutil.mavlink_connection(CONNECTION_STRING)
    m.wait_heartbeat()
    print("Heartbeat received! Waiting for EKF3 to initialize...")

    # Listen to the STATUSTEXT messages from the drone
    # while True:
    #     msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
    #     if msg:
    #         # Decode the text message
    #         text = msg.text
    #         if isinstance(text, bytes):
    #             text = text.decode('utf-8')
            
    #         # Optional: Print the drone's internal logs to your python console
    #         # print(f"Drone Log: {text}")
            
    #         # Break the loop once the EKF is active
    #         if "EKF3 active" in text or "Origin set" in text:
    #             print(">>> EKF3 IS ACTIVE! Navigation is online. <<<")
    #             break
                
    # Give it one extra second to settle
    time.sleep(1)
    return m

def set_mode(master, mode):
    mode = mode.upper()
    mm = master.mode_mapping()
    if mode not in mm:
        raise RuntimeError(f"Mode {mode} not supported. Have: {list(mm.keys())}")
    master.mav.set_mode_send(master.target_system, 1, mm[mode])

def arm(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("Armed.")

def rc_override(master, roll=1500, pitch=1500, throttle=1500, yaw=1500):
    throttle = clamp(int(throttle), QTHROTTLE_MIN, QTHROTTLE_MAX)
    chans = [65535]*18
    chans[0] = int(roll)
    chans[1] = int(pitch)
    chans[2] = int(throttle)
    chans[3] = int(yaw)
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *chans)

def get_local(master):
    # 1. Drain the buffer! Throw away all the old messages piling up.
    while True:
        stale_msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if stale_msg is None:
            break  # The queue is now empty!

    # 2. Wait for a brand new, fresh message to arrive
    fresh_msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
    
    if fresh_msg is None:
        return None
        
    # x North (m), y East (m), z Down (m)
    return float(fresh_msg.x), float(fresh_msg.y), float(fresh_msg.z)

def main():
    m = connect_drone()
    arm(m)

    print("Switching to QSTABILIZE (direct stick attitude)...")
    set_mode(m, "QLOITER")
    time.sleep(2)

    # Climb for ~4s
    print("Climbing...")
    t0 = time.time()
    while time.time() - t0 < 4.0:
        rc_override(m, roll=1500, pitch=1500, throttle=CLIMB_THROTTLE, yaw=1500)
        time.sleep(0.1)

    # Settle 1s
    t0 = time.time()
    while time.time() - t0 < 1.0:
        rc_override(m, roll=1500, pitch=1500, throttle=HOLD_THROTTLE, yaw=1500)
        time.sleep(0.1)

    p0 = get_local(m)
    print(f"Start LOCAL_POSITION_NED: {p0}")

    # Big forward pitch for 5s
    # (If your pitch direction is reversed, swap 1700<->1300)
    print("Commanding BIG forward pitch for 5 seconds...")
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rc_override(m, roll=1500, pitch=1700, throttle=HOLD_THROTTLE, yaw=1500)
        time.sleep(0.1)

    p1 = get_local(m)
    print(f"End   LOCAL_POSITION_NED: {p1}")

    if p0 and p1:
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        print(f"Delta: dx={dx:.2f} m, dy={dy:.2f} m (should be NONZERO if physics is right)")

    print("Done. Switching to QLAND.")
    set_mode(m, "QLAND")

if __name__ == "__main__":
    main()