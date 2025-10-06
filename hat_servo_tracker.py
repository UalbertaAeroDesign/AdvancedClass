from ultralytics import YOLO
import cv2
import serial
from pymavlink import mavutil

# MAVLink helpers
def open_serial(port="/dev/cu.usbserial-DN04T9FH", baud=57600):
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
        print(f"Opened serial port {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"Serial error: {e}")
        return None

def move_servo(ser, servo_num, pwm, target_sys=1, target_comp=1):
    mav = mavutil.mavlink.MAVLink(ser)
    mav.srcSystem = 1
    mav.srcComponent = 1
    msg = mav.command_long_encode(
        target_sys, target_comp,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(servo_num), float(pwm),
        0, 0, 0, 0, 0
    )
    ser.write(msg.pack(mav)); ser.flush()
    print(f"Servo {servo_num} moved to {pwm}")




def main():
    # Serial link to FC (close QGC first if it’s using the same port)
    ser = open_serial("/dev/cu.usbserial-DN04T9FH", 57600) # This will be different depeneding on whos running this 
    if not ser:
        return

    # Load the trained yolo model
    model = YOLO("/opt/homebrew/runs/detect/train/weights/best.pt")  # Change if needed, best.pt is in this directory too

    # Open webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera.")
        ser.close()
        return

    # control params
    SERVO_NUM   = 4        # Servo# we will move
    GAIN        = 1.2      # Pixels → PWM slope
    DEADBAND_PX = 20       # ignore tiny errors to avoid buzz
    PWM_MIN, PWM_MAX = 1000, 2000
    prev_pwm    = 1500

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            # 4) Run YOLO using YOUR working pattern
            #    (you can add conf=0.6, iou=0.5 if you want)
            results = model(frame, stream=True)

            # 5) Use the most confident detection (if any)
            best = None
            best_conf = -1.0
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf > best_conf:
                        best_conf = conf
                        best = box

            if best is not None:
                x1, y1, x2, y2 = map(int, best.xyxy[0])
                cx, cy = (x1 + x2)//2, (y1 + y2)//2

                # draw
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.circle(frame, (cx,cy), 4, (0,255,0), -1)

                # 6) Vertical center tracking → PWM
                center_y = frame.shape[0] // 2
                err = cy - center_y
                if abs(err) < DEADBAND_PX:
                    pwm = 1500
                else:
                    pwm = int(1500 - err * GAIN)
                pwm = max(PWM_MIN, min(PWM_MAX, pwm))

                if pwm != prev_pwm:
                    move_servo(ser, SERVO_NUM, pwm)
                    prev_pwm = pwm

            # HUD
            h, w = frame.shape[:2]
            cv2.line(frame, (0, h//2), (w, h//2), (255,255,255), 1)
            cv2.putText(frame, f"PWM {prev_pwm}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

            cv2.imshow("Hat Detection + Servo", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        ser.close()
        print("Exited — camera and serial closed.")

if __name__ == "__main__":
    main()
