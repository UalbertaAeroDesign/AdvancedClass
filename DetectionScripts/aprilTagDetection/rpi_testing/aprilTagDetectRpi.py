import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2

def main():
    # Initialize Camera
    picam2 = Picamera2()
    # 640x480 is best for speed; increase to (1280, 720) if you need more detail
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2}) # Continuous Autofocus
    picam2.start()

    # Initialize Detector
    at_detector = Detector(families="tag36h11")

    print("Running... Press 'q' on the Pi's keyboard (or Ctrl+C in terminal) to exit.")

    try:
        while True:
            # Capture and convert for OpenCV
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect
            detections = at_detector.detect(gray)

            for det in detections:
                # Draw center
                cx, cy = int(det.center[0]), int(det.center[1])
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Draw boundaries
                corners = det.corners.astype(int)
                for i in range(4):
                    p1 = tuple(corners[i])
                    p2 = tuple(corners[(i+1) % 4])
                    cv2.line(frame, p1, p2, (255, 0, 0), 2)

                # Label ID
                cv2.putText(frame, f"ID:{det.tag_id}", (cx + 10, cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # SHOW THE FRAME ON THE MONITOR
            cv2.imshow("AeroDesign AprilTag Detection", frame)

            # Check for 'q' key to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()