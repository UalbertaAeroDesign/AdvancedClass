import cv2
from pupil_apriltags import Detector
from picamera2 import Picamera2

def main():
    # 1. Setup Camera with Autofocus
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"format": "RGB888", "size": (640, 480)})
    picam2.configure(config)
    
    # This is the "magic" line for Autofocus
    picam2.set_controls({"AfMode": 2}) # 2 = Continuous Autofocus
    
    picam2.start()

    # 2. Setup Detector
    at_detector = Detector(families="tag36h11")

    print("Camera live! Detecting tags...")

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            # Convert RGB to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = at_detector.detect(gray)

            for det in detections:
                print(f"Found Tag ID: {det.tag_id}")
                # (Add your drawing code here from before)

            # Save a frame to check progress (since we are headless)
            if len(detections) > 0:
                cv2.imwrite("detected.jpg", frame)
                print("Detection saved to detected.jpg")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()