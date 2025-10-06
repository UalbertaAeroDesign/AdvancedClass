from ultralytics import YOLO
import cv2

# Load the trained model
model = YOLO("/opt/homebrew/runs/detect/train/weights/best.pt")  # Update path if needed, I included PT file in this repo so you can set it to that

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Failed to open camera.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO model
    results = model(frame, stream=True)

    # Draw bounding boxes
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = model.names[cls]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Hat Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC key to quit
        break

cap.release()
cv2.destroyAllWindows()
