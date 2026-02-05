from ultralytics import YOLO
import cv2

# Load your trained model
model = YOLO(r"./detection_models/box_best.pt")

def detect_white_square_yolo(frame):
    # Run YOLO model on the current frame
    results = model(frame, stream=True, verbose=False)

    # Draw all boxes, regardless of confidence

    conf_scores = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])*100 # get percentage value instead of decimal
            conf_scores += [conf]
            cls = int(box.cls[0]) # grabs the class index but I have only one class which is the whitebox
            label = model.names.get(cls) #grabs the id of the class using the class index

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(frame, f"{label} {conf:.1f}%",(x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    conf = 0
    if conf_scores:
        conf = max(conf_scores) /100
    
    return conf, frame

if __name__ == '__main__':
    # Open webcam (CAP_DSHOW is for windows users, delete second arg for MAC)
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW) 
    if not cap.isOpened():
        print("Failed to open camera")
        raise SystemExit

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Run YOLO model on the current frame
        score, frame = detect_white_square_yolo(frame)
        #print("Score = %f", score)

        # Show webcam feed
        cv2.imshow("WhiteBox Detection", frame)

        # Press ESC to quit
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()




                