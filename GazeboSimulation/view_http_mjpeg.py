# This script is now irrelevant (precision_land_full runs a ffmpeg pipe, thus no need for mjpeg viewer)
import cv2

url = "http://127.0.0.1:8091/feed.mjpg"
cap = cv2.VideoCapture(url)
print("Opened:", cap.isOpened())

while True:
    ok, frame = cap.read()
    if not ok:
        continue
    cv2.imshow("Gazebo MJPEG", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
