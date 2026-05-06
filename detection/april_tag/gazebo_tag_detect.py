import cv2
from pupil_apriltags import Detector

URL = "http://127.0.0.1:8090/feed.mjpg"

det = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,
    refine_edges=True,
    decode_sharpening=0.25,
)

cap = cv2.VideoCapture(URL)
print("Opened:", cap.isOpened())

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = det.detect(gray)

    if tags:
        t = tags[0]
        u, v = t.center
        cv2.putText(frame, f"id={t.tag_id} center=({u:.1f},{v:.1f})",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        pts = t.corners.astype(int)
        cv2.polylines(frame, [pts], True, (0,255,0), 2)

    cv2.imshow("Tag detect", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
