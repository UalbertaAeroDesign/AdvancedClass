import cv2
import numpy as np

def detect_white_square_cv2(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_rect = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 500:
            continue

        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            # Compute angle between edges to check for right angles
            def angle_cos(p0, p1, p2):
                p0, p1, p2 = p0.reshape(-1), p1.reshape(-1), p2.reshape(-1)
                d1, d2 = p0 - p1, p2 - p1
                return abs(np.dot(d1, d2) / (np.linalg.norm(d1) * np.linalg.norm(d2)))

            cosines = []
            for i in range(4):
                cosines.append(angle_cos(approx[i], approx[(i + 1) % 4], approx[(i + 2) % 4]))

            max_cosine = np.max(cosines)

            # Require angles near 90 (cosine close to 0)
            if max_cosine < 0.2:
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                if 0.3 < aspect_ratio < 3.5 and area > max_area:
                    best_rect = approx
                    max_area = area

    conf = 0
    if best_rect is not None:
        conf = 1
        cv2.polylines(frame, [best_rect], True, (255, 0, 0), 2)
        M = cv2.moments(best_rect)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})", (cx - 40, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 0, 200), 1)

    return conf, frame


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open camera or video.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        conf, processed = detect_white_square_cv2(frame)
        #cv2.imshow("White Rectangle Detection", processed)

        key = cv2.waitKey(10)
        if key == 27:  # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
