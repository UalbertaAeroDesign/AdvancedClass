import threading
import cv2

class CameraStream:
    def __init__(self, sdp_file):
        # OpenCV can read SDP files directly, bypassing the need for subprocess!
        self.cap = cv2.VideoCapture(sdp_file, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Limit buffer to 1 frame
        
        self.ret = False
        self.frame = None
        self.running = True
        
        # Start the background thread
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        # Constantly grab the newest frame as fast as possible
        while self.running:
            self.ret, self.frame = self.cap.read()

    def read(self):
        return self.ret, self.frame

    def stop(self):
        self.running = False
        self.thread.join()
        self.cap.release()