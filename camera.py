import cv2

class Camera:
    def __init__(self, ip: str, port: int = 8080, flip: bool = False):
        self.url = f"http://{ip}:{port}/video"
        self.cap = cv2.VideoCapture(self.url)
        self.flip = flip

    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to grab frame from IP webcam")

        if self.flip:
            frame = cv2.flip(frame, 0)  # 0 = flip along horizontal axis (long axis)
        return frame

    def read_label(self, frame) -> str:
        """Detect cube letter — OCR or color based."""
        # Your cube detection logic here
        pass

    def detect_cubes(self, frame) -> bool:
        """Returns True if cubes are visible in frame."""
        pass

    def release(self):
        self.cap.release()