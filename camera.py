import cv2
import numpy as np
import os
from collections import deque, Counter

class Camera:
    def __init__(self, ip: str, port: int = 8080, flip: bool = False):
        self.url = f"http://{ip}:{port}/video"
        self.cap = cv2.VideoCapture(self.url)
        self.flip = flip

        # Letter recognition state
        self.templates = self._load_templates()
        self.detection_buffer = deque(maxlen=15)

        # Tunable params (replaces trackbars)
        self.darkness = 80
        self.min_area = 300
        self.max_area = 15000
        self.match_cutoff = 0.45
        self.margin_cutoff = 0.05

    def capture(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to grab frame from IP webcam")

        if self.flip:
            frame = cv2.flip(frame, 0)  # 0 = flip along horizontal axis (long axis)
        return frame

    # ── Label Reading ────────────────────────────────
    def read_label(self, frame) -> str | None:
        """
        Run letter recognition on a single frame.
        Returns the stable letter string (e.g. 'B') or None if not confident.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, letter_mask = cv2.threshold(gray, self.darkness, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_OPEN, kernel)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(letter_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_letter = None
        best_score = -1.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_area <= area <= self.max_area):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w < 10 or h < 10:
                continue

            aspect = w / float(h)
            if aspect < 0.3 or aspect > 2.0:
                continue

            extent = area / float(w * h)
            if extent < 0.15:
                continue

            blob = letter_mask[y:y + h, x:x + w]
            if blob.size == 0:
                continue

            processed_roi = self._normalize_letter(blob)

            scored = []
            for letter, rotations in self.templates.items():
                letter_best = max(
                    float(np.max(cv2.matchTemplate(processed_roi, rot, cv2.TM_CCOEFF_NORMED)))
                    for rot in rotations
                )
                scored.append((letter_best, letter))
            scored.sort(reverse=True)

            top_score = scored[0][0]
            top_letter = scored[0][1]
            second_score = scored[1][0] if len(scored) > 1 else -1.0

            if top_score < self.match_cutoff:
                continue
            if top_score - second_score < self.margin_cutoff:
                continue

            if top_score > best_score:
                best_score = top_score
                best_letter = top_letter

        # Temporal voting — only return a result when buffer is stable
        if best_letter:
            self.detection_buffer.append(best_letter)
            counts = Counter(self.detection_buffer)
            stable_letter, count = counts.most_common(1)[0]
            if count / len(self.detection_buffer) >= 0.6:  # 60% agreement
                return stable_letter
        else:
            self.detection_buffer.clear()

        return None

    def detect_cubes(self, frame) -> bool:
        """Returns True if any letter-sized blob is visible."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.darkness, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(self.min_area <= cv2.contourArea(c) <= self.max_area for c in contours)

    def release(self):
        self.cap.release()

    # ── Internal helpers ─────────────────────────────
    def _normalize_letter(self, binary_img, size=50):
        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return cv2.resize(binary_img, (size, size))
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        tight = binary_img[y:y + h, x:x + w]
        side = max(w, h)
        square = np.zeros((side, side), dtype=binary_img.dtype)
        square[(side - h) // 2:(side - h) // 2 + h,
        (side - w) // 2:(side - w) // 2 + w] = tight
        return cv2.resize(square, (size, size))

    def _load_templates(self, folder="letter recognition/templates", size=(50, 50)):
        templates = {}
        if not os.path.exists(folder):
            print(f"Warning: template folder '{folder}' not found.")
            return templates
        for filename in os.listdir(folder):
            if filename.lower().endswith((".png", ".jpg", ".jpeg")):
                letter = filename.split('.')[0].upper()
                img = cv2.imread(os.path.join(folder, filename), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                img = cv2.flip(img, 1)
                _, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
                base = self._normalize_letter(thresh, size[0])
                templates[letter] = [np.rot90(base, k) for k in range(4)]
        print(f"Loaded {len(templates)} templates: {list(templates.keys())}")
        return templates
