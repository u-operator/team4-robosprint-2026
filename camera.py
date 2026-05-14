import cv2
import numpy as np
import os
from collections import deque, Counter

# TODO: Test whether the complicated functions work
# TODO: Test function
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
        Single-frame label read with temporal voting across calls.
        Returns a stable letter once the detection buffer reaches 60% agreement,
        otherwise None.

        Use this for repeated scanning (e.g. robot stopped in front of a cube).
        Use find_best_cube() when you need to compare multiple cubes at once.
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
        """
        Returns True if any letter-sized blob is visible.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, self.darkness, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(self.min_area <= cv2.contourArea(c) <= self.max_area for c in contours)

    def detect_all_labels(self, frame) -> list[dict]:
        """
        Detects all visible letter blobs in the frame.

        Returns a list of candidate dicts, each with:
            {
                'letter': str,          # matched letter e.g. 'B'
                'score':  float,        # template match confidence 0–1
                'bbox':   (x, y, w, h), # bounding box in frame pixels
            }

        No temporal voting is applied here — every call is independent.
        Use find_best_cube() for a stable, filtered result.

        LIMITATION: A fully hidden cube produces no blob and is never returned.
        A partially obstructed cube may be misread if its visible area is too
        small for a clean template match — it will usually score below
        match_cutoff and be silently dropped, which is the safe failure mode.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, letter_mask = cv2.threshold(gray, self.darkness, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_OPEN, kernel)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(letter_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.min_area <= area <= self.max_area):
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w < 10 or h < 10:
                continue

            # Reject long thin streaks; letters are roughly square-ish
            aspect = w / float(h)
            if aspect < 0.3 or aspect > 2.0:
                continue

            # Solidity filter: letters fill a decent fraction of their bbox
            extent = area / float(w * h)
            if extent < 0.15:
                continue

            blob = letter_mask[y:y + h, x:x + w]
            if blob.size == 0:
                continue

            processed_roi = self._normalize_letter(blob)

            # Score all templates (all 4 rotations per letter, keep best per letter)
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

            candidates.append({
                'letter': top_letter,
                'score': top_score,
                'bbox': (x, y, w, h),
            })

        return candidates

    def find_best_cube(self, frame, real_cubes: list[str]) -> str | None:
        """
        Returns the label of the closest, unobstructed real cube visible in
        the frame, or None if no valid candidate is found.

        HOW IT WORKS
        ────────────
        1. Detect all letter blobs via detect_all_labels().
        2. Filter to real cubes only.
        3. Filter out blobs that appear obstructed (another blob sits directly
           below them and overlaps horizontally — i.e. is between them and the
           camera).
        4. Pick the closest unobstructed cube using vertical position:
           larger (y + h) = lower in the frame = physically closer to robot.

        RELIABILITY
        ───────────
        This is reliable under the following conditions:
          ✓ Camera is fixed at a forward-facing angle (not top-down)
          ✓ Cubes sit on a flat surface at roughly the same depth plane
          ✓ Cubes are physically distinct objects (not stacked)
          ✓ Lighting is consistent enough for the darkness threshold to work

        Known limitations:
          ✗ Fully hidden cubes are invisible — never returned (safe failure)
          ✗ Two cubes at nearly the same distance may swap order frame-to-frame
            (use temporal voting at the caller level if stability matters)
          ✗ Obstruction check is 2D bounding-box only — a cube that is beside
            another but appears to overlap in the 2D projection may be wrongly
            flagged as obstructed. Rare in practice with well-separated cubes.
          ✗ Blob area is NOT used for distance — vertical position is. If the
            camera is mounted top-down this heuristic breaks entirely.
        """
        candidates = self.detect_all_labels(frame)

        # Step 1 — keep only real cubes
        real = [c for c in candidates if c['letter'] in real_cubes]
        if not real:
            return None

        # Step 2 — filter obstructed cubes
        all_bboxes = [c['bbox'] for c in candidates]
        unobstructed = [c for c in real if self._is_unobstructed(c['bbox'], all_bboxes)]

        # If everything looks obstructed (edge case), fall back to all real cubes
        # rather than returning None — the obstruction check can have false positives
        pool = unobstructed if unobstructed else real

        # Step 3 — closest = lowest in frame (largest y + h)
        closest = max(pool, key=lambda c: c['bbox'][1] + c['bbox'][3])
        return closest['letter']

    def find_cube(self, frame, target_label: str) -> tuple[int, int] | tuple[None, None]:
        """
        Returns the (cx, cy) center point of the target cube in the frame.
        Returns (None, None) if the label is not visible.
        Used by align_to_cube() for horizontal centering.
        """
        candidates = self.detect_all_labels(frame)
        for c in candidates:
            if c['letter'] == target_label:
                x, y, w, h = c['bbox']
                return x + w // 2, y + h // 2
        return None, None

    def get_cube_box(self, frame, target_label: str) -> tuple | None:
        """
        Returns the (x, y, w, h) bounding box of the target cube.
        Returns None if not visible.
        Used by approach_cube() to judge distance via box height.
        """
        candidates = self.detect_all_labels(frame)
        for c in candidates:
            if c['letter'] == target_label:
                return c['bbox']
        return None

    def release(self):
        self.cap.release()

    # ── Internal helpers ─────────────────────────────
    def _normalize_letter(self, binary_img, size=50):
        """
        Tight-crops the largest blob, centres it in a square canvas, and
        resizes to (size x size). Applied identically to templates and live
        candidates so both sides of the match are scaled consistently.
        """
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

    def _is_unobstructed(self, bbox, all_bboxes):
        """
        A cube is obstructed if another bbox overlaps its lower half
        (the region between it and the camera).
        """
        x1, y1, w1, h1 = bbox
        bottom1 = y1 + h1

        for x2, y2, w2, h2 in all_bboxes:
            if (x2, y2, w2, h2) == (x1, y1, w1, h1):
                continue
            # Does the other box sit BELOW and overlapping horizontally?
            if y2 + h2 > bottom1 and x2 < x1 + w1 and x2 + w2 > x1:
                return False
        return True
