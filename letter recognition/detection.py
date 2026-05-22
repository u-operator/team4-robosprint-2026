import cv2
import numpy as np
import os
from collections import deque, Counter

# Taken from Wei Jen Github

def nothing(x):
    pass

def normalize_letter(binary_img, size=50):
    """Tight-crop the largest blob, center it in a square canvas, resize to size x size.

    Applied identically to templates and live candidates so both sides of the
    match are scaled and positioned the same way.
    """
    contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return cv2.resize(binary_img, (size, size))

    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    tight = binary_img[y:y + h, x:x + w]

    side = max(w, h)
    square = np.zeros((side, side), dtype=binary_img.dtype)
    off_x = (side - w) // 2
    off_y = (side - h) // 2
    square[off_y:off_y + h, off_x:off_x + w] = tight
    return cv2.resize(square, (size, size))

def load_templates(template_folder="letter recognition/templates", size=(50, 50)):
    """Loads and preprocesses all letter templates from the target folder."""
    templates = {}
    if not os.path.exists(template_folder):
        print(f"Warning: Folder '{template_folder}' not found. Please create it and add your letter images.")
        return templates

    for filename in os.listdir(template_folder):
        if filename.endswith((".png", ".jpg", ".jpeg")):
            # Extract the letter from the filename (e.g., 'B.png' -> 'B')
            letter = filename.split('.')[0].upper()
            img_path = os.path.join(template_folder, filename)

            template_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if template_img is not None:
                template_img = cv2.flip(template_img, 1)
                # Binarize the template so it's strictly black and white
                _, thresh = cv2.threshold(template_img, 127, 255, cv2.THRESH_BINARY_INV)
                base = normalize_letter(thresh, size[0])

                # NEW (rotation invariance): pre-compute all four 90-degree
                # rotations of each template. Stored as a list so the matcher
                # can test every orientation per letter. np.rot90(base, k)
                # rotates k * 90 degrees counter-clockwise. Each rotation is
                # already a square (size x size) so dimensions stay consistent.
                templates[letter] = [
                    base,                  # 0 deg
                    np.rot90(base, 1),     # 90 deg
                    np.rot90(base, 2),     # 180 deg
                    np.rot90(base, 3),     # 270 deg
                ]

    print(f"Loaded {len(templates)} templates: {list(templates.keys())}")
    return templates

def detect_and_match():
    # cap = cv2.VideoCapture("http://localhost:8080/video")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not access webcam")
        return

    # 1. Load templates and define logic sets before the camera loop starts
    templates = load_templates()
    if templates:
        # NEW (rotation invariance): templates[letter] is now a list of four
        # rotated images instead of a single image. Build a 4-row preview where
        # each row shows the same rotation across all letters
        # (row 0 = 0deg, row 1 = 90deg, row 2 = 180deg, row 3 = 270deg).
        rows = []
        for rot_idx in range(4):
            row_imgs = [templates[letter][rot_idx] for letter in templates]
            rows.append(cv2.hconcat(row_imgs))
        template_display = cv2.vconcat(rows)

        # Create a window to show the black and white templates
        cv2.imshow("Loaded Templates (Black & White)", template_display)

    real_cubes = ['B', 'C', 'E', 'M', 'R', 'U']
    fake_cubes = ['H', 'N', 'O', 'P']

    cv2.namedWindow("Trackbars")

    # Darkness threshold: pixels with grayscale value BELOW this are treated as
    # letter (white in the mask). 0 = only the blackest pixels count;
    # 255 = everything counts. Letters are fully black so a low/mid-value works.
    cv2.createTrackbar("Darkness", "Trackbars", 80, 255, nothing)

    # Candidate blob area limits (pixels)
    cv2.createTrackbar("MinArea", "Trackbars", 300, 20000, nothing)
    cv2.createTrackbar("MaxArea", "Trackbars", 15000, 50000, nothing)

    # Minimum template-match score to accept a blob as a real letter (x/100)
    cv2.createTrackbar("Match%", "Trackbars", 45, 100, nothing)

    # Minimum gap between the best and 2nd-best template score (x/100).
    # Kills ambiguous "R vs M" style close calls — raise if still confusing,
    # lower if good letters are being rejected as ambiguous.
    cv2.createTrackbar("Margin%", "Trackbars", 5, 50, nothing)

    detection_buffer = deque(maxlen=15)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        letter_detected_this_frame = False

        darkness = cv2.getTrackbarPos("Darkness", "Trackbars")
        min_area = cv2.getTrackbarPos("MinArea", "Trackbars")
        max_area = cv2.getTrackbarPos("MaxArea", "Trackbars")
        match_cutoff = cv2.getTrackbarPos("Match%", "Trackbars") / 100.0
        margin_cutoff = cv2.getTrackbarPos("Margin%", "Trackbars") / 100.0

        # Pixels darker than `darkness` become white (letter foreground).
        _, letter_mask = cv2.threshold(gray, darkness, 255, cv2.THRESH_BINARY_INV)

        # Clean up noise and join broken strokes
        kernel = np.ones((3, 3), np.uint8)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_OPEN, kernel)
        letter_mask = cv2.morphologyEx(letter_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(letter_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Track the single strongest candidate this frame for the voting buffer
        best_frame_letter = None
        best_frame_score = -1
        best_frame_box = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w < 10 or h < 10:
                continue

            # Reject long thin streaks; letters are roughly square-ish
            aspect = w / float(h)
            if aspect < 0.3 or aspect > 2.0:
                continue

            # Solidity-ish filter: letters fill a decent fraction of their bbox
            extent = area / float(w * h)
            if extent < 0.15:
                continue

            if not templates:
                continue

            # Tight-crop the candidate blob (no extra padding) and normalize it
            # exactly like the templates: center in a square, then resize to 50x50.
            blob = letter_mask[y:y + h, x:x + w]
            if blob.size == 0:
                continue

            processed_roi = normalize_letter(blob, 50)

            # Match against every template; track the top two scores so we can
            # reject ambiguous picks.
            #
            # NEW (rotation invariance): each template entry is now a list of
            # four rotated versions. For each letter we score all four
            # orientations and keep the best one. That per-letter best is what
            # competes for "winning letter" — so the same letter rotated 90/180
            # /270 deg still maps to a single entry in `scored`.
            scored = []
            best_rot_per_letter = {}
            for letter, rotations in templates.items():
                letter_best = -1.0
                letter_best_rot = 0
                for rot_idx, rot_img in enumerate(rotations):
                    res = cv2.matchTemplate(processed_roi, rot_img, cv2.TM_CCOEFF_NORMED)
                    score = float(np.max(res))
                    if score > letter_best:
                        letter_best = score
                        letter_best_rot = rot_idx
                scored.append((letter_best, letter))
                best_rot_per_letter[letter] = letter_best_rot
            scored.sort(reverse=True)

            best_score, best_letter = scored[0]
            second_best_score = scored[1][0] if len(scored) > 1 else -1.0

            # The match score itself is what filters out noise blobs here
            if best_score < match_cutoff:
                continue

            # Require a clear lead over the runner-up, otherwise abstain
            if best_score - second_best_score < margin_cutoff:
                continue

            # NEW (rotation invariance): include the winning rotation in the
            # on-screen label so it's obvious which orientation matched.
            best_rot_deg = best_rot_per_letter[best_letter] * 90

            # Draw the accepted candidate box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, f"{best_letter}@{best_rot_deg} {best_score:.2f}",
                        (x, max(12, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            if best_score > best_frame_score:
                best_frame_score = best_score
                best_frame_letter = best_letter
                best_frame_box = (x, y, w, h)

        # --- Temporal Voting (Sampling) ---
        if best_frame_letter is not None:
            detection_buffer.append(best_frame_letter)
            letter_detected_this_frame = True

            counts = Counter(detection_buffer)
            stable_letter, count = counts.most_common(1)[0]
            stability = (count / len(detection_buffer)) * 100

            status = ""
            color = (255, 255, 255)
            if stable_letter in real_cubes:
                status = "- REAL"
                color = (0, 255, 0)
            elif stable_letter in fake_cubes:
                status = "- FAKE"
                color = (0, 0, 255)

            display_text = f"Result: {stable_letter} ({stability:.0f}% stable) {status}"
            print(display_text)

            bx, by, _, _ = best_frame_box
            cv2.putText(frame, display_text, (bx, max(20, by - 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        if not letter_detected_this_frame:
            detection_buffer.clear()

        # Darkness threshold readout (top-left of Frame)
        gray_val = int(darkness)
        cv2.rectangle(frame, (10, 10), (50, 50), (gray_val, gray_val, gray_val), -1)
        cv2.rectangle(frame, (10, 10), (50, 50), (0, 0, 0), 1)
        cv2.putText(frame, f"Darkness <= {darkness}", (55, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Frame", frame)
        cv2.imshow("Letter Mask", letter_mask)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_and_match()