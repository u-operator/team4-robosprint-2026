import cv2
import numpy as np
import time

class LineFollower:
    def __init__(self, camera, drivetrain):
        self.camera     = camera
        self.drivetrain = drivetrain

        # PID values — tune these
        self.Kp = 0.4
        self.Ki = 0.0
        self.Kd = 0.1
        self.prev_error = 0
        self.integral   = 0
        self.FRAME_SKIP = 5 # Only do detection every FRAME_SKIP frame
        self.junction_detected = False

    # ── Main Follow Loop ────────────────────────────
    def follow(self, stop_condition=None):
        """ Follow line until a decision point (curve or junction)"""
        frame_count = 0
        while True:
            if self.junction_detected:
                self.drivetrain.stop()
                return True

            frame = self.camera.capture()
            frame_count += 1

            # Detect every FRAME_SKIP frame
            if frame_count % self.FRAME_SKIP != 0:
                continue

            error = self.get_line_error(frame)

            if error is None:
                try:
                    self.handle_line_lost()
                except RuntimeError:
                    return None
                self.reset_pid()
                continue


            correction = self.pid(error)
            self.drivetrain.set_motors(
                speed_left  = 100 + correction,
                speed_right = 100 - correction
            )

    # ── Line Detection (Edge Based) ──────────────────
    # def get_line_error(self, frame) -> float:
    #     """
    #     Detects left and right edges of the line.
    #     Returns how far the midpoint between edges is from frame center.
    #     Negative = line is left, Positive = line is right.
    #     Returns None if edges not found.
    #     """
    #     roi = self.get_roi(frame)
    #     h, w = roi.shape[:2]
    #
    #     gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    #     blur  = cv2.GaussianBlur(gray, (5, 5), 0)
    #     edges = cv2.Canny(blur, 50, 150)  # tweak these thresholds if needed
    #
    #     # Scan a horizontal slice near the bottom of the ROI
    #     # (closest part of line to robot = most reliable)
    #     scan_row = int(h * 0.8)
    #     row      = edges[scan_row, :]
    #
    #     # Find all edge pixel positions in that row
    #     edge_pixels = np.where(row > 0)[0]
    #
    #     if len(edge_pixels) < 2:
    #         return None  # not enough edges found
    #
    #     left_edge  = edge_pixels[0]         # leftmost edge
    #     right_edge = edge_pixels[-1]        # rightmost edge
    #     midpoint   = (left_edge + right_edge) // 2
    #
    #     frame_center = w // 2
    #     error        = midpoint - frame_center
    #
    #     return error

    def get_line_error(self, frame) -> float:
        roi = self.get_roi(frame)
        h, w = roi.shape[:2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Scan multiple rows instead of just one
        scan_rows = [
            int(h * 0.6),
            int(h * 0.7),
            int(h * 0.8),
            int(h * 0.9),
        ]

        midpoints = []
        wide_row_count = 0  # ← counts how many rows look like a junction

        for scan_row in scan_rows:
            row = edges[scan_row, :]
            edge_pixels = np.where(row > 0)[0]

            if len(edge_pixels) < 2:
                continue

            # Filter out rows where line seems too wide (junction horizontal bar)
            left_edge = edge_pixels[0]
            right_edge = edge_pixels[-1]
            line_width = right_edge - left_edge

            # If line is suspiciously wide it's probably the junction bar — skip it
            if line_width > w * 0.5:
                wide_row_count += 1  # ← flag this row as junction-like
                continue  # still skip for line following

            midpoint = (left_edge + right_edge) // 2
            midpoints.append(midpoint)

        # Junction detected?
        self.junction_detected = wide_row_count >= 2

        if not midpoints:
            return None  # LINE LOST

        # Use median to ignore any outlier rows
        best_midpoint = int(np.median(midpoints))
        return best_midpoint - (w // 2) # Error

    def get_roi(self, frame):
        """Crop top half to remove background noise."""
        h = frame.shape[0]
        return frame[int(h * 0.5):h, :]

    # ── PID ─────────────────────────────────────────
    def pid(self, error) -> float:
        self.integral  += error
        derivative      = error - self.prev_error
        self.prev_error = error
        return (self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative)

    def reset_pid(self):
        self.prev_error = 0
        self.integral   = 0

    # ── Line Lost Handling ───────────────────────────
    def handle_line_lost(self, timeout=2.0):
        self.drivetrain.stop()
        start_time = time.time()

        # Spin slowly to search for the line
        while time.time() - start_time < timeout:
            self.drivetrain.set_motors(speed_left=-30, speed_right=30)  # spin in place

            frame = self.camera.capture()
            error = self.get_line_error(frame)

            if error is not None:
                self.reset_pid()  # clear stale PID state before resuming
                return  # line found, resume follow() loop normally

        # Timed out — couldn't find line
        self.drivetrain.stop()
        raise RuntimeError("Line lost: could not relocate line within timeout")




    # ── Test Function ────────────────────────────────
    def test_visual(self):
        """
        Visualize edge detection without running motors.
        Press Q to quit, S to save snapshot.
        """
        print("=== Edge Detection Line Follow Test ===")
        print("Q = quit | S = save snapshot")
        print("Green dot   = midpoint between edges")
        print("Red dots    = left and right edges")
        print("Blue line   = frame center")
        print("Yellow line = scan row")

        snapshot_count = 0

        while True:
            frame = self.camera.capture()
            h, w  = frame.shape[:2]
            roi   = self.get_roi(frame)
            rh, rw = roi.shape[:2]
            roi_top = int(h * 0.5)

            # --- Edge detection ---
            gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur  = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            # --- Scan row ---
            scan_row    = int(rh * 0.8)
            row         = edges[scan_row, :]
            edge_pixels = np.where(row > 0)[0]

            # Convert edges to color for display
            edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

            # Draw scan row line on edge view
            cv2.line(edges_color,
                     (0,  scan_row),
                     (rw, scan_row),
                     (0, 255, 255), 1)

            if len(edge_pixels) >= 2:
                left_edge  = edge_pixels[0]
                right_edge = edge_pixels[-1]
                midpoint   = (left_edge + right_edge) // 2
                error      = midpoint - (rw // 2)
                correction = self.pid(error)

                # Draw on full frame (adjust y by roi_top)
                y = roi_top + scan_row

                # Red dot — left edge
                cv2.circle(frame, (left_edge, y), 8, (0, 0, 255), -1)
                cv2.putText(frame, "L", (left_edge - 15, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Red dot — right edge
                cv2.circle(frame, (right_edge, y), 8, (0, 0, 255), -1)
                cv2.putText(frame, "R", (right_edge + 5, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Green dot — midpoint
                cv2.circle(frame, (midpoint, y), 10, (0, 255, 0), -1)

                # Line width indicator
                line_width = right_edge - left_edge
                cv2.line(frame, (left_edge, y + 15), (right_edge, y + 15),
                         (0, 255, 0), 2)

                # Stats overlay
                cv2.putText(frame, f"error:      {error}px",       (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"correction: {correction:.2f}", (10, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
                cv2.putText(frame, f"L motor:    {100 + correction:.0f}", (10, 86),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.putText(frame, f"R motor:    {100 - correction:.0f}", (10, 114),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.putText(frame, f"line width: {line_width}px",   (10, 142),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

            else:
                cv2.putText(frame, "LINE LOST", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            # Junction banner — big and obvious
            if self.junction_detected:
                # Dark orange background box
                cv2.rectangle(frame, (0, h - 60), (w, h), (0, 100, 200), -1)
                cv2.putText(frame, "JUNCTION DETECTED",
                            (w // 2 - 160, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            # Blue center line
            cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)

            # Yellow scan row line on full frame
            cv2.line(frame, (0, roi_top + scan_row), (w, roi_top + scan_row),
                     (0, 255, 255), 1)

            # ROI box
            cv2.rectangle(frame, (0, roi_top), (w, h), (0, 255, 255), 2)
            cv2.putText(frame, "ROI", (5, roi_top - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Line Follow - Camera View", frame)
            cv2.imshow("Line Follow - Edges",       edges_color)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                snapshot_count += 1
                cv2.imwrite(f"snapshot_{snapshot_count}.jpg", frame)
                cv2.imwrite(f"snapshot_{snapshot_count}_edges.jpg", edges_color)
                print(f"Saved snapshot_{snapshot_count}")

        cv2.destroyAllWindows()
        print("Test ended.")


# ── Run directly ─────────────────────────────────
if __name__ == "__main__":
    from camera import Camera

    class MockDrivetrain:
        def stop(self): pass
        def set_motors(self, speed_left, speed_right): pass

    cam = Camera(ip="2.58.120.241", flip=-1)   # ← replace with your phone's IP
    lf  = LineFollower(cam, MockDrivetrain())
    lf.test_visual()


"""
Issue 1 — T Junction & Curves
The problem is scanning only one row — at a T junction the horizontal bar creates many edge pixels that confuse the left/right edge detection:
normal line:          T junction scan row hits:
    |                 ────────────────
    |                 ← too many edges, error spikes
    |                 
  L   R               L             R
  ●   ●               ●             ●
  midpoint ok         midpoint way off
  
Fix — Multi-row scanning + vote for most consistent result:
def get_line_error(self, frame) -> float:
    roi = self.get_roi(frame)
    h, w = roi.shape[:2]

    gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Scan multiple rows instead of just one
    scan_rows = [
        int(h * 0.6),
        int(h * 0.7),
        int(h * 0.8),
        int(h * 0.9),
    ]

    midpoints = []

    for scan_row in scan_rows:
        row         = edges[scan_row, :]
        edge_pixels = np.where(row > 0)[0]

        if len(edge_pixels) < 2:
            continue

        # Filter out rows where line seems too wide (junction horizontal bar)
        left_edge  = edge_pixels[0]
        right_edge = edge_pixels[-1]
        line_width = right_edge - left_edge

        # If line is suspiciously wide it's probably the junction bar — skip it
        if line_width > w * 0.5:
            continue

        midpoint = (left_edge + right_edge) // 2
        midpoints.append(midpoint)

    if not midpoints:
        return None  # LINE LOST

    # Use median to ignore any outlier rows
    best_midpoint = int(np.median(midpoints))
    return best_midpoint - (w // 2)
Why this works at junctions and curves:
T junction:              Curve:
                    
row 0.6: ──────── ← wide, SKIPPED     row 0.6:   /  ← ok
row 0.7:   |      ← ok ✓              row 0.7:  /   ← ok
row 0.8:   |      ← ok ✓              row 0.8: /    ← ok
row 0.9:   |      ← ok ✓              row 0.9:|     ← ok

median of ok rows = stable midpoint   median = smooth curve tracking

Issue 2 — Speed
Yes this is a real concern. Here's the breakdown:
StepApprox timecamera.capture() over WiFi~50–100ms ← biggest bottleneckcv2.cvtColor + GaussianBlur~2–5mscv2.Canny~3–8msMulti-row scan~1msTotal~60–120ms per frame
That's only 8–15 FPS which may be too slow for fast motor correction.
Fix 1 — Reduce resolution in IP Webcam app
640x480 → too slow
320x240 → sweet spot ✓
Lower resolution = less data over WiFi = faster capture.
Fix 2 — Resize frame immediately after capture
pythondef capture(self):
    ret, frame = self.cap.read()
    if not ret:
        raise RuntimeError("No frame from IP webcam")

    # Resize immediately — all processing after this is faster
    frame = cv2.resize(frame, (320, 240))

    if self.flip:
        frame = cv2.rotate(frame, cv2.ROTATE_180)

    return frame
Fix 3 — Skip frames
pythondef follow(self, stop_condition=None):
    frame_count = 0

    while True:
        if stop_condition and stop_condition():
            self.drivetrain.stop()
            break

        frame = self.camera.capture()
        frame_count += 1

        # Only run detection every 2nd frame
        if frame_count % 2 != 0:
            continue

        error = self.get_line_error(frame)
        ...
Fix 4 — Reduce ROI further
pythondef get_roi(self, frame):
    h = frame.shape[0]
    # Process less pixels = faster
    return frame[int(h * 0.7):h, :]  # only bottom 30%

Realistic Speed After Fixes
Fix appliedEstimated FPSBaseline 640x480~8 FPSResize to 320x240~15 FPSSmaller ROI~20 FPSSkip every 2nd frame~25 FPS effective
20–25 FPS should be sufficient for line following at moderate robot speed. If your robot moves slowly, even 15 FPS is fine.

One More Suggestion — Slow Down at Junctions
Instead of trying to perfectly handle junctions in vision, just slow the motors when a junction is detected:
pythondef get_line_error(self, frame):
    # ... existing code ...

    # If valid rows are fewer than expected, probably near junction
    if len(midpoints) < 2:
        self.junction_detected = True
    else:
        self.junction_detected = False

    return best_midpoint - (w // 2)

def follow(self, stop_condition=None):
    while True:
        ...
        error      = self.get_line_error(frame)
        correction = self.pid(error)

        # Slow down near junction for more stable detection
        speed = 60 if self.junction_detected else 100

        self.drivetrain.set_motors(
            speed_left  = speed + correction,
            speed_right = speed - correction
        )
"""

