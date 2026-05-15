import cv2
import numpy as np
import time
# TODO: Fix junction not detected issue
# NOTE:
# 1. Setting WIDE_ROW_THRESHOLD to 1 triggers the junction_detected multiple times
# - Add a junction_detected counter X


class LineFollower:
    def __init__(self, camera, drivetrain):
        self.camera     = camera
        self.drivetrain = drivetrain
        self.junction_detected = False
        self.junction_detect_count = 0

        # PID values — tune these
        self.Kp = 0.4
        self.Ki = 0.0
        self.Kd = 0.1
        self.prev_error = 0
        self.integral   = 0


        self.FRAME_SKIP = 5 # Only do detection every FRAME_SKIP frame
        self.BASE_SPEED = 100
        self.WIDE_ROW_THRESHOLD = 1
        self.JUNCTION_WIDTH_RATIO = 0.4  # tune this
        self.JUNCTION_CONFIRM_FRAMES = 5  # frames before junction confirmed

    # ── Main Follow Loop ────────────────────────────
    def follow_until_decision(self, stop_condition=None):
        """ Returns true when a decision point is reached (curve or junction) """
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
                speed_left  = min(BASE_SPEED + correction, 255),
                speed_right = max(BASE_SPEED - correction, -255)
            )

    def follow_until_zone(self):
        # Same algorithm as follow_until_decision
        # Except will detect zone boundaries instead
        # Zone boundary: T junction with black straight and white left & right
        # TODO: Implement this
        pass
    def get_line_error(self, frame) -> int | None:
        # TODO: Replace with implementation from test_visual
        pass

    def get_roi(self, frame, roi_top):
        """Crop top half to remove background noise."""
        h = frame.shape[0]
        return frame[roi_top:h, :]

    # ── PID ─────────────────────────────────────────
    def pid(self, error) -> int:
        self.integral  += error
        derivative      = error - self.prev_error
        self.prev_error = error
        return int((self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative))

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
        raise RuntimeError("handle_line_lost(): Line lost: could not relocate line within timeout")

    # ── Test Function ────────────────────────────────
    def test_visual(self):
        """
        Visualize edge detection without running motors.
        Press Q to quit, S to save snapshot.

        ── Detection modes ───────────────────────────────────────────────────
        Uncomment ONE block under "DETECTION MODE" to switch between methods.

        MODE A: Width threshold (current approach)
        MODE B: Fill ratio / contour shape
        MODE C: Bird's eye view + width threshold
        ──────────────────────────────────────────────────────────────────────
        """
        print("=== Edge Detection Line Follow Test ===")
        print("Q = quit | S = save snapshot")
        print("Green dots  = midpoints per row")
        print("Red dots    = left and right edges per row / flagged rows")
        print("Blue line   = frame center")
        print("Cyan lines  = scan rows")

        snapshot_count = 0

        while True:
            frame = self.camera.capture()
            h, w = frame.shape[:2]
            roi_top = int(h * 0.5)
            roi = self.get_roi(frame, roi_top)
            rh, rw = roi.shape[:2]


            junction_detected = False
            midpoints = []
            wide_row_count = 0
            error = None
            mode_label = ""

            # ══════════════════════════════════════════════════════════════
            # DETECTION MODE — uncomment exactly one block
            # ══════════════════════════════════════════════════════════════
            # ── MODE E: Edge detection + contour centroid + histogram ─────────────
            mode_label = "MODE E: edge + centroid + histogram"
            # TODO: Tune the (JUNCTION_WIDTH_RATIO) threshold for junction pixel threshold as it might vary based on height and angle of the phone camera
            # TODO: Tune roi_top for the same reason

            # LOGIC:
            #    - Edge detection (single row) → PID correction
            #   - Histogram peak width → classifies why edge failed:
            #       wide peak  = junction/turn → stop
            #       narrow peak = genuine line lost → spin to recover

            gray      = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur      = cv2.GaussianBlur(gray, (5, 5), 0)
            edges     = cv2.Canny(blur, 50, 150)

            # ── Mask + threshold ──────────────────────────────────────────────
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 80))  # isolate dark pixels
            _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

            binary = cv2.bitwise_and(binary, mask)
            edges_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)  # show masked binary, not raw edges

            # ── Single scan row — edge detection ──────────────────────────────
            scan_row    = int(rh * 0.8)
            row         = edges[scan_row, :]
            edge_pixels = np.where(row > 0)[0]
            y           = roi_top + scan_row

            cv2.line(frame,       (0, y),        (w,  y),        (255, 255, 0), 1)
            cv2.line(edges_color, (0, scan_row), (rw, scan_row), (255, 255, 0), 1)

            edge_error = None
            if len(edge_pixels) >= 2:
                left_edge  = edge_pixels[0]
                right_edge = edge_pixels[-1]
                midpoint   = (left_edge + right_edge) // 2
                edge_error = midpoint - (rw // 2)
                midpoints.append(midpoint)
                cv2.circle(frame, (left_edge,  y), 5, (0, 0, 255), -1)
                cv2.circle(frame, (right_edge, y), 5, (0, 0, 255), -1)
                cv2.circle(frame, (midpoint,   y), 7, (0, 255, 0), -1)

            # ── Histogram — junction classifier ───────────────────────────────
            scan_band_top = int(rh * 0.6)
            scan_band_bot = int(rh * 0.9)
            band          = binary[scan_band_top:scan_band_bot, :]
            col_sum       = np.sum(band, axis=0) / 255
            peak_width    = int(np.sum(col_sum > 0))
            JUNCTION_WIDTH_THRESHOLD = int(rw * self.JUNCTION_WIDTH_RATIO)

            # Draw histogram on edges window
            for x in range(rw):
                bar_h = int(col_sum[x] * 0.1)
                if bar_h > 0:
                    cv2.line(edges_color, (x, rh), (x, rh - bar_h), (0, 255, 0), 1)

            # Draw scan band on frame
            cv2.rectangle(frame,
                          (0,  roi_top + scan_band_top),
                          (rw, roi_top + scan_band_bot),
                          (255, 165, 0), 1)

            # ── Classify edge failure ──────────────────────────────────────────
            is_junction = peak_width > JUNCTION_WIDTH_THRESHOLD

            # Histogram is always the junction classifier — regardless of edge result
            if is_junction:
                self.junction_detect_count += 1
            else:
                self.junction_detect_count = 0

            # Only apply edge correction when clearly not a junction
            if edge_error is not None and not is_junction:
                error = edge_error

            junction_detected = self.junction_detect_count >= self.JUNCTION_CONFIRM_FRAMES

            # Extra stats for MODE E
            extra_stats = [
                f"peak width:  {peak_width}px  (thresh:{JUNCTION_WIDTH_THRESHOLD})",
                f"edge error:  {'None' if edge_error is None else edge_error}",
                f"is_junction: {is_junction}",
            ]

            # ══════════════════════════════════════════════════════════════
            # DISPLAY — shared across all modes
            # ══════════════════════════════════════════════════════════════

            cv2.putText(frame, mode_label, (10, h - 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 50), 2)

            if error is not None:
                spread = (max(midpoints) - min(midpoints)) if len(midpoints) > 1 else 0
                correction = self.pid(error)
                best_midpoint = int(np.median(midpoints))
                cv2.circle(frame, (best_midpoint + rw // 2, roi_top + int(rh * 0.75)), 10, (0, 255, 255), -1)
                cv2.putText(frame, f"error:        {error}px", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"correction:   {correction:.2f}", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 200, 255), 2)
                cv2.putText(frame, f"L motor:      {self.BASE_SPEED + correction:.0f}", (10, 86),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.putText(frame, f"R motor:      {self.BASE_SPEED - correction:.0f}", (10, 114),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
                cv2.putText(frame, f"midpt spread: {spread}px", (10, 142), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (200, 200, 200), 2)
                cv2.putText(frame, f"wide rows:    {wide_row_count}", (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (200, 200, 200), 2)
                cv2.putText(frame, f"junc count:   {self.junction_detect_count}", (10, 198), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (200, 200, 200), 2)

            else:
                cv2.putText(frame, "LINE LOST", (10, 210),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

            if junction_detected:
                cv2.rectangle(frame, (0, h - 60), (w, h), (0, 100, 200), -1)
                cv2.putText(frame, "JUNCTION DETECTED",
                            (w // 2 - 160, h - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

            cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
            cv2.rectangle(frame, (0, roi_top), (w, h), (0, 255, 255), 2)
            cv2.putText(frame, "ROI", (5, roi_top - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Line Follow - Camera View", frame)
            cv2.imshow("Line Follow - Edges / Binary", edges_color)

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

    cam = Camera(ip="10.42.2.24", flip=-1)   # ← replace with your phone's IP
    lf  = LineFollower(cam, MockDrivetrain())
    lf.test_visual()

"""
Fix 4 — Reduce ROI further
def get_roi(self, frame):
    h = frame.shape[0]
    # Process less pixels = faster
    return frame[int(h * 0.7):h, :]  # only bottom 30%
"""



