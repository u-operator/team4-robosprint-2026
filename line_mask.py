# Goal
#

import cv2
import numpy as np

def nothing(x):
    pass

# ============================================================
# IMAGE SOURCE — uncomment ONE option only
# ============================================================

# OPTION 1: Webcam (built-in or USB)
SOURCE = 0

# OPTION 2: IP Webcam (install "IP Webcam" app on Android)
# SOURCE = "http://192.168.x.x:8080/video"

# OPTION 3: Static image for testing
# SOURCE = "test_line.jpg"

# ============================================================

def get_capture(source):
    if isinstance(source, str) and source.endswith((".jpg", ".png", ".jpeg")):
        return None, cv2.imread(source)
    return cv2.VideoCapture(source), None


def detect_black_line():
    cap, static_image = get_capture(SOURCE)

    if cap is not None and not cap.isOpened():
        print("Error: Could not open video source:", SOURCE)
        return

    cv2.namedWindow("Trackbars")

    cv2.createTrackbar("LH", "Trackbars", 0,   180, nothing)
    cv2.createTrackbar("UH", "Trackbars", 180, 180, nothing)
    cv2.createTrackbar("LS", "Trackbars", 0,   255, nothing)
    cv2.createTrackbar("US", "Trackbars", 80,  255, nothing)
    cv2.createTrackbar("LV", "Trackbars", 0,   255, nothing)
    cv2.createTrackbar("UV", "Trackbars", 80,  255, nothing)

    while True:
        if static_image is not None:
            frame = static_image.copy()
        else:
            ret, frame = cap.read()
            if not ret:
                print("Error: Lost connection to source")
                break
            frame = cv2.flip(frame, 1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("LH", "Trackbars")
        u_h = cv2.getTrackbarPos("UH", "Trackbars")
        l_s = cv2.getTrackbarPos("LS", "Trackbars")
        u_s = cv2.getTrackbarPos("US", "Trackbars")
        l_v = cv2.getTrackbarPos("LV", "Trackbars")
        u_v = cv2.getTrackbarPos("UV", "Trackbars")

        lower_black = np.array([l_h, l_s, l_v])
        upper_black = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_black, upper_black)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # --- Overlay: colour the masked region on the original frame ---
        # Creates a green tinted fill over the detected line area
        overlay = frame.copy()
        overlay[mask == 255] = (0, 200, 0)                      # green fill on line pixels
        frame = cv2.addWeighted(overlay, 0.4, frame, 0.6, 0)    # blend: 40% green, 60% original

        # --- Border: draw contour outline around the line ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest) > 500:

                # Outer border (thick, dark)
                cv2.drawContours(frame, [largest], 0, (0, 80, 0), 5)
                # Inner border (thin, bright)
                cv2.drawContours(frame, [largest], 0, (0, 255, 0), 2)

                # --- Centerline: scan each row of the mask for line center ---
                # This traces the skeleton of the line, not just one centroid dot
                height, width = mask.shape
                prev_point = None

                for y in range(0, height, 8):   # step every 8px, smaller = smoother but slower
                    row = mask[y, :]
                    x_coords = np.where(row == 255)[0]

                    if len(x_coords) == 0:
                        prev_point = None
                        continue

                    x_center = int(np.mean(x_coords))   # average of white pixels in this row
                    curr_point = (x_center, y)

                    # Draw dot at each row center
                    cv2.circle(frame, curr_point, 3, (0, 0, 255), -1)

                    # Connect to previous point to form a continuous centerline
                    if prev_point is not None:
                        cv2.line(frame, prev_point, curr_point, (0, 0, 255), 2)

                    prev_point = curr_point

                # --- Overall centroid dot ---
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)          # filled dot
                    cv2.circle(frame, (cx, cy), 14, (255, 255, 255), 2)        # white ring around it
                    cv2.putText(frame, f"cx={cx}, cy={cy}", (cx + 16, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    detect_black_line()