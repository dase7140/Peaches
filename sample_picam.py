from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# BLUE HSV RANGE
LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])


def is_color_visible(frame):
    """
    Detect ANY blue at all, and also report center + top position.
    Returns:
        visible (bool)
        center_x (int or None)
        at_top (bool)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # Minimal noise cleanup
    mask = cv2.dilate(mask, None, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False, None, False

    # Use the largest blue blob
    cnt = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(cnt)

    # Ignore tiny noise blobs
    if w < 10 or h < 10:
        return False, None, False

    center_x = x + w // 2
    center_y = y + h // 2
    frame_h = frame.shape[0]

    # Check if blue is at the top 25% of the image
    at_top = center_y < frame_h * 0.25

    return True, center_x, at_top


# Example usage
while True:
    frame = picam2.capture_array()
    visible, center, at_top = is_color_visible(frame)

    if visible:
        print(f"BLUE FOUND at x={center}", end="")

        if at_top:
            print(" → BLUE at TOP")
        else:
            print(" → BLUE NOT at top")
    else:
        print("NO BLUE detected")

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
