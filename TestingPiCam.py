from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize PiCamera2

# If yellow is seen in the middle, it  will say not detected

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])


def is_yellow_visible(frame):
    """
    Detect yellow tape on the left side of the robot.
    Returns:
        visible (bool): True if yellow tape detected
        center_x (int): x-coordinate of yellow line center (None if not detected)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # Reduce noise
    mask = cv2.GaussianBlur(mask, (5,5), 0)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        # Ignore tiny blobs
        if w < 20 or h < 20:
            continue

        center_x = x + w // 2

        # Check if center is on the left side of the frame
        if center_x < frame.shape[1] // 2:
            return True, center_x

    # If no valid yellow line found
    return False, None

# Example usage
while True:
    frame = picam2.capture_array()
    yellow_seen, center = is_yellow_visible(frame)

    if yellow_seen:
        print(f"Yellow tape detected at x={center}")
    else:
        print("No yellow tape detected")

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
