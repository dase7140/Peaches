from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])




def is_color_visible(frame):
    """
    Detect yellow tape and determine if it is at the top of the frame.
    Returns:
        visible (bool)
        center_x (int or None)
        at_top (bool): True if yellow line is near the top of the screen
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

    # Clean noise
    mask = cv2.GaussianBlur(mask, (5,5), 0)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    frame_h = frame.shape[0]

    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)

        if w < 20 or h < 20:
            continue

        center_x = x + w // 2
        center_y = y + h // 2  # vertical position of tape

        # Tape is considered at the top if its center is in the top 25%***** of the frame
        at_top = center_y < frame_h * 0.25

        return True, center_x, at_top

    return False, None, False


# Example usage
while True:
    frame = picam2.capture_array()
    visible, center, at_top = is_color_visible(frame)

    if visible:
        print(f"Yellow tape at x={center}", end="")

        if at_top:
            print(" → Tape at TOP (correct direction)")
        else:
            print(" → Tape NOT at top (wrong direction!)")
    else:
        print("No yellow tape detected")

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()



