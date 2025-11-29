from picamera2 import Picamera2
import cv2

# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

# Capture the image into a variable (numpy array)
# This variable 'image_data' now holds the picture
image_data = picam2.capture_array()

print(f"Image captured!")

# Display the image
cv2.imshow("Captured Image", image_data)
cv2.waitKey(0)
cv2.destroyAllWindows()

picam2.stop()
