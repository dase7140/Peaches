from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import time
from datetime import datetime

picam2 = Picamera2()

# Configure the camera for still capture
config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()
time.sleep(1)  # give the camera time to adjust

picam2.capture_file(f"/home/peach/RaceCoursePhotos/photo_{datetime.now().isoformat()}.jpg")

print("Photo saved as photo.jpg")

