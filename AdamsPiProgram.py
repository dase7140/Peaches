import cv2
import numpy as np
import serial
import time

port = "/dev/tty/ACM0"
ser = serial.Serial(port, 115200, timeout=1)
time.sleep(2)

def pi_2_ard(command):
    try:
        ser.write((command + '\n').encode('utf-8'))
        time.sleep(0.1)
    except Exception as e:
        print(f"Error sending command to Arduino: {e}")
        return None
    
def ard_2_pi():
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            return line
        else:
            return None
    except Exception as e:
        print(f"Error reading from Arduino: {e}")
        return None
    
def get_user_input():
    command = input("Waiting for command: ")
    return command


# Load image from a given file path
def load_image_from_path(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found at path: {image_path}")
    return image   

# Convert BGR image to HSV color space
def convert_bgr_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Create a mask for yellow color in the HSV image
def yellow_mask(hsv_image):
    lower_yellow = np.array([0, 100, 135])
    upper_yellow = np.array([95, 255, 255])
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    return mask

def detect_yellow_mass(img, mask, area_threshold=1000):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
    # cv2.imshow("Contours", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # Check for large yellow masses
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= area_threshold:
            return True
    return False

def process_image(image):
    # Placeholder for any additional image processing if needed
    image = cv2.resize(image, (300, 300))
    image = convert_bgr_to_hsv(image)
    mask = yellow_mask(image)
    yellow_detected = detect_yellow_mass(image, mask)
    return yellow_detected

def drive():
    while True:
        img = load_image_from_path('RaceCoursePhotos\photo_2025-11-26T14-24-29.867799.jpg')
        yellow_detected = process_image(img)

        if yellow_detected:
            # Drive IR
            print("Yellow line detected: Drive IR")
        else:
            # Drive Blind
            print("No yellow line detected: Drive Blind")

def main():
    while True:
        command = get_user_input()
        pi_2_ard(command)
        line = ard_2_pi()
        print(f"Arduino says: {line}")    
    

if __name__ == "__main__":
    main()

    
# cv2.imshow("Image", mask)
# cv2.waitKey(0)
# cv2.destroyAllWindows()