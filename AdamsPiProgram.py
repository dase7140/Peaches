import cv2
import numpy as np
import serial
import time
import threading
import sys 

# Serial Communication Setup
port = "/dev/ttyACM0"      # Arduino port on Raspberry Pi
ser = serial.Serial(port, 115200, timeout=1) # establish serial connection
time.sleep(2)               # wait for the serial connection to initialize
ser.reset_input_buffer()    # clear input buffer to start fresh
ser.reset_output_buffer()   # clear output buffer to start fresh

# Functions for Serial Communication
# Send command from Pi to Arduino
def pi_2_ard(command):
    try:
        ser.write((command + '\n').encode('utf-8')) # send command to Arduino
        ser.flush()                                 # ensure command is sent
        time.sleep(0.05)                             # brief pause to allow Arduino to process
    
    except Exception as e:                          # Catch any serial communication errors
        print(f"Error sending command to Arduino: {e}")
        return None

# Read line from Arduino to Pi
def serial_reader():
    # Runs forever, printing each complete line from Arduino
    while True:
        try:
            line = ser.readline()
            if line:
                text = line.decode('utf-8', errors='replace').strip()
                if text:
                    print(f"[Ard] {text}")
            else:
                # Tiny sleep prevents busy-wait when no data
                time.sleep(0.01)
        except Exception as e:
            print(f"[Ard] Read error: {e}")
            time.sleep(0.1)


def get_user_input():
    try:
        command = input("Waiting for command: ")
        return command
    except EOFError:
        return "EXIT"


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

    reader = threading.Thread(target = serial_reader, daemon=True)
    reader.start()

    while True:
        command = get_user_input()
        if not command:
            continue
        if command.upper() == "EXIT":
            print("Exiting program.")
            break
        pi_2_ard(command)
        time.sleep(0.05)  # Small delay to avoid overwhelming the serial buffer

if __name__ == "__main__":
    main()