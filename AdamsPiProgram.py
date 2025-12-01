from __future__ import print_function
import cv2
import numpy as np
import serial
import time
import threading
import sys 
from picamera2 import Picamera2
import pixy
from ctypes import *
from pixy import *
import time


# Pixy Setup
#1 = red, 2 = green 3 = blue 4= yellow 5 = purple (GRAVEL)  6 = pink (RAMP)
centerX = 157
deadband = 60
targetSignature = 1
gravelSignature = 5
rampSignature = 6
lastTargetDirection = 0
lostTargetTimer = 0
searchTimeout = 3000 

pixy.init()
pixy.change_prog("color_connected_components")

#class to pull from 
class Blocks(Structure):
    _fields_ = [
        ("m_signature", c_uint),
        ("m_x", c_uint),
        ("m_y", c_uint),
        ("m_width", c_uint),
        ("m_height", c_uint),
        ("m_angle", c_uint),
        ("m_index", c_uint),
        ("m_age", c_uint),
    ]

blocks = BlockArray(100)
frame = 0

def seeColor(sig, count):
    for i in range(count):
        if blocks[i].m_signature == sig:
            return True
    return False

def getTargetX(sig, count):
    for i in range(count):
        if blocks[i].m_signature == sig:
            return blocks[i].m_x
    return -1

def Pixicam():
    count = pixy.ccc_get_blocks(100, blocks)
    if count > 0:
        return True
    else:
        return False

brushMotorOn = False
brushMotorOnTime = 0

def Pixidrive():
    global brushMotorOn
    global brushMotorOnTime
    global lastTargetDirection

    count = pixy.ccc_get_blocks(100, blocks)

    if count > 0:
        targetSeen = seeColor(targetSignature, count)
        targetX = getTargetX(targetSignature, count)
    
        if targetSeen and targetX != -1:
            error = targetX - centerX

            if abs(error) <= deadband:
                print("Move Forward")
                pi_2_ard("ABM")
                pi_2_ard("MFD")
                brushMotorOn = True
                brushMotorOnTime = time.time() * 1000  # current time in milliseconds
                lastTargetDirection = 0 #debugging
            elif error < 0:
                print("Turn Left")
                pi_2_ard("ML0")
                lastTargetDirection = -1
            else:
                print("Turn Right")
                pi_2_ard("MR0")
                lastTargetDirection = 1
    else:
        print(f"Count =  {count}")
        pi_2_ard("MF0")


# Serial Communication Setup
port = "/dev/ttyACM0"      # Arduino port on Raspberry Pi
ser = serial.Serial(port, 115200, timeout=1) # establish serial connection
time.sleep(2)               # wait for the serial connection to initialize
ser.reset_input_buffer()    # clear input buffer to start fresh
ser.reset_output_buffer()   # clear output buffer to start fresh

# Initalize PiCamera2
picam = Picamera2()
config = picam.create_preview_configuration(main={"size": (640, 480)})
picam.configure(config)
picam.start()

yellow_detected = False
flag = True

def capture_image():
    # Capture the image into a variable (numpy array)
    image_data = picam.capture_array()
    # cv2.imshow("Captured Image", image_data)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return image_data

# Functions for Serial Communication
# Send command from Pi to Arduino
def pi_2_ard(command):
    try:
        ser.write((command + '\n').encode('utf-8')) # send command to Arduino
        ser.flush()                                 # ensure command is sent
        time.sleep(0.5)                             # brief pause to allow Arduino to process
    
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
    lower_yellow = np.array([80, 110, 150])
    upper_yellow = np.array([120, 255, 255])
    mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    return mask

def detect_yellow_mass(img, mask, area_threshold=1000):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (255, 0, 0), 3)
    # Check for large yellow masses
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= area_threshold:
            return True
    return False

def process_image(image):
    # Placeholder for any additional image processing if needed
    image = cv2.resize(image, (300, 300))
    # Crop to upper half only (keep top 150 rows, discard bottom 150)
    image = image[0:150, :]
    image = convert_bgr_to_hsv(image)
    mask = yellow_mask(image)
    yellow_detected = detect_yellow_mass(image, mask)
    return yellow_detected



def wait_for_start():
    while True:
        try:
            cmd = input('Type "Start" to begin, or "EXIT" to quit: ').strip()
        except EOFError:
            cmd = "EXIT"
        if cmd.upper() == "EXIT":
            print("Exiting program.")
            return False
        if cmd.strip().lower() == "start":
            print("Starting...")
            return True
        print('Not started. Please type "Start" or "EXIT".')

def UserControl():
    while True:
        command = get_user_input()
        if not command:
            continue
        if command.upper() == "EXIT":
            print("Exiting program.")
            break
        pi_2_ard(command)
        time.sleep(0.05)  # Small delay to avoid overwhelming the serial buffer

# Main driving function
def drive():
    global brushMotorOn
    global brushMotorOnTime
    yellowFlagLast = None  # None, True, or False
    pixi = None

    while True:
        pixi = Pixicam()
        img = capture_image()
        yellowFlagCurrent = process_image(img)
        print(f"Yellow Detected: {yellowFlagCurrent}")

        if pixi is True:
            Pixidrive()
        elif pixi is False:
            if brushMotorOn is True:
                currentTime = time.time() * 1000  # current time in milliseconds
                if currentTime - brushMotorOnTime >= 5000:  # 5 seconds have passed
                    pi_2_ard("DBM")  # Stop Brush Motor
                    brushMotorOn = False
            if yellowFlagCurrent and yellowFlagLast is not True:
                pi_2_ard("DBI")
                print("Sent DBI (yellow acquired)")
            elif not yellowFlagCurrent and yellowFlagLast is not False:
                pi_2_ard("YLL")
                print("Sent YLL (yellow lost)")

        yellowFlagLast = yellowFlagCurrent
        time.sleep(0.1)

def main():

    if not wait_for_start():
        return
    
    reader = threading.Thread(target = serial_reader, daemon=True)
    reader.start()

    #UserControl()
    #drive()
    # capture_image()
    while True:
        Pixidrive()

if __name__ == "__main__":
    main()