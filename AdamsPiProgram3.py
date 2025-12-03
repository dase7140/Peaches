from __future__ import print_function
import cv2
import numpy as np
import serial
import time
import threading
import sys
from queue import Queue, Empty
from picamera2 import Picamera2
import pixy
from ctypes import *
from pixy import *

########################
# Pixycam Setup
########################
#1 = red, 2 = green 3 = blue 4= yellow 5 = purple (GRAVEL)  6 = pink (RAMP)



COLOR_MAP = {
    "red": 1,
    "green": 2,
    "blue": 3,
    "yellow": 4,
    "purple": 5,    # GRAVEL
    "orange": 6,    # RAMP
    "l_blue": 7     # OTHER FLAG COLOR
}

#Pixy Init:
#1 = red, 2 = green 3 = blue 4= yellow 5 = purple (GRAVEL)  6 = pink (RAMP)
centerX = 157
deadband = 30
target_Signature = COLOR_MAP["red"]  # change for dice designation, red die for now
dice_sig = target_Signature
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
onGravel = False
onBridge = False
bridgeTimer = 0

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

def getColor(index):
    return blocks[index].m_signature

def Pixicam():  # FOR DICE
    """
    Queries Pixy camera for color blocks matching target signature.
    Ignores targets in the TOP HALF of the frame.
    
    Returns:
        bool - True if target detected in bottom half, False otherwise
    """
    try:
        count = pixy.ccc_get_blocks(100, blocks)
        now = int(round(time.time() * 1000))

        if count > 0:
            for i in range(count):

                # Only consider blocks with the target signature
                if blocks[i].m_signature == target_Signature:

                    x = blocks[i].m_x
                    y = blocks[i].m_y

                    # IGNORE dice in top half of screen
                    if y < 150:   # top half
                        continue  # skip it

                    # valid block found in bottom half
                    return True

            # no valid dice detected in bottom half
            return False

        else:
            return False

    except Exception:
        return False

FLAG_WIDTH = 60 # TODO calibrate this value

def getArea(index):
    return (blocks[index].m_width * blocks[index].m_height)

def lookForFlags():
    """
    Detects flag colors (orange, light blue, purple) but ignores them
    if they appear in the lower 20% of the screen.
    """

    try:
        count = pixy.ccc_get_blocks(100, blocks)
        if count == 0:
            return True, False, False, False

        frame_height = 208
        ignore_threshold = int(frame_height * 0.80)

        # Colors involved in flag combinations
        FLAG_SIGS = {
            COLOR_MAP["orange"],
            COLOR_MAP["l_blue"],
            COLOR_MAP["purple"]
        }

        # Initialize seen colors
        seen = {sig: False for sig in COLOR_MAP.values()}

        for i in range(count):
            sig = blocks[i].m_signature
            y   = blocks[i].m_y

            # Ignore ONLY flag-related colors in lower 20%
            if sig in FLAG_SIGS and y > ignore_threshold:
                continue

            # Mark color as seen
            if sig in seen:
                seen[sig] = True

        # Combinations for flags
        first_flag  = seen[COLOR_MAP["orange"]] and seen[COLOR_MAP["purple"]]
        second_flag = seen[COLOR_MAP["orange"]] and seen[COLOR_MAP["l_blue"]]
        third_flag  = seen[COLOR_MAP["purple"]] and seen[COLOR_MAP["l_blue"]]

        return True, first_flag, second_flag, third_flag

    except Exception as e:
        print("[PIXY ERROR] lookForFlags exception:", e)
        return False, False, False, False

# Debounce counters
first_flag_count = 0
second_flag_count = 0
third_flag_count = 0

DEBOUNCE_FRAMES = 3   # number of consecutive frames required

def pixySetFlags():
    """
    Debounced flag detection for gravel/bridge progression.
    A flag must be detected for DEBOUNCE_FRAMES in a row
    before applying mode changes.
    """

    global onGravel, onBridge
    global first_flag_count, second_flag_count, third_flag_count

    success, first_flag, second_flag, third_flag = lookForFlags()
    if not success:
        return

    # --- Update debounce counters ---
    first_flag_count  = first_flag_count  + 1 if first_flag  else 0
    second_flag_count = second_flag_count + 1 if second_flag else 0
    third_flag_count  = third_flag_count  + 1 if third_flag  else 0

    # ---- First Flag: Enter Gravel Mode ----
    if first_flag_count >= DEBOUNCE_FRAMES and not onGravel:
        print("[PIXY FLAGS] Entering gravel mode (debounced)")
        onGravel = True
        onBridge = False
        bridgeTimer = 0

    # ---- Second Flag: Enter Bridge Mode ----
    elif second_flag_count >= DEBOUNCE_FRAMES and not onBridge:
        print("[PIXY FLAGS] Entering bridge mode (debounced)")
        onBridge = True
        bridgeTimer = True
        onGravel = False

    # ---- Third Flag: Exit special modes ----
    elif third_flag_count >= DEBOUNCE_FRAMES:
        if onGravel or onBridge:
            print("[PIXY FLAGS] Leaving special areas (debounced)")
        onGravel = False
        onBridge = False
        bridgeTimer = 0


        

############################
# Serial Communication Setup
############################

port = "/dev/ttyACM0"      # Arduino port on Raspberry Pi
ser = serial.Serial(port, 115200, timeout=1) # establish serial connection
time.sleep(2)               # wait for the serial connection to initialize
ser.reset_input_buffer()    # clear input buffer to start fresh
ser.reset_output_buffer()   # clear output buffer to start fresh

# Thread control and state variables
serial_reader_running = False
estop_triggered = False
serial_thread = None
user_stop_requested = False
stdin_thread = None

# Message queues for routing serial data
ack_queue = Queue()  # For ACK messages
ir_queue = Queue()   # For IR sensor data

def serial_reader():
    """
    Continuously reads incoming serial messages from Arduino.
    Routes messages to appropriate queues for other functions to consume.
    
    Message routing:
    - ESTOP: Sets global flag immediately
    - ACK:*: Routed to ack_queue for pi_2_ard()
    - IR:*: Routed to ir_queue for read_ir_sensors()
    - Other: Printed as general Arduino output
    """
    global serial_reader_running, estop_triggered, onGravel, onBridge
    
    print("[Serial Reader] Thread started")
    
    while serial_reader_running:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                
                if line:
                    # Handle ESTOP message - immediate action
                    if line == "ESTOP":
                        estop_triggered = True
                        print("[Arduino] EMERGENCY STOP TRIGGERED")
                    
                    # Route ACK messages to ack_queue
                    elif line.startswith("ACK:"):
                        ack_queue.put(line)
                    
                    # Route IR data to ir_queue
                    elif line.startswith("IR:"):
                        ir_queue.put(line)

                    elif line == "BMA":
                        print("[Arduino] Bridge Mode Activated")
                        onBridge = True
                        bridgeTimer = time.time

                    elif line == "BMD":
                        print("[Arduino] Bridge Mode Deactivated")
                        onBridge = False
                        bridgeTimer = 0
                    
                    # Print other messages
                    else:
                        print(f"[Arduino] {line}")
            
            # Small delay to prevent busy waiting
            time.sleep(0.01)
            
        except Exception as e:
            print(f"[Serial Reader] Error: {e}")
            time.sleep(0.1)
    
    print("[Serial Reader] Thread stopped")



def start_serial_reader():
    """
    Starts the serial reader thread.
    Call this at the beginning of your main program.
    """
    global serial_reader_running, serial_thread
    
    if serial_thread is not None and serial_thread.is_alive():
        print("[Serial Reader] Thread already running")
        return
    
    serial_reader_running = True
    serial_thread = threading.Thread(target=serial_reader, daemon=True)
    serial_thread.start()
    print("[Serial Reader] Thread started successfully")



def stop_serial_reader():
    """
    Stops the serial reader thread.
    Call this when shutting down the program.
    """
    global serial_reader_running, serial_thread
    
    if serial_thread is None or not serial_thread.is_alive():
        print("[Serial Reader] Thread not running")
        return
    
    print("[Serial Reader] Stopping thread...")
    serial_reader_running = False
    serial_thread.join(timeout=2.0)  # Wait up to 2 seconds for thread to finish
    
    if serial_thread.is_alive():
        print("[Serial Reader] Warning: Thread did not stop cleanly")
    else:
        print("[Serial Reader] Thread stopped successfully")



def User_Input():
    """
    Monitors stdin for user commands in a separate thread.
    Specifically watches for 'STOP' command to emergency stop the robot.
    """
    global user_stop_requested
    
    print("[User Input] Thread started - Type 'STOP' at any time to emergency stop")
    
    while not user_stop_requested:
        try:
            user_input = input().strip().upper()
            if user_input == "STOP":
                user_stop_requested = True
                print("\n[USER STOP] Emergency stop requested!")
                break
        except (EOFError, KeyboardInterrupt):
            break
        except Exception as e:
            print(f"[User Input] Error: {e}")
            time.sleep(0.1)
    
    print("[User Input] Thread stopped")



def start_User_Input():
    """
    Starts the stdin monitor thread.
    Allows user to type 'STOP' to emergency stop the robot.
    """
    global user_stop_requested, stdin_thread
    
    if stdin_thread is not None and stdin_thread.is_alive():
        print("[User Input] Thread already running")
        return
    
    user_stop_requested = False
    stdin_thread = threading.Thread(target=User_Input, daemon=True)
    stdin_thread.start()



def stop_User_Input():
    """
    Signals the stdin monitor thread to stop.
    """
    global user_stop_requested, stdin_thread
    
    if stdin_thread is None or not stdin_thread.is_alive():
        return
    
    user_stop_requested = True


# Send command from Pi to Arduino with ACK waiting and retry logic
def pi_2_ard(command, max_retries=3, timeout=1.0):
    """
    Sends a command to Arduino and waits for acknowledgment from ack_queue.
    Retries if no ACK is received within timeout period.
    
    Args:
        command: str - Command to send to Arduino
        max_retries: int - Maximum number of retry attempts (default: 3)
        timeout: float - Seconds to wait for ACK before retrying (default: 1.0)
        
    Returns:
        bool - True if ACK received, False if all retries failed
    """
    for attempt in range(max_retries):
        try:
            # Clear any stale ACKs from queue
            stale_count = 0
            while not ack_queue.empty():
                try:
                    stale_ack = ack_queue.get_nowait()
                    stale_count += 1
                except Empty:
                    break
            
            if stale_count > 0:
                print(f"[Comm] Cleared {stale_count} stale ACK(s) from queue")
            
            # Brief delay to ensure Arduino is ready for next command
            time.sleep(0.05)
            
            # Send command to Arduino
            ser.write((command + '\n').encode('utf-8'))
            ser.flush()
            
            # Wait for ACK response from queue
            expected_ack = f"ACK:{command}"
            
            # Keep checking queue until we get the right ACK or timeout
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                try:
                    # Wait for message from queue with shorter timeout for polling
                    ack_msg = ack_queue.get(timeout=0.1)
                    
                    # Check if this is the expected ACK
                    if ack_msg == expected_ack:
                        if attempt > 0:
                            print(f"[Comm] Command '{command}' acknowledged (attempt {attempt + 1})")
                        return True
                    else:
                        # Wrong ACK - likely from previous command, keep waiting
                        print(f"[Comm] Ignoring late ACK: {ack_msg} (waiting for {expected_ack})")
                        
                except Empty:
                    # No message yet, keep waiting
                    continue
            
            # Timeout reached - no correct ACK received
            if attempt < max_retries - 1:
                print(f"[Comm] No ACK for '{command}' (attempt {attempt + 1}/{max_retries}), retrying...")
            else:
                print(f"[Comm] FAILED: No ACK for '{command}' after {max_retries} attempts")
        
        except Exception as e:
            print(f"[Comm] Error sending command '{command}': {e}")
            if attempt < max_retries - 1:
                time.sleep(0.1)  # Brief delay before retry
    
    return False  # All retries failed




###################
# Pi Camera 
###################
# picam = Picamera2()
# config = picam.create_preview_configuration(main={"size": (640, 480)})
# picam.configure(config)
# picam.start()


# def capture_image():
#     image_data = picam.capture_array()
#     blurred = cv2.GaussianBlur(image_data, (5, 5), 0)
#     hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
#     return hsv_frame

# def load_image_from_path(image_path):
#     image = cv2.imread(image_path)
#     if image is None:
#         raise FileNotFoundError(f"Image not found at path: {image_path}")
#     return image   


# # Yellow Detection HSV Thresholds
# # These can be tuned based on lighting conditions
# # HSV ranges for yellow (adjust as needed)
# YELLOW_LOWER_H = 80      # Lower hue bound
# YELLOW_UPPER_H = 135      # Upper hue bound
# YELLOW_LOWER_S = 35      # Lower saturation (increase to filter pale yellows)
# YELLOW_UPPER_S = 255     # Upper saturation
# YELLOW_LOWER_V = 60      # Lower value/brightness (increase in bright conditions)
# YELLOW_UPPER_V = 255     # Upper value/brightness


# def yellow_mask(hsv_frame):
#     # Define lower and upper bounds for yellow in HSV
#     lower_yellow = np.array([YELLOW_LOWER_H, YELLOW_LOWER_S, YELLOW_LOWER_V])
#     upper_yellow = np.array([YELLOW_UPPER_H, YELLOW_UPPER_S, YELLOW_UPPER_V])
#     # Create binary mask: pixels within range are white (255), others are black (0)
#     mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
#     return mask



# # Morphological operation parameters (adjust for your needs)
# MORPH_KERNEL_SIZE = 5       # Size of structuring element (5x5 is a good start)
# MORPH_OPEN_ITERATIONS = 2   # Number of opening operations (removes small noise)
# MORPH_CLOSE_ITERATIONS = 3  # Number of closing operations (fills small gaps)

# def clean_mask(mask):
#     # Create structuring element (kernel) for morphological operations
#     kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    
#     # Opening: Erosion followed by Dilation
#     # Removes small white noise/spots while preserving larger shapes
#     opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=MORPH_OPEN_ITERATIONS)
    
#     # Closing: Dilation followed by Erosion
#     # Fills small black holes inside white regions and connects nearby regions
#     cleaned_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=MORPH_CLOSE_ITERATIONS)
    
#     return cleaned_mask


# drive_with_ellipse = True


# def find_yellow_ellipse(mask):
#     """
#     Fits an elipse to the largest yellow contour in the mask

#     Args
#         maskL numpy array - binary mask with yellow regions
    
#     Returns:
#         Tuple: (found, cy, angle)
#         found: bool - true if yellow region was detected
#         cy: int - y coordinate of centroid
#         angle: int - angle between the horizontal axis and the major axis of the ellipse
#     """
#     # Find all contours in the mask
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     # Check if any contours were found
#     if len(contours) == 0:
#         return False, 0, 0
    
#     # Find the largest contour by area
#     largest_contour = max(contours, key=cv2.contourArea)
    
#     # Filter out very small contours (likely noise)
#     MIN_CONTOUR_AREA = 150  # Adjust this threshold as needed
#     if cv2.contourArea(largest_contour) < MIN_CONTOUR_AREA:
#         return False, 0, 0
    
#     ellipse = cv2.fitEllipse(largest_contour)
#     ((_, cy), (_, _), angle) = ellipse
#     angle = angle-90

#     return True, cy, angle

# def find_yellow_centroid(mask):
#     """
#     Finds the largest yellow contour in the mask and computes its centroid.
    
#     Args:
#         mask: numpy array - Binary mask with yellow regions
        
#     Returns:
#         tuple: (found, cx, cy)
#             found: bool - True if yellow region was detected
#             cx: int - X coordinate of centroid (0 if not found)
#             cy: int - Y coordinate of centroid (0 if not found)
#     """
#     # Find all contours in the mask
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     # Check if any contours were found
#     if len(contours) == 0:
#         return False, 0, 0
    
#     # Find the largest contour by area
#     largest_contour = max(contours, key=cv2.contourArea)
    
#     # Filter out very small contours (likely noise)
#     MIN_CONTOUR_AREA = 150  # Adjust this threshold as needed
#     if cv2.contourArea(largest_contour) < MIN_CONTOUR_AREA:
#         return False, 0, 0
    
#     # Calculate moments of the largest contour
#     M = cv2.moments(largest_contour)
    
#     # Compute centroid coordinates
#     # Check for division by zero
#     if M["m00"] == 0:
#         return False, 0, 0
    
#     cx = int(M["m10"] / M["m00"])
#     cy = int(M["m01"] / M["m00"])
    
#     return True, cx, cy



# def compute_error(centroid_y, image_height):
#     """
#     Computes the vertical error between the yellow centroid and the target position.
#     Camera is mounted UPSIDE DOWN on the LEFT side of the robot.
    
#     Target position is the center of the BOTTOM HALF of the image (at 3/4 from top).
    
#     Args:
#         centroid_y: int - Y coordinate of the yellow centroid
#         image_height: int - Height of the camera frame in pixels
        
#     Returns:
#         error: float - Vertical offset from target position
#                       Positive = yellow is too far (robot needs to turn left)
#                       Negative = yellow is too close (robot needs to turn right)
#                       Zero = yellow is at target position (go straight)
#     """
#     # Calculate the target y-coordinate: center of the image
#     target_y = image_height / 2
    
#     # Compute error: positive means yellow is below target (higher Y) = too far in real world
#     # Since camera is upside down: high Y value in image = far from robot in real world
#     error = centroid_y - target_y
    
#     return error


# # Yellow Line Following Control Parameters
# ERROR_THRESHOLD = 70        # Deadband in pixels - go straight if error < this
# ANGLE_THRESHOLD = 10        # Angular deadband - dont turn if angle is less than threshold
# EXTREME_ERROR_THRESHOLD = 150  # Pixels - switch from veer to full turn if error exceeds this
# EXTREME_ANGLE_THRESHOLD = 35   # Degrees - switch from veer to full turn if angle exceeds this
# BASE_SPEED = 2              # Default speed level (1-5)
# TURN_SPEED = 2              # Speed when turning to follow line
# VEER_SPEED = 3              # speed when veering

# def yellow_line_steering(error):
#     """
#     Converts centroid error to motor steering command.
    
#     Camera mounted upside down on LEFT side:
#     - Positive error = yellow too far (top of upside-down image) → turn LEFT
#     - Negative error = yellow too close (bottom of upside-down image) → turn RIGHT
#     - Small error = go straight FORWARD
    
#     Args:
#         error: float - Error from compute_error()
        
#     Returns:
#         str - Motor command ("MF2", "ML2", "MR2", etc.)
#     """
#     # Within deadband - go straight
#     if abs(error) < ERROR_THRESHOLD:
#         return f"MF{BASE_SPEED}"
    
#     # Positive error - yellow is too far, turn LEFT to get closer
#     elif error > 0:
#         return f"ML{TURN_SPEED}"
    
#     # Negative error - yellow is too close, turn RIGHT to move away
#     else:
#         return f"MR{TURN_SPEED}"

# def yellow_line_steering_ellipse(y_error, angle):
#     """
#     Enhanced ellipse-based steering with progressive response:
#     - Extreme errors: Stop and do full turn (ML/MR)
#     - Moderate errors: Veer to correct (VL/VR)
#     - Small errors: Go straight (MF)
    
#     Args:
#         y_error: float - Vertical centroid error in pixels
#         angle: float - Ellipse angle in degrees
        
#     Returns:
#         str - Motor command (e.g., "MF2", "VL2", "ML1")
#     """
#     # Check for EXTREME conditions requiring full stop and turn
#     # Priority 1: Extreme centroid error
#     if abs(y_error) > EXTREME_ERROR_THRESHOLD:
#         if y_error > 0:
#             # Yellow too far - stop and turn LEFT
#             return f"ML{TURN_SPEED}"
#         else:
#             # Yellow too close - stop and turn RIGHT
#             return f"MR{TURN_SPEED}"
    
#     # Priority 2: Extreme angle error (even if centroid is centered)
#     if abs(angle) > EXTREME_ANGLE_THRESHOLD:
#         if angle < 0:
#             # Line angled left - stop and turn LEFT to straighten
#             return f"ML{TURN_SPEED}"
#         else:
#             # Line angled right - stop and turn RIGHT to straighten
#             return f"MR{TURN_SPEED}"
    
#     # MODERATE conditions - veer while moving forward
#     # Centroid is reasonably centered, adjust angle with veering
#     if abs(y_error) < ERROR_THRESHOLD:
#         if abs(angle) < ANGLE_THRESHOLD:
#             # Both centered - go straight
#             return f"MF{BASE_SPEED}"
#         elif angle < 0:
#             # Line angled left - veer left to align
#             return f"VL{VEER_SPEED}"
#         else:
#             # Line angled right - veer right to align
#             return f"VR{VEER_SPEED}"
    
#     # Centroid has moderate error - veer to correct
#     else:
#         if y_error > 0:
#             # Yellow too far - veer LEFT to get closer
#             return f"VL{VEER_SPEED}"
#         else:
#             # Yellow too close - veer RIGHT to move away
#             return f"VR{VEER_SPEED}"






##############
# IR Sensors
##############

# IR Sensor Configuration
IR_SENSOR_COUNT = 5
IR_SENSOR_NAMES = ["Left", "Front_Right", "Front_Left", "Right", "Back"]

# IR sensor index mapping (matches Arduino)
# Arduino channels: Left=0, Front_Right=1, Front_Left=2, Right=3, Back=4
IR_LEFT = 0
IR_FRONT_RIGHT = 1
IR_FRONT_LEFT = 2
IR_RIGHT = 3
IR_BACK = 4

class IRSensorData:
    """Class to hold IR sensor readings in a structured format"""
    def __init__(self):
        self.left = 0
        self.front_left = 0
        self.front_right = 0
        self.right = 0
        self.back = 0
        self.raw_distances = [0] * IR_SENSOR_COUNT
        self.timestamp = 0
        self.valid = False
    
    def __str__(self):
        return (f"IR Sensors [valid={self.valid}]: "
                f"L={self.left}mm, FL={self.front_left}mm, "
                f"FR={self.front_right}mm, R={self.right}mm, B={self.back}mm")

def read_ir_sensors():
    """
    Requests IR sensor data from Arduino and parses the response from ir_queue.
    
    Returns:
        IRSensorData: Object containing all 5 IR distance readings
                      Returns None if communication fails or timeout occurs
    """
    try:
        # Clear any stale IR data from queue
        while not ir_queue.empty():
            try:
                ir_queue.get_nowait()
            except Empty:
                break
        
        # Send command to Arduino to read IR sensors with ACK waiting
        success = pi_2_ard("RIS")
        
        if not success:
            print("[IR] Failed to send RIS command")
            return None
        
        # Create sensor data object
        sensor_data = IRSensorData()
        sensor_data.timestamp = time.time()
        
        # Wait for IR response from queue (blocking with timeout)
        try:
            line = ir_queue.get(timeout=1.0)  # 1 second timeout
            
            # Parse compact format: "IR:450,320,500,380,600"
            if line.startswith("IR:"):
                try:
                    data_str = line[3:]  # Remove "IR:" prefix
                    values = data_str.split(',')
                    
                    # Validate we got all 5 sensors
                    if len(values) != IR_SENSOR_COUNT:
                        print(f"[IR] Invalid data count: got {len(values)}, expected {IR_SENSOR_COUNT}")
                        return None
                    
                    # Parse each value
                    for i, val_str in enumerate(values):
                        sensor_data.raw_distances[i] = int(val_str.strip())
                    
                    # Map to named fields
                    sensor_data.left = sensor_data.raw_distances[IR_LEFT]
                    sensor_data.front_left = sensor_data.raw_distances[IR_FRONT_LEFT]
                    sensor_data.front_right = sensor_data.raw_distances[IR_FRONT_RIGHT]
                    sensor_data.right = sensor_data.raw_distances[IR_RIGHT]
                    sensor_data.back = sensor_data.raw_distances[IR_BACK]
                    sensor_data.valid = True
                    
                    print(f"[IR] {sensor_data}")
                    return sensor_data
                    
                except (ValueError, IndexError) as e:
                    print(f"[IR] Parse error: {line} - {e}")
                    return None
            else:
                print(f"[IR] Unexpected message: {line}")
                return None
                
        except Empty:
            # Timeout reached
            print(f"[IR] Timeout: No IR data received")
            return None
            
    except Exception as e:
        print(f"[IR] Error reading sensors: {e}")
        return None
###################



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





def reposition2():
    """
    Intelligent repositioning function when obstacle detected.
    
    Strategy:
    1. Try to find yellow line ellipse and align to it
       - If angle is off, turn to straighten the ellipse
       - Target angle is 0° (horizontal line in upside-down view)
    2. If no ellipse found, fall back to IR sensor-based repositioning
       - Read IR sensors to assess surroundings
       - If back is clear (>100mm), reverse to create space
       - Calculate left area vs right area and turn towards clearance
    
    Returns:
        bool - True if repositioning succeeded, False if failed
    """
    print("[Reposition] Starting repositioning maneuver")
        
    # Step 1: Read current sensor values
    ir_data = read_ir_sensors()
    if ir_data is None or not ir_data.valid:
        print("[Reposition] Failed to read IR sensors")
        return False
    
    print(f"[Reposition] Sensor readings: {ir_data}")
    
    # Helper function to cap sensor readings (VL53L0X returns 8190+ when out of range)
    def cap_distance(distance, max_distance=2000):
        """Cap distance readings to handle out-of-range sensors"""
        if distance >= 8000:  # VL53L0X out-of-range indicator
            return max_distance
        return min(distance, max_distance)
    
    # Step 2: Check if back is clear and reverse if possible
    BACK_CLEAR_THRESHOLD = 100  # mm
    back_distance = cap_distance(ir_data.back)
    
    if back_distance > BACK_CLEAR_THRESHOLD:
        print(f"[Reposition] Back clear ({back_distance}mm) - reversing")
        pi_2_ard("MB3")  # Reverse at speed 3
        time.sleep(0.2)  # Reverse for 200ms
        pi_2_ard("MF0")  # Stop motors
        time.sleep(0.2)
    else:
        print(f"[Reposition] Back blocked ({back_distance}mm) - skipping reverse")
    
    # Step 3: Cap sensor readings and calculate left and right clearance areas
    front_left_capped = cap_distance(ir_data.front_left)
    left_capped = cap_distance(ir_data.left)
    front_right_capped = cap_distance(ir_data.front_right)
    right_capped = cap_distance(ir_data.right)
    
    left_area = front_left_capped + left_capped
    right_area = front_right_capped + right_capped
    
    print(f"[Reposition] Left area: {left_area}mm (FL:{front_left_capped} + L:{left_capped})")
    print(f"[Reposition] Right area: {right_area}mm (FR:{front_right_capped} + R:{right_capped})")
    
    # Step 4: Turn towards the direction with more clearance
    
    if left_area > right_area:
        print(f"[Reposition] Turning LEFT (left area larger by {left_area - right_area}mm)")
        pi_2_ard("ML5")  # Turn left at speed 5
        time.sleep(0.2)  # Turn for 200ms
    else:
        print(f"[Reposition] Turning RIGHT (right area larger by {right_area - left_area}mm)")
        pi_2_ard("MR5")  # Turn right at speed 5
        time.sleep(0.2)  # Turn for 200ms
    
    pi_2_ard("MF0")  # Stop motors
    
    print("[Reposition] Repositioning complete")
    return True

def drive2():
    """
    Main driving control loop using Arduino autonomous navigation.
    Arduino handles line following and obstacle avoidance independently.
    Pi only takes manual control when Pixicam detects a target for steering.
    """
    global user_stop_requested, estop_triggered, onGravel, onBridge
    
    print("[Drive] Starting main control loop")
    print("[Drive] Type 'STOP' at any time to emergency stop")
    
    # Brush motor state management
    brush_motor_active = False
    brush_motor_off_time = None
    BRUSH_MOTOR_DELAY = 10.0  # seconds to keep brush on after target detected
    
    # Bridge mode brush motor deactivation
    bridge_brush_disabled_time = None
    BRIDGE_BRUSH_DISABLE_DURATION = 5.0  # seconds to keep brush off when on bridge
    
    # Control mode state
    autonomous_mode = True  # True = Arduino controls, False = Pi controls
    sd5_sent = False  # Track if SD5 command has been sent
    
    # Manual control tracking
    last_manual_cmd = None
    manual_control_start_time = None
    
    # Pixy camera error tracking
    pixy_error_count = 0
    MAX_PIXY_ERRORS = 10  # Warn after this many consecutive errors
    
    # Pixicam target steering parameters
    PIXY_CENTER_X = 158  # Center X coordinate of Pixy camera (316/2)
    PIXY_DEADBAND = 30   # Pixels - don't steer if target within this range of center
    
    try:
        # Start Arduino autonomous line following
        print("[Drive] Starting Arduino autonomous mode (SD5)")
        SDmode = "MF5"
        if onBridge:
            currTime = time.time()
            timer = currTime - bridgeTimer
            if timer < 5:
                SDmode = "MF5"
            elif 5 <= timer and timer <= 15:
                SDmode = "MF5"
            else:
                SDmode = "MF5"
        elif onGravel:
            SDmode = "MF5"

        pi_2_ard(SDmode)
        sd5_sent = True
        autonomous_mode = True

        while True:
            # Check for user stop command
            if user_stop_requested:
                print("\n[Drive] User STOP command received - shutting down motors")
                pi_2_ard("MF0", max_retries=5, timeout=0.2)  # Stop motors
                pi_2_ard("DBM", max_retries=3, timeout=0.2)  # Turn off brush motor
                break
            
            # Check for Arduino emergency stop
            if estop_triggered:
                print("\n[Drive] Arduino ESTOP - handling repositioning autonomously")
                estop_triggered = False  # Reset flag
                # Arduino automatically resumes drive_IR after repositionArduino() completes
                # No action needed from Pi
            
            # ===== PIXICAM TARGET DETECTION AND MANUAL STEERING =====
            ##pixySetFlags()  # Check and set flags first
            
            # Handle bridge/gravel mode - deactivate brush motor
            if onBridge or onGravel:
                if bridge_brush_disabled_time is None:
                    print("[Drive] Entering bridge/gravel mode - deactivating brush motor")
                    pi_2_ard("DBM")
                    brush_motor_active = False
                    brush_motor_off_time = None
                    bridge_brush_disabled_time = time.time()

            # Reset bridge timer when not on bridge/gravel anymore
            if bridge_brush_disabled_time is not None and not onBridge and not onGravel:
                elapsed_time = time.time() - bridge_brush_disabled_time
                if elapsed_time >= BRIDGE_BRUSH_DISABLE_DURATION:
                    print(f"[Drive] Bridge mode ended - brush motor can be reactivated (was disabled for {elapsed_time:.1f}s)")
                    bridge_brush_disabled_time = None
                else:
                    # Still within the 5-second disable period after leaving bridge
                    print(f"[Drive] Bridge disable period active ({BRIDGE_BRUSH_DISABLE_DURATION - elapsed_time:.1f}s remaining)")
                    # Let dice detection run, but block motor activation below
            
            # Normal operation - check for dice and manage brush motor
            # (This section runs when not on bridge/gravel and bridge timer has expired)
            
            # ===== DICE DETECTION WITH PIXICAM =====
            # try:
            #     target_detected = Pixicam()
            #     pixy_error_count = 0  # Reset error count on successful read
            # except Exception as e:
            #     # Pixy error - assume no target detected
            #     target_detected = False
            #     pixy_error_count += 1
                
            #     # Warn if errors persist
            #     if pixy_error_count == MAX_PIXY_ERRORS:
            #         print(f"[Pixy] WARNING: {MAX_PIXY_ERRORS} consecutive errors - camera may be disconnected")
            #         print("[Pixy] Continuing without target detection...")
            
            # # Target detected - activate brush motor with 10-second timer
            # if target_detected:
            #     if not brush_motor_active and bridge_brush_disabled_time is None:
            #         # Only activate if NOT in bridge disable period
            #         print("[Drive] Dice detected - activating brush motor for 10 seconds")
            #         pi_2_ard("ABM")
            #         brush_motor_active = True
            #         brush_motor_off_time = time.time() + BRUSH_MOTOR_DELAY
            #     elif brush_motor_active and brush_motor_off_time is not None:
            #         # Dice still detected - reset the timer to keep motor running
            #         brush_motor_off_time = time.time() + BRUSH_MOTOR_DELAY
            #     elif bridge_brush_disabled_time is not None:
            #         # Dice detected but still in bridge disable period
            #         elapsed = BRIDGE_BRUSH_DISABLE_DURATION - (time.time() - bridge_brush_disabled_time)
            #         if elapsed > 0:
            #             print(f"[Drive] Dice detected but bridge disable active ({elapsed:.1f}s remaining)")
            
            # # Check if it's time to turn off brush motor (after 10 seconds)
            # if brush_motor_active and brush_motor_off_time is not None:
            #     if time.time() >= brush_motor_off_time:
            #         print("[Drive] 10-second timer expired - turning off brush motor")
            #         pi_2_ard("DBM")
            #         brush_motor_active = False
            #         brush_motor_off_time = None
            
            time.sleep(0.05)  # Small delay to prevent busy loop
            
    except KeyboardInterrupt:
        print("\n[Drive] Keyboard interrupt - shutting down motors")
        pi_2_ard("MF0", max_retries=5, timeout=0.2)
        pi_2_ard("DBM", max_retries=3, timeout=0.2)
    
    print("[Drive] Control loop ended")

def main():
    # Start serial reader thread
    start_serial_reader()
    
    if not wait_for_start():
        return
    
    # Start stdin monitor thread for emergency stop capability
    start_User_Input()
    
    try:
        # Start the main yellow-following loop
        drive2()
    finally:
        # Emergency shutdown - ensure motors are stopped
        print("\n[Main] Shutting down...")
        
        # CRITICAL: Send stop commands BEFORE stopping serial reader thread
        try:
            print("[Main] Stopping motors...")
            pi_2_ard("MF0", max_retries=5, timeout=1.0)
            time.sleep(0.2)  # Allow command to be processed
            print("[Main] Deactivating brush motor...")
            pi_2_ard("DBM", max_retries=5, timeout=1.0)
            time.sleep(0.2)  # Allow command to be processed
            print("[Main] Motor shutdown commands sent")
        except Exception as e:
            print(f"[Main] Error during motor shutdown: {e}")
        
        # Clean shutdown of threads and camera
        # Stop user input first (no longer needed)
        stop_User_Input()
        
        # Give Arduino time to process final commands before closing serial
        time.sleep(0.5)
        
        # Stop serial reader LAST (after all commands sent)
        stop_serial_reader()
        
        # try:
        #     picam.stop()
        #     print("[Main] Camera stopped")
        # except:
        #     pass
        
        print("[Main] Shutdown complete")


if __name__ == "__main__":
    main()