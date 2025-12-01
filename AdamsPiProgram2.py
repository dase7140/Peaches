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

# Note: Pixy library may print "error: no response" to stdout if USB communication fails.
# This is a library-level message and cannot be suppressed from Python.
# The Pixicam() function handles errors gracefully by returning False.

pixy.init()
pixy.change_prog("color_connected_components")

target_signature = 1

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
    """
    Queries Pixy camera for color blocks matching target signature.
    
    Returns:
        bool - True if target detected, False otherwise
        
    Note: Pixy library may print "error: no response" to stdout if
    camera communication fails. This is a library message, not from our code.
    """
    try:
        count = pixy.ccc_get_blocks(100, blocks)
        #now is for incase we need a search timeout
        now = int(round(time.time() * 1000))

        if count > 0:
            targetSeen = seeColor(target_signature, count)
            targetX = getTargetX(target_signature, count)

            if targetSeen and targetX != -1:
                return True
            
            return False  # No matching signature found
        else:
            return False
    except Exception as e:
        # Pixy communication error - return False to avoid crashing
        # (Pixy library will already have printed error message)
        return False


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
    global serial_reader_running, estop_triggered
    
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
picam = Picamera2()
config = picam.create_preview_configuration(main={"size": (640, 480)})
picam.configure(config)
picam.start()


def capture_image():
    image_data = picam.capture_array()
    blurred = cv2.GaussianBlur(image_data, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    return hsv_frame

def load_image_from_path(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found at path: {image_path}")
    return image   


# Yellow Detection HSV Thresholds
# These can be tuned based on lighting conditions
# HSV ranges for yellow (adjust as needed)
YELLOW_LOWER_H = 80      # Lower hue bound
YELLOW_UPPER_H = 135      # Upper hue bound
YELLOW_LOWER_S = 35      # Lower saturation (increase to filter pale yellows)
YELLOW_UPPER_S = 255     # Upper saturation
YELLOW_LOWER_V = 60      # Lower value/brightness (increase in bright conditions)
YELLOW_UPPER_V = 255     # Upper value/brightness


def yellow_mask(hsv_frame):
    # Define lower and upper bounds for yellow in HSV
    lower_yellow = np.array([YELLOW_LOWER_H, YELLOW_LOWER_S, YELLOW_LOWER_V])
    upper_yellow = np.array([YELLOW_UPPER_H, YELLOW_UPPER_S, YELLOW_UPPER_V])
    # Create binary mask: pixels within range are white (255), others are black (0)
    mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
    return mask



# Morphological operation parameters (adjust for your needs)
MORPH_KERNEL_SIZE = 5       # Size of structuring element (5x5 is a good start)
MORPH_OPEN_ITERATIONS = 2   # Number of opening operations (removes small noise)
MORPH_CLOSE_ITERATIONS = 3  # Number of closing operations (fills small gaps)

def clean_mask(mask):
    # Create structuring element (kernel) for morphological operations
    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
    
    # Opening: Erosion followed by Dilation
    # Removes small white noise/spots while preserving larger shapes
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=MORPH_OPEN_ITERATIONS)
    
    # Closing: Dilation followed by Erosion
    # Fills small black holes inside white regions and connects nearby regions
    cleaned_mask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=MORPH_CLOSE_ITERATIONS)
    
    return cleaned_mask


def find_yellow_ellipse(mask):
    """
    Fits an elipse to the largest yellow contour in the mask

    Args
        maskL numpy array - binary mask with yellow regions
    
    Returns:
        Tuple: (found, cy, angle)
        found: bool - true if yellow region was detected
        cy: int - y coordinate of centroid
        angle: int - angle between the horizontal axis and the major axis of the ellipse
    """
    # Find all contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Check if any contours were found
    if len(contours) == 0:
        return False, 0, 0
    
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Filter out very small contours (likely noise)
    MIN_CONTOUR_AREA = 500  # Adjust this threshold as needed
    if cv2.contourArea(largest_contour) < MIN_CONTOUR_AREA:
        return False, 0, 0
    
    ellipse = cv2.fitEllipse(largest_contour)
    ((_, cy), (_, _), angle) = ellipse

    return True, cy, angle

def find_yellow_centroid(mask):
    """
    Finds the largest yellow contour in the mask and computes its centroid.
    
    Args:
        mask: numpy array - Binary mask with yellow regions
        
    Returns:
        tuple: (found, cx, cy)
            found: bool - True if yellow region was detected
            cx: int - X coordinate of centroid (0 if not found)
            cy: int - Y coordinate of centroid (0 if not found)
    """
    # Find all contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Check if any contours were found
    if len(contours) == 0:
        return False, 0, 0
    
    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Filter out very small contours (likely noise)
    MIN_CONTOUR_AREA = 500  # Adjust this threshold as needed
    if cv2.contourArea(largest_contour) < MIN_CONTOUR_AREA:
        return False, 0, 0
    
    # Calculate moments of the largest contour
    M = cv2.moments(largest_contour)
    
    # Compute centroid coordinates
    # Check for division by zero
    if M["m00"] == 0:
        return False, 0, 0
    
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    
    return True, cx, cy



def compute_error(centroid_y, image_height):
    """
    Computes the vertical error between the yellow centroid and the target position.
    Camera is mounted UPSIDE DOWN on the LEFT side of the robot.
    
    Target position is the center of the BOTTOM HALF of the image (at 3/4 from top).
    
    Args:
        centroid_y: int - Y coordinate of the yellow centroid
        image_height: int - Height of the camera frame in pixels
        
    Returns:
        error: float - Vertical offset from target position
                      Positive = yellow is too far (robot needs to turn left)
                      Negative = yellow is too close (robot needs to turn right)
                      Zero = yellow is at target position (go straight)
    """
    # Calculate the target y-coordinate: center of the image
    target_y = image_height / 2
    
    # Compute error: positive means yellow is below target (higher Y) = too far in real world
    # Since camera is upside down: high Y value in image = far from robot in real world
    error = centroid_y - target_y
    
    return error


# Yellow Line Following Control Parameters
ERROR_THRESHOLD = 50        # Deadband in pixels - go straight if error < this
BASE_SPEED = 2              # Default speed level (1-5)
TURN_SPEED = 2              # Speed when turning to follow line


def yellow_line_steering(error):
    """
    Converts centroid error to motor steering command.
    
    Camera mounted upside down on LEFT side:
    - Positive error = yellow too far (top of upside-down image) → turn LEFT
    - Negative error = yellow too close (bottom of upside-down image) → turn RIGHT
    - Small error = go straight FORWARD
    
    Args:
        error: float - Error from compute_error()
        
    Returns:
        str - Motor command ("MF2", "ML2", "MR2", etc.)
    """
    # Within deadband - go straight
    if abs(error) < ERROR_THRESHOLD:
        return f"MF{BASE_SPEED}"
    
    # Positive error - yellow is too far, turn LEFT to get closer
    elif error > 0:
        return f"ML{TURN_SPEED}"
    
    # Negative error - yellow is too close, turn RIGHT to move away
    else:
        return f"MR{TURN_SPEED}"






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



def reposition():
    """
    Intelligent repositioning function when obstacle detected.
    
    Strategy:
    1. Read IR sensors to assess surroundings
    2. If back is clear (>100mm), reverse to create space
    3. Calculate left area (front_left + left) vs right area (front_right + right)
    4. Turn towards the direction with more clearance
    
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
        pi_2_ard("MB2")  # Reverse at speed 2
        time.sleep(0.1)  # Reverse for 100ms
        pi_2_ard("MF0")  # Stop motors
        time.sleep(0.1)
    else:
        print(f"[Reposition] Back blocked ({back_distance}mm) - skipping reverse")
    
    # Step 3: Cap sensor readings and calculate left and right clearance areas
    front_left_capped = cap_distance(ir_data.front_left)
    left_capped = cap_distance(ir_data.left)
    front_right_capped = cap_distance(ir_data.front_right)
    right_capped = cap_distance(ir_data.right)
    
    left_area = front_left_capped*2 + left_capped
    right_area = front_right_capped*2 + right_capped
    
    print(f"[Reposition] Left area: {left_area}mm (FL:{front_left_capped} + L:{left_capped})")
    print(f"[Reposition] Right area: {right_area}mm (FR:{front_right_capped} + R:{right_capped})")
    
    # Step 4: Turn towards the direction with more clearance
    MIN_AREA_DIFFERENCE = 100  # mm - Minimum difference to prefer one side
    MINIMUM_CLEARANCE = 150     # mm - Minimum total area to consider turning
    
    if left_area < MINIMUM_CLEARANCE and right_area < MINIMUM_CLEARANCE:
        print(f"[Reposition] Both sides blocked (L:{left_area}, R:{right_area}) - attempting large turn")
        pi_2_ard("ML1")  # Turn left
        time.sleep(0.2)  # Turn for 200ms 
    elif left_area > right_area:
        print(f"[Reposition] Turning LEFT (left area larger by {left_area - right_area}mm)")
        pi_2_ard("ML1")  # Turn left at speed 2
        time.sleep(0.2)  # Turn for 200ms
    else:
        print(f"[Reposition] Turning RIGHT (right area larger by {right_area - left_area}mm)")
        pi_2_ard("MR1")  # Turn right at speed 1
        time.sleep(0.2)  # Turn for 200ms
    
    pi_2_ard("MF0")  # Stop motors
    
    print("[Reposition] Repositioning complete")
    return True



def drive():
    """
    Main driving control loop.
    Monitors for stop conditions and handles motor shutdown.
    Manages Pixicam target detection and brush motor control.
    """
    global user_stop_requested, estop_triggered
    
    print("[Drive] Starting main control loop")
    print("[Drive] Type 'STOP' at any time to emergency stop")
    
    # Brush motor state management
    brush_motor_active = False
    brush_motor_off_time = None
    BRUSH_MOTOR_DELAY = 5.0  # seconds to keep brush on after target disappears
    
    # Yellow line following state management
    last_steering_cmd = None
    yellow_lost_counter = 0
    YELLOW_LOST_THRESHOLD = 5  # frames - tolerate brief occlusions
    SEARCH_ENABLED = True
    search_start_time = None
    SEARCH_TIMEOUT = 3.0  # seconds - max time to search before stopping
    
    # Pixy camera error tracking
    pixy_error_count = 0
    MAX_PIXY_ERRORS = 10  # Warn after this many consecutive errors
    
    try:
        while True:
            # Check for user stop command
            if user_stop_requested:
                print("\n[Drive] User STOP command received - shutting down motors")
                pi_2_ard("MF0", max_retries=5, timeout=0.2)  # Stop motors
                pi_2_ard("DBM", max_retries=3, timeout=0.2)  # Turn off brush motor
                break
            
            # Check for Arduino emergency stop
            if estop_triggered:
                print("\n[Drive] Arduino ESTOP triggered - initiating repositioning")
                estop_triggered = False  # Reset flag after handling
                # Arduino has already stopped motors and started grace period
                # Now reposition to find clear path
                if not reposition():
                    print("[Drive] Repositioning failed - staying stopped")
                    time.sleep(0.5)
                else:
                    # Repositioning succeeded - reset steering state to resume driving
                    print("[Drive] Repositioning succeeded - resuming yellow line following")
                    last_steering_cmd = None  # Force next steering command to be sent
            
            # ===== YELLOW LINE/WALL FOLLOWING (Priority: Always check first) =====
            # This runs FIRST to ensure continuous line following
            # Capture and process frame
            hsv_frame = capture_image()
            
            # Create yellow mask
            mask = yellow_mask(hsv_frame)
            
            # Clean up noise
            cleaned_mask = clean_mask(mask)
            
            # Find yellow centroid
            found, cx, cy = find_yellow_centroid(cleaned_mask)
            
            if found:
                # Reset lost counter and search timer - yellow is visible
                yellow_lost_counter = 0
                search_start_time = None
                
                # Get image dimensions
                image_height, image_width = hsv_frame.shape[:2]
                
                # Compute error (camera is upside down on left side)
                error = compute_error(cy, image_height)
                
                # Generate steering command
                steering_cmd = yellow_line_steering(error)
                
                # Only send command if it's different from last time
                if steering_cmd != last_steering_cmd:
                    pi_2_ard(steering_cmd)
                    last_steering_cmd = steering_cmd
                    
                    # Debug output when command changes
                    if abs(error) > ERROR_THRESHOLD:
                        print(f"[YellowFollow] Error: {error:.1f}px → {steering_cmd}")
                    else:
                        print(f"[YellowFollow] Centered → {steering_cmd}")
            
            else:
                # Yellow not detected - increment lost counter
                yellow_lost_counter += 1
                
                # Only react after several consecutive lost frames
                if yellow_lost_counter >= YELLOW_LOST_THRESHOLD:
                    
                    if SEARCH_ENABLED:
                        # Start search timer on first detection
                        if yellow_lost_counter == YELLOW_LOST_THRESHOLD:
                            print("[YellowFollow] Yellow line lost - starting search pattern")
                            search_start_time = time.time()
                            search_cmd = "ML1"  # Slow turn left to search
                            if search_cmd != last_steering_cmd:
                                pi_2_ard(search_cmd)
                                last_steering_cmd = search_cmd
                        
                        # Check if search has timed out
                        elif search_start_time is not None and (time.time() - search_start_time) > SEARCH_TIMEOUT:
                            print(f"[YellowFollow] Search timeout after {SEARCH_TIMEOUT}s - stopping")
                            stop_cmd = "MF0"
                            if stop_cmd != last_steering_cmd:
                                pi_2_ard(stop_cmd)
                                last_steering_cmd = stop_cmd
                            search_start_time = None  # Reset for next search
                    
                    elif not SEARCH_ENABLED and yellow_lost_counter == YELLOW_LOST_THRESHOLD:
                        # Search disabled - just stop
                        print("[YellowFollow] Yellow line lost - stopping")
                        stop_cmd = "MF0"
                        if stop_cmd != last_steering_cmd:
                            pi_2_ard(stop_cmd)
                            last_steering_cmd = stop_cmd
                
                else:
                    # Still within tolerance - keep last command
                    # Reset search timer since we're still within threshold
                    search_start_time = None
            
            # ===== PIXICAM TARGET DETECTION AND BRUSH MOTOR CONTROL =====
            # This runs AFTER yellow line following to avoid blocking steering
            try:
                target_detected = Pixicam()
                pixy_error_count = 0  # Reset error count on successful read
            except Exception as e:
                # Pixy error - assume no target detected
                target_detected = False
                pixy_error_count += 1
                
                # Warn if errors persist
                if pixy_error_count == MAX_PIXY_ERRORS:
                    print(f"[Pixy] WARNING: {MAX_PIXY_ERRORS} consecutive errors - camera may be disconnected")
                    print("[Pixy] Continuing without target detection...")
            
            # Target detected - activate brush motor
            if target_detected:
                if not brush_motor_active:
                    print("[Drive] Target detected - activating brush motor")
                    pi_2_ard("ABM")
                    brush_motor_active = True
                    # Reset steering command so next yellow line command will be sent
                    last_steering_cmd = None
                # Cancel any pending shutdown timer only if brush motor is active
                # This prevents flickering detections from resetting the timer
                if brush_motor_active:
                    brush_motor_off_time = None
            
            # Target lost - start shutdown timer (only if motor was active)
            elif not target_detected and brush_motor_active:
                if brush_motor_off_time is None:
                    print(f"[Drive] Target lost - brush motor will turn off in {BRUSH_MOTOR_DELAY}s")
                    brush_motor_off_time = time.time() + BRUSH_MOTOR_DELAY
            
            # Check if it's time to turn off brush motor
            if brush_motor_off_time is not None and time.time() >= brush_motor_off_time:
                print("[Drive] Turning off brush motor")
                pi_2_ard("DBM")
                brush_motor_active = False
                brush_motor_off_time = None
                # Reset steering command so next yellow line command will be sent
                last_steering_cmd = None
            
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
        drive()
    finally:
        # Emergency shutdown - ensure motors are stopped
        print("\n[Main] Shutting down...")
        try:
            pi_2_ard("MF0", max_retries=5, timeout=0.2)
            pi_2_ard("DBM", max_retries=3, timeout=0.2)
        except:
            pass
        
        # Clean shutdown of threads and camera
        stop_User_Input()
        stop_serial_reader()
        
        try:
            picam.stop()
            print("[Main] Camera stopped")
        except:
            pass
        
        print("[Main] Shutdown complete")


if __name__ == "__main__":
    main()