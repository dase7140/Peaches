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
            print("Drop Tray")
            pi_2_ard("ABM")
            brushMotorOn = True
            brushMotorOnTime = time.time() * 1000  # current time in milliseconds
            lastTargetDirection = 0 #debugging
    else:
        print(f"Count =  {count}")


# Serial Communication Setup
port = "/dev/ttyACM0"      # Arduino port on Raspberry Pi
ser = serial.Serial(port, 115200, timeout=1) # establish serial connection
time.sleep(2)               # wait for the serial connection to initialize
ser.reset_input_buffer()    # clear input buffer to start fresh
ser.reset_output_buffer()   # clear output buffer to start fresh

###################
# Pi Camera 
###################
picam = Picamera2()
config = picam.create_preview_configuration(main={"size": (640, 480)})
picam.configure(config)
picam.start()

yellow_detected = False
flag = True



def capture_image():
    image_data = picam.capture_array()
    return image_data



def load_image_from_path(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image not found at path: {image_path}")
    return image   



def preprocess_frame(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    return hsv_frame



# Yellow Detection HSV Thresholds
# These can be tuned based on lighting conditions
# HSV ranges for yellow (adjust as needed)
YELLOW_LOWER_H = 20      # Lower hue bound (yellow starts around 20-30)
YELLOW_UPPER_H = 35      # Upper hue bound (yellow ends around 35-40)
YELLOW_LOWER_S = 50      # Lower saturation (increase to filter pale yellows)
YELLOW_UPPER_S = 255     # Upper saturation
YELLOW_LOWER_V = 50      # Lower value/brightness (increase in bright conditions)
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
    Computes the vertical error between the yellow centroid and image center.
    
    Args:
        centroid_y: int - Y coordinate of the yellow centroid
        image_height: int - Height of the camera frame in pixels
        
    Returns:
        error: float - Vertical offset from center
                      Positive = yellow is above center
                      Negative = yellow is below center
                      Zero = yellow is centered
    """
    # Calculate the center x-coordinate of the image
    image_center_y = image_height / 2
    
    # Compute error: positive means yellow is above center, negative means below
    error = centroid_y - image_center_y
    
    return error



# Steering Control Parameters
ERROR_THRESHOLD = 50        # Pixels - deadband for "centered" (adjust based on testing)
MAX_ERROR = 320             # Maximum possible error (half of image width)
BASE_SPEED = 180            # Base forward speed (matches Arduino speed variable)
MIN_TURN_SPEED = 100        # Minimum speed for gentle turns
MAX_TURN_SPEED = 180        # Maximum speed for sharp turns

# Proportional gain for steering correction
STEERING_KP = 0.5           # Adjust this to change steering responsiveness

def camera_based_steering(error):
    """
    Converts the yellow centroid error into motor control commands.
    Sends appropriate commands to Arduino via serial.
    
    Args:
        error: float - Vertical offset from center
                      Positive = yellow is above center (turn left)
                      Negative = yellow is below center (turn right)
                      
    Returns:
        command: str - The command sent to Arduino
    """
    
    # Check if yellow is centered (within deadband)
    if abs(error) < ERROR_THRESHOLD:
        # Yellow is centered - drive straight forward
        command = "MFD"
        pi_2_ard(command)
        print(f"[Steering] Centered (error={error:.1f}) -> Forward")
        return command
    
    # Calculate turn intensity based on error magnitude
    # Normalize error to 0.0 - 1.0 range
    error_normalized = min(abs(error) / MAX_ERROR, 1.0)
    
    # Calculate turn speed using proportional control
    turn_intensity = error_normalized * STEERING_KP
    turn_intensity = min(turn_intensity, 1.0)  # Cap at 1.0
    
    # Determine direction and send command
    if error > 0:
        # Yellow is to the RIGHT - turn right
        command = "MR0"
        pi_2_ard(command)
        print(f"[Steering] Error={error:.1f} -> Turn RIGHT (intensity={turn_intensity:.2f})")
        return command
    else:
        # Yellow is to the LEFT - turn left
        command = "ML0"
        pi_2_ard(command)
        print(f"[Steering] Error={error:.1f} -> Turn LEFT (intensity={turn_intensity:.2f})")
        return command



#### 
# IR Sensors
##########

# ...existing code...

# IR Sensor Configuration
IR_SENSOR_COUNT = 5
IR_SENSOR_NAMES = ["Left", "Front_Left", "Front_Right", "Right", "Back"]

# IR sensor index mapping (matches Arduino)
IR_LEFT = 0
IR_FRONT_LEFT = 1
IR_FRONT_RIGHT = 2
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
    Requests IR sensor data from Arduino and parses the response.
    
    Returns:
        IRSensorData: Object containing all 5 IR distance readings
                      Returns None if communication fails or timeout occurs
    """
    try:
        # Clear any old data in the buffer
        ser.reset_input_buffer()
        
        # Send command to Arduino to read IR sensors
        pi_2_ard("RIS")
        
        # Create sensor data object
        sensor_data = IRSensorData()
        sensor_data.timestamp = time.time()
        
        # Wait for and parse response from Arduino
        # Arduino sends multiple lines, we need to collect them
        timeout = time.time() + 2.0  # 2 second timeout
        distances_collected = 0
        
        while distances_collected < IR_SENSOR_COUNT and time.time() < timeout:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                
                # Skip empty lines and status messages
                if not line or line.startswith("ACK:") or line.startswith("IR Distances"):
                    continue
                
                # Parse lines like "0=450" or "1=320"
                if '=' in line:
                    try:
                        sensor_id, distance_str = line.split('=')
                        sensor_id = int(sensor_id.strip())
                        distance = int(distance_str.strip())
                        
                        # Store in raw array
                        if 0 <= sensor_id < IR_SENSOR_COUNT:
                            sensor_data.raw_distances[sensor_id] = distance
                            distances_collected += 1
                            
                    except (ValueError, IndexError) as e:
                        print(f"[IR] Parse error: {line} - {e}")
                        continue
            else:
                time.sleep(0.01)  # Small delay to avoid busy waiting
        
        # Check if we got all sensor readings
        if distances_collected == IR_SENSOR_COUNT:
            # Map to named fields
            sensor_data.left = sensor_data.raw_distances[IR_LEFT]
            sensor_data.front_left = sensor_data.raw_distances[IR_FRONT_LEFT]
            sensor_data.front_right = sensor_data.raw_distances[IR_FRONT_RIGHT]
            sensor_data.right = sensor_data.raw_distances[IR_RIGHT]
            sensor_data.back = sensor_data.raw_distances[IR_BACK]
            sensor_data.valid = True
            
            print(f"[IR] {sensor_data}")
            return sensor_data
        else:
            print(f"[IR] Timeout: Only received {distances_collected}/{IR_SENSOR_COUNT} readings")
            return None
            
    except Exception as e:
        print(f"[IR] Error reading sensors: {e}")
        return None
###################


# Obstacle Avoidance Parameters
CRITICAL_DISTANCE = 150      # mm - Emergency stop distance
WARNING_DISTANCE = 250       # mm - Slow down and prepare to turn
SIDE_WARNING_DISTANCE = 180  # mm - Side sensors warning threshold
BACK_WARNING_DISTANCE = 120  # mm - Back sensor threshold (for reversing)

class AvoidanceAction:
    """Class to hold obstacle avoidance decisions"""
    def __init__(self):
        self.override = False       # Should we override yellow-following?
        self.command = None         # Command to send to Arduino
        self.reason = ""            # Why we're overriding (for logging)
        self.priority = 0           # Priority level (higher = more urgent)

def avoid_obstacles(ir_data):
    """
    Analyzes IR sensor data to determine if obstacle avoidance is needed.
    
    Args:
        ir_data: IRSensorData object with current sensor readings
        
    Returns:
        AvoidanceAction: Object containing:
            - override: bool - True if obstacle avoidance should override camera
            - command: str - Arduino command to execute (or None)
            - reason: str - Explanation for logging
            - priority: int - Urgency level (0=none, 1=warning, 2=critical)
    """
    action = AvoidanceAction()
    
    # Check if sensor data is valid
    if not ir_data or not ir_data.valid:
        print("[Avoidance] Invalid sensor data - no override")
        return action
    
    # PRIORITY 1: CRITICAL FRONT OBSTACLES (Emergency Stop)
    if ir_data.front_left < CRITICAL_DISTANCE or ir_data.front_right < CRITICAL_DISTANCE:
        action.override = True
        action.command = "MF0"  # Stop
        action.reason = f"CRITICAL: Front obstacle detected (FL={ir_data.front_left}mm, FR={ir_data.front_right}mm)"
        action.priority = 3
        print(f"[Avoidance] {action.reason} -> STOP")
        return action
    
    # PRIORITY 2: FRONT WARNING (Slow down and turn away from closest obstacle)
    if ir_data.front_left < WARNING_DISTANCE or ir_data.front_right < WARNING_DISTANCE:
        action.override = True
        action.priority = 2
        
        # Determine which side has more clearance
        if ir_data.front_left < ir_data.front_right:
            # Left side is closer, turn right
            action.command = "MR0"
            action.reason = f"WARNING: Front-left obstacle (FL={ir_data.front_left}mm) -> Turn RIGHT"
        else:
            # Right side is closer, turn left
            action.command = "ML0"
            action.reason = f"WARNING: Front-right obstacle (FR={ir_data.front_right}mm) -> Turn LEFT"
        
        print(f"[Avoidance] {action.reason}")
        return action
    
    # PRIORITY 3: SIDE OBSTACLES (Gentle steering correction)
    if ir_data.left < SIDE_WARNING_DISTANCE:
        action.override = True
        action.command = "MR0"  # Turn right to avoid left obstacle
        action.reason = f"Side obstacle LEFT (L={ir_data.left}mm) -> Steer RIGHT"
        action.priority = 1
        print(f"[Avoidance] {action.reason}")
        return action
    
    if ir_data.right < SIDE_WARNING_DISTANCE:
        action.override = True
        action.command = "ML0"  # Turn left to avoid right obstacle
        action.reason = f"Side obstacle RIGHT (R={ir_data.right}mm) -> Steer LEFT"
        action.priority = 1
        print(f"[Avoidance] {action.reason}")
        return action
    
    # PRIORITY 4: BACK OBSTACLE (Stop reversing if backing up)
    if ir_data.back < BACK_WARNING_DISTANCE:
        action.override = True
        action.command = "MF0"  # Stop
        action.reason = f"Back obstacle detected (B={ir_data.back}mm) -> STOP"
        action.priority = 2
        print(f"[Avoidance] {action.reason}")
        return action
    
    # No obstacles detected - allow normal yellow-following operation
    action.override = False
    action.command = None
    action.reason = "Clear path"
    action.priority = 0
    
    return action


def decide_motion(camera_command, ir_data):
    """
    Unified control function that determines the robot's motion.
    Prioritizes obstacle avoidance over camera-based yellow following.
    
    Args:
        camera_command: str - Command from camera-based steering (e.g., "MFD", "ML0", "MR0")
        ir_data: IRSensorData - Current IR sensor readings
        
    Returns:
        dict: {
            'command': str - Final command to send to Arduino,
            'source': str - Either 'obstacle_avoidance' or 'camera_steering',
            'reason': str - Explanation of the decision,
            'priority': int - Priority level (0=normal, 1-3=avoidance)
        }
    """
    
    # First, check for obstacles (highest priority)
    avoidance = avoid_obstacles(ir_data)
    
    # If obstacle avoidance needs to override
    if avoidance.override:
        decision = {
            'command': avoidance.command,
            'source': 'obstacle_avoidance',
            'reason': avoidance.reason,
            'priority': avoidance.priority
        }
        print(f"[Decision] OVERRIDE: {avoidance.reason}")
        return decision
    
    # No obstacles detected - use camera-based steering
    decision = {
        'command': camera_command,
        'source': 'camera_steering',
        'reason': 'Following yellow line',
        'priority': 0
    }
    print(f"[Decision] Camera control: {camera_command}")
    return decision



# Functions for Serial Communication
# Send command from Pi to Arduino
def pi_2_ard(command):
    try:
        ser.write((command + '\n').encode('utf-8')) # send command to Arduino
        ser.flush()                                 # ensure command is sent
    
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


# ...existing code...

# Debug Configuration Flags
DEBUG_ENABLED = True              # Master debug switch
DEBUG_SHOW_MASK = True           # Show binary mask window
DEBUG_SHOW_CLEANED = True        # Show cleaned mask window
DEBUG_SHOW_FRAME = True          # Show original frame with annotations
DEBUG_PRINT_HSV = False          # Print HSV threshold info
DEBUG_PRINT_STEERING = True      # Print steering decisions
DEBUG_PRINT_IR = True            # Print IR sensor readings
DEBUG_SAVE_FRAMES = False        # Save debug frames to disk
DEBUG_FRAME_SAVE_PATH = "debug_frames/"  # Path for saved frames

# Create debug frame directory if needed
if DEBUG_SAVE_FRAMES:
    import os
    os.makedirs(DEBUG_FRAME_SAVE_PATH, exist_ok=True)

def debug_visualize(frame, hsv, mask, cleaned, found, cx, cy, error, camera_cmd, decision, frame_count):
    """
    Displays debug visualization windows showing processing pipeline.
    
    Args:
        frame: Original BGR frame
        hsv: HSV converted frame
        mask: Raw yellow mask
        cleaned: Cleaned yellow mask
        found: Whether yellow was detected
        cx, cy: Centroid coordinates
        error: Steering error
        camera_cmd: Camera steering command
        decision: Final motion decision
        frame_count: Current frame number
    """
    if not DEBUG_ENABLED:
        return
    
    image_height, image_width = frame.shape[:2]
    
    # Create annotated frame copy
    annotated_frame = frame.copy()
    
    # Draw image center line (reference)
    center_y = image_height // 2
    cv2.line(annotated_frame, (0, center_y), (image_width, center_y), (0, 255, 255), 2)
    cv2.putText(annotated_frame, "CENTER", (10, center_y - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    # Draw centroid and error if yellow found
    if found:
        # Draw centroid as circle
        cv2.circle(annotated_frame, (cx, cy), 10, (0, 255, 0), -1)
        cv2.putText(annotated_frame, f"({cx},{cy})", (cx + 15, cy), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw line from center to centroid
        cv2.line(annotated_frame, (cx, center_y), (cx, cy), (255, 0, 255), 2)
        
        # Display error value
        error_text = f"Error: {error:.1f}px"
        cv2.putText(annotated_frame, error_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    else:
        cv2.putText(annotated_frame, "NO YELLOW DETECTED", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    # Display camera command
    cmd_color = (0, 255, 0) if decision['source'] == 'camera_steering' else (0, 165, 255)
    cv2.putText(annotated_frame, f"Cmd: {camera_cmd}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, cmd_color, 2)
    
    # Display decision source
    source_text = f"Source: {decision['source']}"
    cv2.putText(annotated_frame, source_text, (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Display priority level
    priority_text = f"Priority: {decision['priority']}"
    cv2.putText(annotated_frame, priority_text, (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Display frame count
    cv2.putText(annotated_frame, f"Frame: {frame_count}", (10, image_height - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Show windows
    if DEBUG_SHOW_FRAME:
        cv2.imshow("Debug: Annotated Frame", annotated_frame)
    
    if DEBUG_SHOW_MASK:
        cv2.imshow("Debug: Raw Yellow Mask", mask)
    
    if DEBUG_SHOW_CLEANED:
        cv2.imshow("Debug: Cleaned Mask", cleaned)
    
    # Save frames if enabled
    if DEBUG_SAVE_FRAMES and frame_count % 10 == 0:  # Save every 10th frame
        cv2.imwrite(f"{DEBUG_FRAME_SAVE_PATH}frame_{frame_count:04d}.jpg", annotated_frame)
        cv2.imwrite(f"{DEBUG_FRAME_SAVE_PATH}mask_{frame_count:04d}.jpg", cleaned)
    
    # Wait for key press (1ms) - allows window updates
    cv2.waitKey(1)

def debug_print_hsv_thresholds():
    """Prints current HSV threshold values for yellow detection."""
    if not DEBUG_ENABLED or not DEBUG_PRINT_HSV:
        return
    
    print("\n" + "="*50)
    print("HSV YELLOW DETECTION THRESHOLDS")
    print("="*50)
    print(f"Hue:        {YELLOW_LOWER_H} - {YELLOW_UPPER_H}")
    print(f"Saturation: {YELLOW_LOWER_S} - {YELLOW_UPPER_S}")
    print(f"Value:      {YELLOW_LOWER_V} - {YELLOW_UPPER_V}")
    print("="*50 + "\n")

def debug_print_steering(error, camera_cmd, found):
    """Prints detailed steering information."""
    if not DEBUG_ENABLED or not DEBUG_PRINT_STEERING:
        return
    
    if found:
        direction = "RIGHT" if error > 0 else "LEFT" if error < 0 else "CENTERED"
        print(f"[Steering Debug] Error: {error:+7.1f}px | Direction: {direction:8s} | Command: {camera_cmd}")
    else:
        print(f"[Steering Debug] NO YELLOW - Searching")

def debug_print_ir_sensors(ir_data):
    """Prints formatted IR sensor data."""
    if not DEBUG_ENABLED or not DEBUG_PRINT_IR:
        return
    
    if ir_data and ir_data.valid:
        print(f"[IR Debug] L:{ir_data.left:4d} | FL:{ir_data.front_left:4d} | "
              f"FR:{ir_data.front_right:4d} | R:{ir_data.right:4d} | B:{ir_data.back:4d}")
    else:
        print("[IR Debug] Invalid sensor data")

def debug_print_decision(decision):
    """Prints detailed decision information."""
    if not DEBUG_ENABLED:
        return
    
    priority_label = ["NORMAL", "LOW", "MEDIUM", "HIGH"][min(decision['priority'], 3)]
    print(f"[Decision Debug] {decision['source']:20s} | "
          f"Priority: {priority_label:6s} | "
          f"Command: {decision['command']:4s} | "
          f"Reason: {decision['reason']}")

def debug_print_performance(fps, loop_time):
    """Prints performance metrics."""
    if not DEBUG_ENABLED:
        return
    
    status = "OK" if fps >= 15 else "SLOW"
    print(f"[Performance] FPS: {fps:5.1f} | Loop: {loop_time:6.1f}ms | Status: {status}")


# Main Loop Configuration
TARGET_FPS = 20                    # Target frames per second
FRAME_DELAY = 1.0 / TARGET_FPS    # Delay between frames (0.05s = 20 FPS)
IR_READ_INTERVAL = 5              # Read IR sensors every N frames (reduces overhead)

def main_loop():
    """
    Main control loop that:
    1. Captures and processes camera frames
    2. Detects yellow and computes steering
    3. Reads IR sensors periodically
    4. Combines decisions with priority (obstacles > camera)
    5. Sends commands to Arduino
    
    Runs at real-time speed (target 15-20 FPS)
    """
    print("[Main Loop] Starting yellow-following with obstacle avoidance...")
    print("[Main Loop] Press Ctrl+C to stop")
    
    # Print HSV thresholds at startup
    debug_print_hsv_thresholds()
    
    frame_count = 0
    loop_start_time = time.time()
    last_ir_data = None
    
    try:
        while True:
            loop_iteration_start = time.time()
            
            # ===== 1. CAPTURE FRAME =====
            frame = capture_image()
            if frame is None:
                print("[Main Loop] Failed to capture frame")
                time.sleep(FRAME_DELAY)
                continue
            
            image_height, image_width = frame.shape[:2]
            
            # ===== 2. PROCESS FRAME FOR YELLOW DETECTION =====
            hsv = preprocess_frame(frame)
            mask = yellow_mask(hsv)
            cleaned = clean_mask(mask)
            found, cx, cy = find_yellow_centroid(cleaned)
            
            # ===== 3. COMPUTE CAMERA STEERING COMMAND =====
            if found:
                # Camera is sideways, so use cy (vertical) with image_height for steering
                error = compute_error(cy, image_height)
                
                # Determine camera command WITHOUT sending it yet
                if abs(error) < ERROR_THRESHOLD:
                    camera_cmd = "MFD"
                elif error > 0:
                    camera_cmd = "MR0"  # Yellow above center -> turn right
                else:
                    camera_cmd = "ML0"  # Yellow below center -> turn left
                
                # Debug print steering
                debug_print_steering(error, camera_cmd, found)
            else:
                # No yellow detected - search for it
                camera_cmd = "YLL"
                debug_print_steering(0, camera_cmd, found)
            
            # ===== 4. READ IR SENSORS (periodically to reduce overhead) =====
            if frame_count % IR_READ_INTERVAL == 0:
                last_ir_data = read_ir_sensors()
                debug_print_ir_sensors(last_ir_data)
            
            # ===== 5. DECIDE FINAL MOTION (obstacle avoidance has priority) =====
            decision = decide_motion(camera_cmd, last_ir_data)
            debug_print_decision(decision)
            
            # ===== 6. SEND COMMAND TO ARDUINO =====
            if decision['source'] == 'obstacle_avoidance':
                pi_2_ard(decision['command'])
            elif decision['source'] == 'camera_steering':
                pi_2_ard(camera_cmd)
            
            # ===== 7. DEBUG VISUALIZATION =====
            debug_visualize(frame, hsv, mask, cleaned, found, cx, cy, error if found else 0, 
                          camera_cmd, decision, frame_count)
            
            # ===== 8. FRAME TIMING AND PERFORMANCE MONITORING =====
            frame_count += 1
            loop_iteration_time = time.time() - loop_iteration_start
            
            # Calculate FPS every 20 frames
            if frame_count % 20 == 0:
                elapsed = time.time() - loop_start_time
                current_fps = 20 / elapsed
                debug_print_performance(current_fps, loop_iteration_time * 1000)
                loop_start_time = time.time()
            
            # Maintain target frame rate
            if loop_iteration_time < FRAME_DELAY:
                time.sleep(FRAME_DELAY - loop_iteration_time)
            else:
                if DEBUG_ENABLED:
                    print(f"[Warning] Loop running slow: {loop_iteration_time*1000:.1f}ms (target: {FRAME_DELAY*1000:.1f}ms)")
    
    except KeyboardInterrupt:
        print("\n[Main Loop] Stopped by user")
        pi_2_ard("MF0")  # Stop robot
        print("[Main Loop] Robot stopped")
        
        # Close all debug windows
        if DEBUG_ENABLED:
            cv2.destroyAllWindows()
    
    except Exception as e:
        print(f"[Main Loop] Error: {e}")
        pi_2_ard("MF0")  # Stop robot on error
        
        # Close all debug windows
        if DEBUG_ENABLED:
            cv2.destroyAllWindows()
        raise



def main():

    if not wait_for_start():
        return
    
    reader = threading.Thread(target = serial_reader, daemon=True)
    reader.start()

    # Start the main yellow-following loop
    main_loop()

if __name__ == "__main__":
    main()