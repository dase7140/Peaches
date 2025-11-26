# main loop should:
	#Enter a specific based on the current mode, execute descision, and re-evaluate sensor data to set next mode
	#Modes: gravel, bridge, dice_track, go, turn_left, turn_right, reverse, stop
	# go - align with the tape and move forwards. Use sensor readings to avoid obstacles and detext when to turn
	# turn_left/right - once sensor readings indicate we're close to a wall in front, determine turn direction based on which side is more open. turn in place until x

# Parallel loops check sensors/cams, update variables
	

# Pi communication protocol: 7 character message; ex: L0099D1
	# First character: Direction (L, R, B) left, right, backwards
	# Next 2 chatacters: Angle (00-99, XX) - gives percentage faster to turn motor indicated by previous character. If XX, turn in place (drive one backwards)
	# Next 2 characters: Speed (00-99) - gives percentage of max speed to spin drive 
	# 6th character: D or U - gives position of collection tray, down or up
	# Last character: (0,9,R) - gives speed of brush motor (0-9), or R for reverse (reverse slowly, just to lift brush out of way)

# Arduino communication protocol to communicate sensor readings:
	#Message Format: SENS USF_L=123 USF_R=128 USS=87 USL=210 USR=199 IRF_L=1 IRF_R=0 IRS=0 IRL=1 IRR=0\n 
	#contains distance readings for all ultrasonic and IR sensors, begins with SENS to distinguish from other messages, ends with newline character
	#If sensor does not work or garbage data, write USF_L = -1, and we will ignore it in the logic.
import pixy
from ctypes import *
from pixy import *
import time
import serial
from picamera2 import Picamera2
import cv2
import numpy as np
import threading

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)


# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()



#Pixy Init:
#1 = red, 2 = green 3 = blue 4= yellow 5 = purple (GRAVEL)  6 = pink (RAMP)
centerX = 157
deadband = 30
targetSignature = 3 
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

state_lock = threading.Lock()
robot_state = {
    # current mode:
    "mode" : "go", # go, turn_left, turn_right, reverse, stop, gravel, bridge, dice_track
      
	# Pi Camera color detection  
      #ask adam about readings from camera, what variables we have, etc.
      "yellow_detected": False,

	# Pixy camera block detection
      "pixy_block_count": 0,
      "pixy_blocks": [],
	  "dice_visible": False,

	# Pixy camera flag for gravel/ramp detection
	  "gravel_flag_detected": False,
	  "ramp_flag_detected": False,
      
	# Arduino Sensor Readings
      "sensors": {
      "USFL": -1,
      "USFR": -1,
      "USL": -1,
      "USR": -1,
      "USB": -1,
      
	  "IRFL": -1,
	  "IRFR": -1,
	  "IRL": -1,
	  "IRR": -1,
	  "IRB": -1,
	  },
        
	# Other variables as needed
      "last_arduino_comm" : 0.0,
      "last_pixy_time" : 0.0,
      "last_camera_time" : 0.0,

	# Variables calculated from sensor data
      "front_obstacle_distance": -1,

	# Timing variables used in modes
	  "turn_start_time" : None,
	  "reverse_start_time" : None,
	  "bridge_start_time" : None,
	  "bridge_aligned": False,
	  
}

def command_robot(direction, angle, speed, tray_position, brush_speed):
	"""
    Build 7-character command string for Arduino.

    direction : one-char: 'L','R','B' (left, right, backward)
    angle     : int 00–99 OR 'XX' string for in-place turn
    speed     : int 00–99 
    tray      : 'D' or 'U'
    brush     : int 0–9 or 'R'
    """
	if direction not in ['L','R','B']:
		raise ValueError("Invalid direction")
	
	if isinstance(angle, float):
		if not (0 <= angle <= 99):
			raise ValueError("Invalid angle")
		else:
			angle = f"{angle:02}"
	elif angle != 'XX':
		raise ValueError("Invalid angle")
	else:
		angle = 'XX'
	
	if not (0 <= speed <= 99):
		raise ValueError("Invalid speed")
	else:
		speed = f"{speed:02}"
	
	if tray_position not in ['D','U']:
		raise ValueError("Invalid tray position")
	
	if isinstance(brush_speed, int):
		if not (0 <= brush_speed <= 9):
			raise ValueError("Invalid brush speed")
		else:
			brush_speed = f"{brush_speed:01}"
	elif brush_speed != 'R':
		raise ValueError("Invalid brush speed")
	else:
		brush_speed = 'R'


	command = f"{direction}{angle:02}{speed:02}{tray_position}{brush_speed}"
	if len(command) != 7:
		raise ValueError("Command length incorrect")
	
	print(f"Sending command: {command}")
	arduino.write(command.encode())

# sepatarate thread to read arduino sensor data, parses and updates robot_state. Other messages such as errors should be parsed here as well.
# def read_arduino_thread(ser):
# 	"""
# 	Thread: continuously read lines from Arduino, parse SENS packets.
# 	Expected format:
# 		SENS USF_L=123 USF_R=128 USS=87 USL=210 USR=199 IRF_L=1 IRF_R=0 IRS=0 IRL=1 IRR=0
# 	If a value cannot be parsed, put -1 for that key.
# 	"""
# 	while True:
# 		try:
# 			line = ser.readline().decode(errors="ignore").strip()
# 			if not line.startswith("SENS"):
# 				continue

# 			parts = line.split()
# 			parsed = {}

# 			for p in parts[1:]:
# 				if "=" in p:
# 					key, val = p.split("=")
# 					try:
# 						parsed[key] = int(val)
# 					except:
# 						parsed[key] = -1

# 			with state_lock:
# 				# Update only the keys that exist in robot_state
# 				for key in robot_state["sensors"]:
# 					if key in parsed:
# 						robot_state["sensors"][key] = parsed[key]

# 				robot_state["last_arduino_comm"] = time.time()

# 		except Exception as e:
# 			print("Arduino read error:", e)
# 			time.sleep(0.01)

# # Start Arduino reading thread
# arduino_thread = threading.Thread(
# 	target=read_arduino_thread, 
# 	args=(arduino,),
# 	daemon=True
# )
# arduino_thread.start()

# Thread to read Pixy blocks, update robot_state - Need to ask dania about finishing this and adding relevent variables to robot_state
def read_pixy_thread():
	"""
	Thread: continuously read blocks from Pixy, update robot_state.
	"""


pixy_thread = threading.Thread(
	target=read_pixy_thread, 
	args=(),
	daemon=True
)
pixy_thread.start()

# Thread to read PiCamera2 frames, process for color detection, update robot_state - Need to ask adam about finishing this and adding relevent variables to robot_state
def update_piCam_thread():
	"""
	Thread: continuously capture frames from PiCamera2, process for color detection, update robot_state.
	"""


piCam_thread = threading.Thread(
	target=update_piCam_thread, 
	args=(),
	daemon=True
)
piCam_thread.start()

def calculate_centering(USL, USR,IRL, IRR):
	"""
	Calculate adjustment angle based on ultrasonic and IR sensor readings.
	negative value means turn right, positive means turn left.
	"""
	# Example logic: average distance difference
	ultrasonic_error = USL - USR if USL >=0 and USR >=0 else 0
	ir_error = IRL - IRR if IRL >=0 and IRR >=0 else 0
	
	if abs(ultrasonic_error) > 100 and abs(ir_error) > 100: #implies open wall, on side, don't adjust TODO test bounds
		return 0

	# Weighted sum (weights can be adjusted)
	total_error = (ultrasonic_error * 0.5) + (ir_error * 0.5) # need to calibrate this weighting so we don't overcorrect; should return 
	if abs(total_error) > 99:  # deadband threshold
		return 99 if total_error > 0 else -99
	
	return total_error

def calculate_obstacle_distance(USFL, USFR, IRFL, IRFR):
	"""
	Calculate minimum distance to obstacle ahead using front sensors.
	Returns minimum distance in cm, or -1 if no valid readings.
	"""
	min_distance = 9999
	if USFL >= 0:
		min_distance = min(min_distance, USFL)
	if USFR >= 0:
		min_distance = min(min_distance, USFR)
	if IRFL >= 0:
		min_distance = min(min_distance, IRFL)
	if IRFR >= 0:
		min_distance = min(min_distance, IRFR)

	return min_distance if min_distance != 9999 else -1

def calculate_speed_ahead(front_obstacle_distance):
	"""
	Determine a safe speed ahead based on front sensors.
	Returns speed percentage (0-99).
	Slower speed when approaching walls/obstacles.
	"""
	threshold_distance = 5  # cm, cutoff for minimum distance, below which speed is 0
	# finds which sensor gives the minimum distance reading


	if front_obstacle_distance < threshold_distance:
		return 0  # too close, stop
	elif front_obstacle_distance > 50:
		return 99  # clear ahead, full speed
	else:
		# Scale speed linearly between threshold and 50 cm
		return int((front_obstacle_distance - threshold_distance) / (50 - threshold_distance) * 99)
	
def go_tree(current_state):
	"""
	Implements logic to update robot mode from "go" state.
	"""
	# This function will be a series of checks to update robot_state["mode"] based on sensor and camera data. 
	# Its basically a decision tree to figure out what to do next, will need to play around to figure out priorities;
	# The hierarchy of decisions can be seen in the documentation
	wallDistance = current_state["front_obstacle_distance"] 
	if wallDistance >= 0 and wallDistance < 15:
		# Obstacle detected ahead, decide turn direction
		USL = current_state["sensors"]["USL"]
		IRL = current_state["sensors"]["IRL"]
		leftDistance = IRL if IRL >= 0 else USL
		USR = current_state["sensors"]["USR"]
		IRR = current_state["sensors"]["IRR"]
		rightDistance = IRR if IRR >= 0 else USR
		# Decide turn direction based on more open side
		if leftDistance >= 25 and leftDistance > rightDistance:
			return "turn_left"
		elif rightDistance >= 25 and rightDistance > leftDistance:
			return "turn_right"	
		else: # both sides blocked, reverse
			return "reverse"
	else:
		dice_visible = current_state["dice_visible"]
		if dice_visible:
			return "dice_track"
		else:
			return "go"

def turn_left_tree(current_state):
	"""
	Implements logic to update robot mode from "turn_left" state.
	"""
	# This function will be a series of checks to update robot_state["mode"] based on sensor and camera data. 
	# Its basically a decision tree to figure out what to do next, will need to play around to figure out priorities;
	# The hierarchy of decisions can be seen in the documentation
	if current_state["turn_start_time"] is None:
		with state_lock:
			robot_state["turn_start_time"] = time.time()
	
	else :
		turn_duration = time.time() - current_state["turn_start_time"] # if we have been turning for less than threshold time, keep turning
		if turn_duration > 3.0:
			with state_lock:
				robot_state["turn_start_time"] = None
			return "go"
		if turn_duration < 0.5: #TODO dial in time threshold
			return "turn_left"	
		else: # check front sensors to see if we are clear to go forward again
			USFL = current_state["sensors"]["USFL"]
			USFR = current_state["sensors"]["USFR"]
			IRFL = current_state["sensors"]["IRFL"]
			IRFR = current_state["sensors"]["IRFR"]
			frontDistance = calculate_obstacle_distance(USFL, USFR, IRFL, IRFR)

			gravelFlag = current_state["gravel_flag_detected"]
			rampFlag = current_state["ramp_flag_detected"]
		if frontDistance >= 20:  # TODO dial in distance threshold
			if gravelFlag: # once we see gravel, this flag will be set in the robot_state object by the pixy thread. We enter gravel mode if we've seen that flag, and the front is clear again. We reset the flag once we exit gravel mode
				with state_lock:
					robot_state["turn_start_time"] = None
				return "gravel"
			elif rampFlag:
				with state_lock:
					robot_state["turn_start_time"] = None
					robot_state["bridge_start_time"] = time.time()
				return "bridge" # bridge mode is gonna be a complicated one, we enter it coming out of this turn, but it will need to go forwards a bit, align with the bridge, and then cross it. This will require testing!!
			else: 
				with state_lock:
					robot_state["turn_start_time"] = None
				return "go"
		else:
			return "turn_left"

def turn_right_tree(current_state):
	"""
	Implements logic to update robot mode from "turn_right" state.
	"""
	# This function will be a series of checks to update robot_state["mode"] based on sensor and camera data. 
	# Its basically a decision tree to figure out what to do next, will need to play around to figure out priorities;
	# The hierarchy of decisions can be seen in the documentation
	if current_state["turn_start_time"] is None:
		with state_lock:
			robot_state["turn_start_time"] = time.time()
	
	else :
		turn_duration = time.time() - current_state["turn_start_time"] # if we have been turning for less than threshold time, keep turning
		if turn_duration > 3.0:
			with state_lock:
				robot_state["turn_start_time"] = None
			return "go"
		if turn_duration < 0.5: #TODO dial in time threshold
			return "turn_right"	
		else: # check front sensors to see if we are clear to go forward again
			USFL = current_state["sensors"]["USFL"]
			USFR = current_state["sensors"]["USFR"]
			IRFL = current_state["sensors"]["IRFL"]
			IRFR = current_state["sensors"]["IRFR"]
			frontDistance = calculate_obstacle_distance(USFL, USFR, IRFL, IRFR)

			gravelFlag = current_state["gravel_flag_detected"]
			rampFlag = current_state["ramp_flag_detected"]
		if frontDistance >= 20:  # TODO dial in distance threshold
			if gravelFlag: # once we see gravel, this flag will be set in the robot_state object by the pixy thread. We enter gravel mode if we've seen that flag, and the front is clear again. We reset the flag once we exit gravel mode
				with state_lock:
					robot_state["turn_start_time"] = None
				return "gravel"
			elif rampFlag:
				with state_lock:
					robot_state["turn_start_time"] = None
				return "bridge" # bridge mode is gonna be a complicated one, we enter it coming out of this turn, but it will need to go forwards a bit, align with the bridge, and then cross it. This will require testing!!
			else: 
				with state_lock:
					robot_state["turn_start_time"] = None
				return "go"
		else:
			return "turn_right"

def reverse_tree(current_state):
	if current_state["reverse_start_time"] is None:
		with state_lock:
			robot_state["reverse_start_time"] = time.time()
	else:
		reverse_duration = time.time() - current_state["reverse_start_time"]
		if reverse_duration > 1.0: # reverse for 1 seconds
			with state_lock:
				robot_state["reverse_start_time"] = None
			return "go"
		else:
			return "reverse"

def gravel_tree(current_state):
	"""
	Implements logic to update robot mode from "gravel" state.
	"""
	# This function will be a series of checks to update robot_state["mode"] based on sensor and camera data. 
	# Its basically a decision tree to figure out what to do next, will need to play around to figure out priorities;
	# The hierarchy of decisions can be seen in the documentation
	wallDistance = current_state["front_obstacle_distance"] 
	if wallDistance >= 0 and wallDistance < 15:
		# Obstacle detected ahead, decide turn direction
		return "turn_left"  # for now just always turn left out of gravel, can improve later
	else:
		return "gravel"

def bridge_tree(current_state):
	bridge_duration = time.time() - current_state["bridge_start_time"]


def main_loop():
	"""
	Main loop: decide robot mode based on sensor data in robot_state.
	"""
	# while True:
	# 	command_robot('L', 0, 99, 'D', 0)  # stop robot initially
	# 	time.sleep(1)  # wait a second before starting main loop
	while True:
		with state_lock:
			# Copy current state for decision making
			current_state = robot_state.copy()

		# Decision logic based on current_state
		# Example:
		mode = current_state["mode"]
		next_mode = mode  # default to current mode
		sensors = current_state["sensors"]
		pixy_blocks = current_state["pixy_blocks"]
		yellow_detected = current_state["yellow_detected"]
		print(f"Current mode: {mode}")
		front_obstacle_distance = calculate_obstacle_distance(
			sensors["USFL"], sensors["USFR"],sensors["IRFL"], sensors["IRFR"])
		current_state["front_obstacle_distance"] = front_obstacle_distance

		if mode == "go": # centers robot on track and moves forward, slows down if appraching obstacles
			angle_adjust = calculate_centering(
				sensors["USL"], sensors["USR"],
				sensors["IRL"], sensors["IRR"]
			)
			speed_ahead = calculate_speed_ahead(
				front_obstacle_distance
			)
			command_robot(
				'L' if angle_adjust >= 0 else 'R' if angle_adjust < 0 else 'B',
				abs(angle_adjust),
				speed_ahead,
				'D',
				5  # example brush speed
			)
			next_mode = go_tree(robot_state)
		
		elif mode == "turn_left": # implements turn left logic; starts turning and records time, exit conditions are when a certain time has passed and the left  and front sensors reads a far distance again (will need to calibrate time)
			#behavior in this mode is just turning in place, dice collection system can continue
			command_robot("L","XX",50,'D',5)
			next_mode = turn_left_tree(robot_state)

		elif mode == "turn_right":
			command_robot("R","XX",50,'D',5)
			next_mode = turn_right_tree(robot_state)

		elif mode == "reverse": # for now just moves backwards for a set time (1 second), then goes to go mode. Will need to add logic to determine when to exit reverse based on sensor readings
			command_robot("B","00",50,'D',0)
			next_mode = reverse_tree(robot_state)

		elif mode == "stop":  # just stops all motors, breaks the loop to end the program. We can have some trigger for this if we want, but otherwise its unused 
			command_robot('L', 'XX', 0, 'D', 0)
			next_mode = "stop"
			break
		elif mode == "gravel": # this mode is entered once we see the gravel flag after a turn, we need to move forward and continue alignment
			# similar to go but with tray raised. Can implement some protocol to get brush out of way, like briefly running brush in reverse, or just ball
			angle_adjust = calculate_centering(
				sensors["USL"], sensors["USR"],
				sensors["IRL"], sensors["IRR"]
			)
			speed_ahead = calculate_speed_ahead(
				front_obstacle_distance
			)
			command_robot(
				'L' if angle_adjust >= 0 else 'R' if angle_adjust < 0 else 'B',
				abs(angle_adjust),
				speed_ahead,
				'U',
			  	0)  # example brush speed
			next_mode = gravel_tree(robot_state)
			
		elif mode == "bridge": # bridge mode is gonna be a complicated one, we enter it coming out of this turn, but it will need to go forwards a bit, align with the bridge, and then cross it. This will require testing!!
			# implement with cameras, etc. will probably be histeresis based, where we have a series of steps to align with and cross the bridge
			# not sure how much of this can be done with the camera, we could align usung the two sensors on the front (go until both front sensors read equal distance, then cross bridge for set time)
			# TODO implement bridge logic
			next_mode = bridge_tree(robot_state)
			
		elif mode == "dice_track":
			# TODO this is a variation on Go where it picks whichever dice is most cenetered and aims for that.
			# this will need to use pixy blocks to determine dice positions. Will need some threshold to preven getting too close to walls, we can't get dice that're within ~3" of walls
			pass
		
		with state_lock:
			# Update robot_state if needed
			robot_state["mode"] = next_mode  # may need to update more parameters, but for now just sets mode for next loop iteration
			robot_state["front_obstacle_distance"] = front_obstacle_distance
		time.sleep(0.1)  # Adjust loop rate as needed