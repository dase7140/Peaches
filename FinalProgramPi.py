# main loop should:
	# if yellow is seen, navigate state
	# if rampseen, ramp warn state
	# if gravel flag seen, state = gravel warn
	# if dice seen
	# state = dice track
	# if state = designated state
		# do state function 
		# ex. if state = navigate
			# basic TOF obstacle avoidance
from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import time
import serial
from picamera2 import Picamera2
import cv2
import numpy as np

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)


# Initialize PiCamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([140, 255, 255])


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

#Function for TOFs
def checkTOFs():
	arduino.reset_input_buffer()
	time.sleep(2)
	arduino.write(b"PSD\n")
	flags = arduino.readline().decode().strip()

	if flags:  # only parse if data is received
        #print("Received:", flags)
		try:
			parts = dict(p.split(":") for p in flags.split())
            # convert to booleans
			sensor_flags = {
				"FL": parts.get("FL") == "1",
				"FR": parts.get("FR") == "1",
				"L":  parts.get("L")  == "1",
				"R":  parts.get("R")  == "1",
				"B":  parts.get("B")  == "1"
			}
			print(sensor_flags)
			return sensor_flags
		except Exception as e:
			print("Parsing error:", e)
			return None
 
	else:
		print("No data received from Arduino")
		return None

	time.sleep(0.1)  # small delay for reliability
    
#Function for Picam
def is_color_visible(frame):
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

#navigation state = check TOFs, write motor functions for each case it could see.
def navigate_with_tofs():
	flags = checkTOFs()
	if not flags:
		print("No sensor data")
		return
		
	fl = flags["FL"]
	fr = flags["FR"]
	left = flags["L"]
	right = flags["R"]
	back = flags["B"]
    
	if fl or fr:
		if left and not right:
			arduino.write(b"MR0\n")
			print("Turning Right, obstacle at front")
			return
		if right and not left:
			arduino.write(b"ML0\n")
			print("Turning Left, obstacle at front")
			return
		if left and right:
			print("Reversing, obstacle at front")
			arduino.write(b"MB0\n")
			return
		else:
			print("Turning right")
			arduino.write(b"MR0\n")
			return

    # 2. LEFT BLOCKED → steer right
	if left and not right:
		print("Left blocked → turning right")
		arduino.write(b"MR0\n")
		return

    # 3. RIGHT BLOCKED → steer left
	if right and not left:
		print("Right blocked → turning left")
		arduino.write(b"ML0\n")
		return

    # 4. BOTH LEFT & RIGHT BLOCKED 
	if left and right:
		print("move straight")
		arduino.write(b"MF0\n")
		return

    # 5. BACK BLOCKED → avoid reverse, move forward
	if back:
		print("Back blocked → moving forward")
		arduino.write(b"MF0\n")     # move forward
		return

    # 6. ALL CLEAR → move forward
	#print("Path clear → moving forward")
	#arduino.write(b"MF0\n")

def dice_track():
	count = pixy.ccc_get_blocks(100, blocks)
	now = int(round(time.time() * 1000)) #now is for incase we need a search timeout
 
	if count > 0: # if there are blocks being formed, might remove this??
		targetSeen = seeColor(targetSignature, count)
		targetX = getTargetX(targetSignature, count)

		if targetSeen and targetX != -1: #Is die visible?
			error = targetX - centerX
			lostTargetTimer = now
			if abs(error) <= deadband:
				print("Move Forward, Dice")
				arduino.write(b"MFD\n") #Move forward, dice
				lastTargetDirection = 0 #debugging
			elif error < 0:
				leftBlocked = checkTOFs() #Check if side TOFs are flagging
				if leftBlocked != 1: 
					print("Turn Left")
					arduino.write(b"ML0\n")
					lastTargetDirection = -1
				else:
					arduino.write(b"MF0\n") #move forward
			else:
				rightBlocked = checkTOFs() #Check if TOFs are flagging
				if rightBlocked != 1: 
					print("Turn Right")
					arduino.write(b"MR0\n")
					lastTargetDirection = 1
				else:
					#move forward
					arduino.write(b"MF0\n")	
      
		else:
			print("No target found")
			arduino.write(b"TNF\n")
            
 
	else:
		print(f"Count =  {count}")
		arduino.write(b"TNF\n")
         
def gravel_warn():
	count = pixy.ccc_get_blocks(100, blocks)
	now = int(round(time.time() * 1000)) #now is for incase we need a search timeout
 
	if count > 0: 
		gravelSignature = seeColor(gravelSignature, count)
		gravelX = getTargetX(gravelSignature, count)

		if gravelSignature and gravelX != -1: 
			arduino.write(b"G00\n")
         
#Main loop  
while True:   
#Setting variables that capture sensors & cams
	#Picam
	frame = picam2.capture_array()
	visible, center, at_top = is_color_visible(frame)
	#Pixy
	count = pixy.ccc_get_blocks(100, blocks)
	
	if visible:
		if at_top:
			print(" → Tape at TOP (correct direction)")
			#only if visible does the navigation function run
			#"State Model Below" lines would continue here
		else:
			print(" → Tape NOT at top (wrong direction!)")
	
#State Model Below:
	#Variables that check the sensor data:
	targetSeen = seeColor(targetSignature, count)
	targetX = getTargetX(targetSignature, count)
	gravelSeen = seeColor(gravelSignature, count)
	gravelX = getTargetX(gravelSignature, count)
	rampSeen = seeColor(rampSignature, count)
	rampX = getTargetX(rampSignature, count)
	
	state = "navigate"
	#if state == "navigate":
		#print(state)
		#navigate_with_tofs()
	if gravelSeen:
		state = "dice_track"
		print(state)
		#gravel_warn()
	if rampSeen:
		state = "ramp_warn"
		print(state)
		#ramp_warn()
	if targetSeen:
		state = "dice_track"
		print(state)
		#dice_track()
	navigate_with_tofs()		
		
	

