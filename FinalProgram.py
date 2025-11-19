
# note: mass comment = alt+shift+c
# note: mass uncomment = alt+shift+u
# ////////Psuedocode for overall final program//////////
# Read TOFs, PixyCam and PiCam
# Is die visible? (PixyCam)
	# If yes, Check if side sensors are not flagging using a range thats half the size of the tank (we could do this another way, my worry is we'll hit the wall when centering)
		# Lower servo motor
		# Turn on spinner motor
		# Turn until die centered, drive forward toward die
		# Wait a bit to pick it up, then raise servo motor
		# back to sensor read	
# If no, Is yellow line visible? (PiCam)
	# Obstacle ahead? (TOFs)
		# If yes, Turn left or right
		# back to sensor read
	# If no, Drive forward
		# back to sensor read
# If yelllow not visible, Turn in place until visible again
	# back to sensor read

# special case: corner detection
	# if front tof < corner threshold and left/right tofs differ a lot, turn towards direction of larger with largest TOF distance

from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import time
import serial

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)
	

# for tracking
centerX = 157
deadband = 30
targetSignature = 3 #1 = red, 2 = greeb 3 = blue
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
#def yellowseen() for PiCam**
def checkTOFs():
	arduino.write(b"PSD\n") #PTS = Pull Sensor Data , arduino processes flags
	#read from arduino
	#get which are being flagged with 1s % 0s
	#freeToTurn checks side sensors
	
	
while True:
	#check front TOFs first
	
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
            
            #Is yellow line visible?******
            
            #If not, 
            
            
    else:
        print(f"Count =  {count}")
        arduino.write(b"TNF\n")
