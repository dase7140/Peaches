from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import time
import serial

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)
	

#1 = red, 2 = green 3 = blue 4= yellow 5 = purple (GRAVEL)  6 = pink (RAMP)

# for tracking
centerX = 157
deadband = 30
targetSignature = 3 
gravel_sig = 5
ramp_sig = 6
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

# Main loop
while True:
    count = pixy.ccc_get_blocks(100, blocks)
    #now is for incase we need a search timeout
    now = int(round(time.time() * 1000))

    if count > 0:
        targetSeen = seeColor(targetSignature, count)
        targetX = getTargetX(targetSignature, count)
        gravelSeen = seeColor(targetSignature, count)
        gravelX = getTargetX(targetSignature, count)
        rampSeen = seeColor(targetSignature, count)
        rampX = getTargetX(targetSignature, count)
        
		 if gravelSeen and gravelX != -1:
            error = targetX - centerX
            lostTargetTimer = now
            #pick up tamp
        if rampSeen and rampX != -1:
            error = targetX - centerX
            lostTargetTimer = now     
			#if ramp seen, turn then move straight
            
        if targetSeen and targetX != -1:
            error = targetX - centerX
            lostTargetTimer = now

            if abs(error) <= deadband:
                print("Move Forward")
                arduino.write(b"MF0\n")
                lastTargetDirection = 0 #debugging
            elif error < 0:
                print("Turn Left")
                arduino.write(b"ML0\n")
                lastTargetDirection = -1
            else:
                print("Turn Right")
                arduino.write(b"MR0\n")
                lastTargetDirection = 1
        else:
            print("No target found")
            arduino.write(b"TNF\n")


    else:
        print(f"Count =  {count}")
        arduino.write(b"TNF\n")
#one for move backwards

time.sleep(0.05)
