from __future__ import print_function
import pixy
from ctypes import *
from pixy import *
import time
import serial

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)
	
while True:
	arduino.write(b"PSD\n")
	flags = arduino.readline().decode().strip()

	print(flags)
# Example output: "FL:0 FR:1 L:0 R:0 B:0"

	parts = dict(p.split(":") for p in flags.split())

	flBlocked  = parts["FL"] == "1"
	frBlocked = parts["FR"] == "1"
	leftBlocked = parts["L"]  == "1"
	rightBlocked = parts["R"]  == "1"
	backBlocked = parts["B"]  == "1"
    

time.sleep(5)


#left at checking what pins are what sensors
