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
                    if y < 100:   # top half
                        continue  # skip it

                    # valid block found in bottom half
                    return True

            # no valid dice detected in bottom half
            return False

        else:
            return False

    except Exception:
        return False


def lookForFlags():
    """
    looks for the signatures associated with the three flags on the course, checks if within distance threshold of them
    returns which of the three flags are visible at any time

    wrapped in function that determines which flag is present, updates booleans and updates relevant commands (raises tray, etc)

    returns
        success: bool - returns true if the camera successfully read, otherwise all arguments are false
        first_flag: bool -(before gravel)
        second_flag: bool -  (after gravel)
        third_flag: bool - (after bridge)
    """
    try:
        count = pixy.ccc_get_blocks(100, blocks)
        if count == 0:
            # no blocks detected
            return True, False, False, False

        # Map signatures to booleans
        seen = {sig: seeColor(sig, count) for sig in COLOR_MAP.values()}

        # Example logic 
        first_flag = seen.get(COLOR_MAP["orange"], False) and seen.get(COLOR_MAP["purple"], False)
        second_flag = seen.get(COLOR_MAP["orange"], False) and seen.get(COLOR_MAP["l_blue"], False)
        third_flag = seen.get(COLOR_MAP["purple"], False) and seen.get(COLOR_MAP["l_blue"], False)

        return True, first_flag, second_flag, third_flag
    except Exception as e:
        print("[PIXy ERROR] lookForFlags exception:", e)
        return False, False, False, False

def pixySetFlags():
    """
    When called, checks the current set of pixy blocks to determine if we have seen the colors relevent to the flags
    on the course

    If so, makes relevent variable adjustments:
    See first flag: raises collection tray, bool blocks any further commands to lower it, increases speeds in all navigation commands
    See second flag: removes gravel boolean and sets bridge one. This changes speeds and may initiate a bridge animation
    See third flag: reset first two flags

    returns 
        nothing

    """
    #print("Setting pixy flags:")

    global onGravel, onBridge, BASE_SPEED, TURN_SPEED, VEER_SPEED 
    success, first_flag, second_flag, third_flag = lookForFlags()
    if not success:
        return

    if first_flag and not onGravel:
        print("[PIXY FLAGS] Entering gravel mode")
        onGravel = True
        onBridge = False
 

    elif second_flag and not onBridge:
        print("[PIXY FLAGS] Entering bridge mode")
        onBridge = True
        onGravel = False


    elif third_flag: #SLOW motors to pass ramp
        if onGravel or onBridge:
            print("[PIXY FLAGS] Leaving special areas")
        onBridge = False
        onGravel = False


def mainLoop(): #logic for dice vs flag prioiritization
   
    while True: 
        pixySetFlags()  # Check and set flags first
        diceSeen = Pixicam()
        if diceSeen:
            print("Dice seen")

if __name__ == '__main__':
    mainLoop()
    time.sleep(0.02)
