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

    # ---- Second Flag: Enter Bridge Mode ----
    elif second_flag_count >= DEBOUNCE_FRAMES and not onBridge:
        print("[PIXY FLAGS] Entering bridge mode (debounced)")
        onBridge = True
        onGravel = False

    # ---- Third Flag: Exit special modes ----
    elif third_flag_count >= DEBOUNCE_FRAMES:
        if onGravel or onBridge:
            print("[PIXY FLAGS] Leaving special areas (debounced)")
        onGravel = False
        onBridge = False


def mainLoop(): #logic for dice vs flag prioiritization
   
    while True: 
        pixySetFlags()  # Check and set flags first
        diceSeen = Pixicam()
        if diceSeen:
            print("Dice seen")

if __name__ == '__main__':
    mainLoop()
    time.sleep(0.02)

