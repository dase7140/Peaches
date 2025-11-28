from __future__ import print_function
import time
import serial

if __name__ == '__main__':
	arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=.05)
	time.sleep(1)
def checkTOFs():
    arduino.write(b"PSD\n")
    flags = arduino.readline().decode().strip()

    if flags:  # only parse if data is received
        #print("Received:", flags)
        try:
            parts = dict(p.split(":") for p in flags.split())
            flBlocked   = parts.get("FL") == "1"
            frBlocked   = parts.get("FR") == "1"
            leftBlocked = parts.get("L")  == "1"
            rightBlocked= parts.get("R")  == "1"
            backBlocked = parts.get("B")  == "1"

            # Example usage: robot logic
            print(f"FL:{flBlocked} FR:{frBlocked} L:{leftBlocked} R:{rightBlocked} B:{backBlocked}")

        except Exception as e:
            print("Parsing error:", e)
 
    else:
        print("No data received from Arduino")

    time.sleep(0.1)  # small delay for reliability

	
while True:
	checkTOFs()
