
"sample_" files are working scripts used to test the main functions of the robot, like modular functions that can be tweaked and implements to the "Final-" .py and .ino scripts. 

Update: (Friday before break)
  I've been working off mostly the FinalProgram- files. I believe all the functions are implemented into the Pi Final Program. These functions come from modified versions of the sample scripts. 
  To run the Final Program:
    First upload the FinalProgramArd, make sure the serial monitor is not on, then upload the FinalProgramPi with the airplane envelope icon.
    Notes on errors: if there is an indentation error, I've had to just bckspace and tab each line from the error until it goes away.
    
  As of now, the Final Program reads the sensors and moves to avoid obstacles. This is what I'm defining as a navigation state. The plan is to prepare each state and then place logic in a main loop that runs them and changes state based on sensor and camera reading priorities. I left off with the left sensor (viewed from the back) not reading. Natalie and i believe this is due to power not being connected but we can't test cause batter is out of juice. 
  More on the navigation state: the function checkTOFs and navigate_with_tofs pull from each US sensor and make an array with "sensor name:" 0 or 1. If 1 that means the sensor is blocked. It determines blocked based on a value set on the Arduino Final Program. Then the functions pull which are blocked to run through some navigation logic.

  Also, the pixy is trained on the ramp and gravel flags. If you see on the pi main loop you'll see that there are new target signatures that the pi is checking. It runs with the priority of gravel->ramp-> dice. if it sees gravel: it should pick up the tray(the tray motor definition isnt set up on the arduino yet), if it sees a ramp: it should orient? (this is where I haven't really decided what to do), the dice is the same as what I showed yall awhile ago. It centers towards the dice. My concern with this is that itll run into a wall trying to center. So we should add checkTOFs() at certain points before it orients to ensure it CAN in the first place. 

Things left to do:    
  Set the behavior for what to do when gravel and ramp flags appear,
  Test navigation,
    Test the gravel and ramp flags,
  Pick up the pan,
  Color sorter
  
if you can't find the files on the pi for the python scripts, make a new sketch but make sure its in the python mechatronics directory next to a .i, pixy.cpy and pixy.py file (only for python code that uses pixy)

Lowkey I might just come in on Tuesday cause yea
