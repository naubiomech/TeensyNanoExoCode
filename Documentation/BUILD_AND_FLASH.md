## Downloads
1. Download Arduino IDE 1.8.19, which can be found [here](https://www.arduino.cc/en/software)
2. Now install Teensyduino, which can be found [here](https://www.pjrc.com/teensy/td_download.html). Be sure to install the 1.8.X version for your system.
3. Install the Arduino Nano 33 BLE through the Arduino Boards Manager.  
3. Install the Arduino libraries according to [Libraries](#libraries).

## Flashing
Now that you have all of the tools installed, lets flash.
1. Connect your computer to the Teensy using a micro-USB cable. 
2. Change the Board to the Teensy 4.1
3. Change the Arduino port to use the Teensyduino port. The correct port will have
the board name to the right of the COM (Windows Only) port. Like this:
![Diagram](/Documentation/Figures/ConnectedArduino.png)
4. Make sure that the Board Version is properly set in the Config.h
5. Press the Arrow button on the Arduino IDE to upload the code to the Teensy. 
6. Change the Board to the Arduino Nano 33 BLE.
7. Change the Arduino port in the same manner as step 3. 
8. Press the upload button. Note: The Nano will take much longer to build than 
the Teensy. 

## Libraries
All of the files in the libraries directory of this codebase should be copied to
C:\Users\\\[USER\]\Documents\Arduino\libraries\ or system equivalent. This will
'install' the libraries on your system. 
Details on the libraries can be found in the [Libraries Folder](/Libraries).

## Arduino Boards Manager
To open the boards manager press:
Tools -> Board -> Boards Manager

