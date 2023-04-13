# Adding a new microcontroller
This file assumes you are still using Arduino.  
If you are moving away from Arduino you will need to remove Arduino specific items, like logger::prints, pin defs that use A#.
Most of the code should run with little modification but there will be some reformatting to do, but if you are moving the code away from Arduino I am assuming you know enough to do this without my direction.

## Selecting microcontroller
    You should confirm that the microcontroller has the required elements.
    As of 20220517 this included:
        - Controller
            - SPI
            - CAN
            - 8 Analog Inputs
            - 9 Digital output
                - Ideally with 3 PWM for LED
            - 2 digital inputs
	    - Floating Point Unit
        - Coms
            - I2C
            - SPI
            - Bluetooth Low Energy
	    - IMU
            - Floating Point Unit
    It is also good to check that there are existing arduino libraries for the modules, ideally with identical interfaces.
        - SPI peripheral (slave) can be particularly challenging as most SPI libraries assume the microcontroller is the controller (master)

## Design board
    - When designing the board start from the current board as a template
    - In the schematic put components near where they are needed on the board. Particularly for capacitors and resistors where proximity matters
    - For the board file make sure those capacitors and resistors are where they belong
    - For the CAN bus place a restrict above the traces to prevent capacitive coupling on the lines.

## Board.h
    1. Create a new #define for your board where you define the name and a number for it.
    2. Create a new #elif for the new board, defining the pins for your board.
        - Add a #if defined([Microcontroller]), this will make sure the you can use pin names like A19 without throwing a compiling error when compiling for another board.
    3. Assign the pins based on the board design
        - Use the existing boards as a template.
    
## config.ini
    1. Add your board and version to the file
    
## ParseIni.h
    1. Add the board_name and board_version to the config_defs namespace
    2. for the controller board make sure that the std::map is available
    3. for the config_map namespace add the mapping for your board and version

## Other files
    Here is where it gets tricker
    1. Go through each of the files and add preprocessor defines around the areas that should compile for your microcontroller
        - for the controller micro you can add your micro to the #if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
        - for the coms micro you can add your micro to the #if defined(ARDUINO_ARDUINO_NANO33BLE)
        - There may be some locations were you need to add specific includes, ex. spi in ExoCode.ino
        - To find the name of your microcontroller to use in the defined([board])
            - go to the arduino directory/hardware/[type]/boards.txt
            - find your micro and look for *.board.name=[name]
            - in your define you will prepend this name with ARDUINO_
                - For example Teensy 4.1 is TEENSY41 so the define uses ARDUINO_TEENSY41
                - Be careful the Nano 33 is listed as ARDUINO_NANO33BLE so the define uses ARDUINO_ARDUINO_NANO33BLE
            

    