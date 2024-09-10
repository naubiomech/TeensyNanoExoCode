# Calibration Manager Controller

## Description
This controller applies a small, constant torque while printing statements of relevance (command value/sign, torque sensor value/sign, angle sensor value/sign) to the Arduino Serial Monitor. 

## Parameters
There are no user defined parameters associated with this controller. 

## Purposes
After you have built an exo, one of the first things to check is the motor directions and the signs (positive/negative) of any of your external sensors (torque, angle, etc.). 
Having correct motor directions is important as it allows the exo controller designers to focus on the control itself, knowing that, if calibrated properly before use, the exo assistance will always be in the designed directions. 
The same principle applies for the sensors involved with control. Since PID control is implemented in many of our exo controllers, it is crucial to make sure that the directions match. 
This article will elaborate how to conduct the direction calibration using the CalibrManager controller. 
On each exo, this calibration process only needs to be done **once**, its results will need to be **manually** saved on the SD card and will be loaded **automatically** every time the exo starts up.

## Motor directions
When designing a controller, the values returned from the "calc_motor_cmd()" function determines the motor torque command (feedforward). To ease the controller design process, we propose that controller designers follow the following direction definitions for torque commands:
- At both ankle joints: Positive for Dorsiflexion, and negative for Plantarflexion
- At both knee joints: Positive for Extension, and negative for Flexion
- At both hip joints: Positve for Flexion, and negative for Extension
- At both shoulder joints: Positive for Flexion, and negative for Extension
- At both elbow joints: Positive for Flexion, and negative for Extension

In summary, this follows the Right-hand rule.

## Torque sensor signs (positive/negative)
Following the motor direction definitions, we need to verify that, when the motor supplies a positive torque command, the torque sensors, when zeroed properly, also returns a positive reading when the motor shaft is not spinning.

# How to calibrate
1. To calibrate, set Direction Calibration Manager as the default controller for the current exo. Connect the Teensy to a computer through USB, open up the Arduino Serial monitor. Connect the exo (Arduino Nano) with the python app, power on the exo and start a new session. The Direction Calibration Manager will send a positive torque command (3.5 Nm by default) to each motor.
2. Feel each torque through, for example, the foot plates on an ankle exo. If the torque applied on, for example, the left foot plate, is not causing it to rotate in the positive direction (see above for the specific directions), flip the motor direction for the left side. Flipping the motor direction is done by modifying the "config.ini" on the SD card.
3. After modifying all motor directions (if needed), start over again, and this time focus on the sensor readings (e.g., torque, angle) shown on the Arduino Serial Monitor. Feel the motor torque through, for example, a foot plate on an ankle exo, manually move the foot plate back to its neutral position, 0 degree for example. If the corresponding sensor does not return a postive value, flip the sensor direction by modifying the "config.ini" on the microSD card. Go through all torque sensors (if needed).
