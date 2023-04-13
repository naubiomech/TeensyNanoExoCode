# Zero Torque Controller

## Description
This controller attempts to control the joint to zero torque.  
This can be done with or without closed loop control.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls