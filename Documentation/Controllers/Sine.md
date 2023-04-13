# Sine Controller

## Description
This controller plays a sine wave with a defined amplitude, period, and phase shift

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- amplitude - amplitude in Nm
- period - period in ms
- phase shift - phase shift in rad
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls