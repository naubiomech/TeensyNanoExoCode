# User Defined Controller

## Description
This controller uses a simple lookup table with interpolation between values.
Values are normalized to user mass.
Assumes table is evenly spaced across percent gait, 0 to 100 where f(0) = f(100)

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- mass - User mass used to denormalize values
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls
- curve -  A sequence of evenly spaced points that define the curve