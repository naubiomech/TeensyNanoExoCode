# Bang Bang Controller

## Description
Applies constant torque while extending, then a constant torque while flexing.
Based on the Extension Angle Controller.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- flexion_setpoint - Parameters for maximum exo flexion torque(Nm).
- extension_setpoint - Parameters for maximum exo extension torque(Nm).
- target_flexion_percent_max - Parameter for fraction of peak flexion angle where mode will switch to extension angle.
- clear_angle - Parameter for flag to reset the the peak range of motion angles.
- angle_threshold - Angle(rad) where the system will switch from extension assistance to flexion assistance. 
- velocity_threshold - Velocity(rad/s) where the system will switch from flexion assistance to extension assistance, if velocity inverts before the target_flexion_percent_max.  ***Value should be NEGATIVE ***
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls