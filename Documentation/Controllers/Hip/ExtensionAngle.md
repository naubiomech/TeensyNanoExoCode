# Extension Angle Controller

## Description
Applies torque based on the angle of the hip while extending, then a constant torque while flexing.
We found the torque for extension was not as noticeable due to the scaling, so created the Bang Bang controller based on this one.
Neither have been formally evaluated yet so it is not known if the torque not being obvious is less beneficial.
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