# Zhang Collins Controller

## Description
This controller applies a spline defined as a percent of gait.  This requires a heel FSR.
The profile is based on: 
Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. (2017). 
[Human-in-the-loop optimization of exoskeleton assistance during walking. ](https://www.science.org/doi/full/10.1126/science.aal5054)
Science, 356(6344), 1280-1284.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- torque - Magnitude of the peak torque in Nm
- peak_time - Time when the peak torque occurs (as a % of gait cycle)
- rise_time - Time from zero torque until peak torque (expressed as a % of gait cycle)
- fall_time - Time from peak torque until zero torque (expressed as a % of gait cycle)
- direction - Flag that flips torque from PF to DF (essentailly an assistance/resistance flag)
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls