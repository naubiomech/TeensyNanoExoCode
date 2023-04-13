# Heel Toe Controller

## Description
NOT CURRENTLY IMPLEMENTED
This controller is designed to estimate the hip torque based on heel and toe FSR measurements and apply that torque to the hip joint.
Based on : 

S\. S\. Pour Aji Bishe, L. Liebelt, Y. Fang and Z. F. Lerner, 
"[A Low-Profile Hip Exoskeleton for Pathological Gait Assistance: Design and Pilot Testing](https://ieeexplore.ieee.org/abstract/document/9812300)," 
2022 International Conference on Robotics and Automation (ICRA), 2022, pp. 5461-5466, doi: 10.1109/ICRA46639.2022.9812300.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- max_torque - Maximum torque to apply, i.e. hip flexion torque
- min_torque - Minimum torque to apply, i.e. hip extension torque
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls