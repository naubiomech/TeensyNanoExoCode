# Proportional Joint Moment Controller

## Description
This controller applies a torque to the ankle proportional to the normalized FSR reading of the toe.
More details on this controller can be found in:

G\. M\. Gasparri, J. Luque and Z. F. Lerner, 
"[Proportional Joint-Moment Control for Instantaneously Adaptive Ankle Exoskeleton Assistance](https://ieeexplore.ieee.org/abstract/document/8669971)," 
in IEEE Transactions on Neural Systems and Rehabilitation Engineering, vol. 27, no. 4, pp. 751-759, April 2019, doi: 10.1109/TNSRE.2019.2905979.

## Parameters
Parameter index order can be found in [ControllerData.h](/ExoCode/src/ControllerData.h).
- max_torque - The peak torque in Nm.  When the normalized FSR torque is 1 this will be the torque at the ankle.
- is_assitance - When this is 1(assistive) the system will apply the torque in the plantar flexion direction, when 0(resistive) will be in the dorsiflexion direction.
- use_pid - This flag turns PID on(1) or off(0)
- p_gain - Proportional gain for closed loop controls
- i_gain - Integral gain for closed loop controls
- d_gain - Derivative gain for closed loop controls