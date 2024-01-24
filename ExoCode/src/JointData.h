/**
 * @file JointData.h
 *
 * @brief Declares a class used to store data for joint to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef JointData_h
#define JointData_h


#include "Arduino.h"

#include "MotorData.h"
#include "ControllerData.h"
#include "ParseIni.h"
#include "Board.h"


#include <stdint.h>
#include <queue>


// forward declaration
class ExoData;

/**
 * @brief class to store information related to joint.
 * 
 */
class JointData {
	public:
        JointData(config_defs::joint_id id, uint8_t* config_to_send, float joint_RoM, bool flip_ankle_angle, float torque_offset, float motor_pos_safety_factor, float motor_RoM);
        
        /**
         * @brief reconfigures the the joint data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id; /**< id of the joint */
        MotorData motor; /**< data for the motor attached to the joint */
        ControllerData controller; /**< data for the controller attached to the joint */
        float torque_reading;  /**< calibrated reading from the torque sensor */ 
		float torque_reading_microSD;
        bool is_left; /**< if the leg is left */
        bool flip_direction; /**< if true invert the current to the motor and the position/velocity readings */
        bool is_used; /**< stores if the joint is used, joint is skipped if it is not used */
        bool calibrate_torque_sensor;  /**< flag for if we should calibrate the torque sensor. */ 
        bool calibrate_angle_sensor;  /**< flag for if we should calibrate the angle sensor. */
        
        float position; /**< The position of the joint, this should be motor position with compensated for gearing. */
        float velocity; /**< The velocity of the joint, this should be motor velocity with compensated for gearing. */

        float joint_position; /**< The position of the joint, after any transmission */
        float joint_global_angle; /**< The angle of the joint relative to the ground */
        float prev_joint_position; /**< The position of the joint, after any transmission */
        float joint_velocity;
        const float joint_position_alpha = 0.05f;
        const float joint_velocity_alpha = 0.05f;
		const float joint_RoM;
		bool do_flip_angle;
		const float torque_offset;
		float torque_offset_reading;
		const float motor_pos_safety_factor;
		const float motor_RoM;

        /* Error handling data */
        // Torque tracking check
        float torque_output_alpha = 0.2; /**< low pass to describe the lag of the low level controller relative to setpoint. */
        float post_transmission_torque = 0; /**< torque after torque_output_alpha. */

        // Torque absolute check
        float torque_output_threshold = 55; /**< maximum value of the the filtered torque. */

        // Torque variance check
        const int torque_failure_count_max = 2; /**< ammount of samples outside of error bounds. */
        const int torque_std_dev_multiple = 10; /**< number of standard deviations from the mean to be considered an error. */
        const float torque_data_window_max_size = 100; /**< number of samples to use for the standard deviation calculation. */
        std::queue<float> torque_data_window; /**< queue to store torque sensor values. */
        int torque_failure_count = 0; /**< number of successive samples outside of error bounds. */

        // Transmission efficiency check
        const float transmission_efficiency_threshold = 0.001;
        const float motor_torque_smoothing = 0.1;
        const float torque_error_smoothing = 0.02;
        float smoothed_motor_torque = 0;
        const float close_to_zero_tolerance = 0.1;
        float torque_error = 0;
		float torque_error_max;
		bool do_calc_motor_pos_range = true;
		
		float motor_diff_1 = 0;
		float motor_pos_1 = 0;
		float motor_pos_2 = 0;
		float motor_diff_2 = 0;
		float motor_ref_pos = 0;
		
		bool do_calc_motor_pos_offset = true;
		float motor_pos_0 = 0;
		float motor_pos_offset = 0;
		
		//Calibration Manager Controller
		float motor_pos_max = 0;
		float motor_pos_min = 0;
		bool motor_pos_first_run = true;
};


#endif