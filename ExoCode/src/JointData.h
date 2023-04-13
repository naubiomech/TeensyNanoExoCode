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


// forward declaration
class ExoData;

/**
 * @brief class to store information related to joint.
 * 
 */
class JointData {
	public:
        JointData(config_defs::joint_id id, uint8_t* config_to_send);
        
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
        bool is_left; /**< if the leg is left */
        bool flip_direction; /**< if true invert the current to the motor and the position/velocity readings */
        bool is_used; /**< stores if the joint is used, joint is skipped if it is not used */
        bool calibrate_torque_sensor;  /**< flag for if we should calibrate the torque sensor. */ 
        
        float position; /**< The position of the joint, this should be motor position with compensated for gearing. */
        float velocity; /**< The velocity of the joint, this should be motor velocity with compensated for gearing. */


        float joint_position; /**< The position of the joint, after any transmission */
        float joint_global_angle; /**< The angle of the joint relative to the ground */
        float prev_joint_position; /**< The position of the joint, after any transmission */
        float joint_velocity;
        const float joint_position_alpha = 0.1f;
        const float joint_velocity_alpha = 0.1f;

};


#endif