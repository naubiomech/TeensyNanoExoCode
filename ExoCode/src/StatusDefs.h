/**
 * @file StatusDefs.h
 *
 * @brief Stores the different status messages for the system
 * 
 * @author P. Stegall 
 * @date Dec. 2021
*/


#ifndef StatusDefs_h
#define StatusDefs_h

#include "Arduino.h"
#include "Board.h"

/**
 * @brief stores the status messages
 */
namespace status_defs
{
    /**
     * @brief stores the status messages
     */
    namespace messages 
    {
        
        //0b error ,trial_on, trial_off
        const uint16_t off =  0; /**< off status id */
        const uint16_t trial_off = 1; /**< trial off status id */
        const uint16_t trial_on = 2; /**< trial on status id */
        const uint16_t test = 3; /**< test status id, generally won't be used.*/
        const uint16_t torque_calibration = 4; /**< torque calibration status id */
        const uint16_t fsr_calibration = 5; /**< fsr calibration status id */
        const uint16_t fsr_refinement =6; /**< fsr refinement status id */
        const uint16_t motor_start_up =7; /**< motor startup status id */
        // Specific error messages will use the 4 highest bits, giving 31 error messages
        const uint16_t error_bit = 4; /**< bit to indicate an error, gives 12 bits of error messages*/ 
        const uint16_t error = 1<<(error_bit-1); /**<general error message status id */
        
        const uint16_t error_left_heel_fsr =  1<<error_bit | error; /**< left heel fsr error id */
        const uint16_t error_left_toe =  2<<error_bit | error; /**< left toe fsr error id */
        const uint16_t error_right_heel_fsr =  3<<error_bit | error; /**< right heel fsr error id */
        const uint16_t error_right_toe_fsr =  4<<error_bit | error; /**< right toe fsr error id */
        
        const uint16_t error_left_hip_torque_sensor =  5<<error_bit | error; /**< left hip torque sensor error id */
        const uint16_t error_left_knee_torque_sensor =  6<<error_bit | error; /**< left knee torque sensor  error id */
        const uint16_t error_left_ankle_torque_sensor =  7<<error_bit | error; /**< left ankle torque sensor  error id */
        const uint16_t error_right_hip_torque_sensor =  8<<error_bit | error; /**< right hip torque sensor  error id */
        const uint16_t error_right_knee_torque_sensor =  9<<error_bit | error; /**< right knee torque sensor  error id */
        const uint16_t error_right_ankle_torque_sensor =  10<<error_bit | error; /**< right ankle torque sensor  error id */
        
        const uint16_t error_left_hip_motor =  11<<error_bit | error; /**< left hip motor error id */
        const uint16_t error_left_knee_motor =  12<<error_bit | error; /**< left knee motor error id */
        const uint16_t error_left_ankle_motor =  13<<error_bit | error; /**< left ankle motor error id */
        const uint16_t error_right_hip_motor =  14<<error_bit | error; /**< right hip motor error id */
        const uint16_t error_right_knee_motor =  15<<error_bit | error; /**< right knee motor error id */
        const uint16_t error_right_ankle_motor =  16<<error_bit | error; /**< right ankle motor error id */
        
        const uint16_t error_left_hip_controller =  17<<error_bit | error; /**< left hip controller error id */
        const uint16_t error_left_knee_controller =  18<<error_bit | error; /**< left knee controller error id */
        const uint16_t error_left_ankle_controller =  19<<error_bit | error; /**< left ankle controller error id */
        const uint16_t error_right_hip_controller =  20<<error_bit | error; /**< right hip controller error id */
        const uint16_t error_right_knee_controller =  21<<error_bit | error; /**< right knee controller error id */
        const uint16_t error_right_ankle_controller =  22<<error_bit | error;  /**< right ankle controller error id */
        
        const uint16_t error_to_be_used_1 =  23<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_2 =  24<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_3 =  25<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_4 =  26<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_5 =  27<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_6 =  28<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_7 =  29<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_8 =  30<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_9 =  31<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_10 =  32<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_11 =  33<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_12 =  34<<error_bit | error; /**< placeholder error id */
        const uint16_t error_to_be_used_13 =  35<<error_bit | error; /**< placeholder error id */
        
        const uint16_t warning_bit = error_bit + 6 + 1; /**< Bit to indicate warning, error bit plus 2^6-1 error messages, bit 11 for warning and 2^5-1 warning messages.*/
        const uint16_t warning = 1<<(warning_bit-1); /**< general warning id */
        const uint16_t warning_exo_run_time =  1<<warning_bit | warning;  /**< running time overflow warning id */
        
        const uint16_t warning_to_be_used_1 =  2<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_2 =  3<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_3 =  4<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_4 =  5<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_5 =  6<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_6 =  7<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_7 =  8<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_8 =  9<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_9 =  10<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_10 =  11<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_11 =  12<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_12 =  13<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_13 =  14<<warning_bit | warning; /**< placeholder warning id */
        const uint16_t warning_to_be_used_14 =  15<<warning_bit | warning;  /**< placeholder warning id */
    }
}

/**
 * @brief prints the status message
 *
 * @param message id
 */
void print_status_message(uint16_t message);

#endif
