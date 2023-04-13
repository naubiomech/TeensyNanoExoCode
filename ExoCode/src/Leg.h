/**
 * @file Leg.h
 *
 * @brief Declares a class used to operate a leg 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Leg_h
#define Leg_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "joint.h"
#include "controller.h"
#include "FSR.h"
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"
#include "StatusDefs.h"
#include "ThIMU.h"
#include "InclinationDetector.h"

#include <stdint.h>
#include <algorithm>

/**
 * @brief class to operate a leg.
 * 
 */
class Leg
{
    public:
		//Leg();
        Leg(bool is_left, ExoData* exo_data); // constructor: 
        
        /**
         * @brief read FSR,  calc percent gait, read joint data, send joint commands
         */
        void run_leg(); 
		
        /**
         * @brief Checks if calibration flags are set, and runs calibration if they are.
         */
        void check_calibration();  
		
        /**
         * @brief Reads motor data from each motor used in the leg and stores the values
         * Reads the FSR, detects ground strike, and calculates percent gait.
         * Sets the values to the corresponding place in data class.
         */
        void read_data(); 
		
        /**
         * @brief sends new control command to the motors used in the leg, based on the defined controllers
         */
        void update_motor_cmds();   
		
        /**
         * @brief Changes the controller for an individual joint
         */
        void set_controller(int joint, int controller);  
		
        /**
         * @brief Simply clears the step time estimate for when it gets off by more than can be adjusted for.
         */
        void clear_step_time_estimate();

        /**
         * @brief Disables all motors in the leg
         * 
         */
        void disable_motors();

        float get_Kt_for_joint(uint8_t id);
        
	private:
        /**
         * @brief Applies the FSR thresholds set in the data class to the FSRs
         * 
         */
        void _check_thresholds();
		
        /**
         * @brief Calculates the percent of gait based on the ground contact reading
         * and an estimate of the step time based on the average time of the last few steps.
         * returns the percent gait which saturates at 100%
         * 
         * @return percent gait from heel strike
         */
        float _calc_percent_gait();
        
        /**
         * @brief Calculates the expected duration of a step by averaging the time the last N steps took.
         * Should only be called when a ground strike has occurred.
         *
         * @return expected stance duration in ms 
         */
        float _update_expected_duration();
        
        /**
         * @brief Checks for state changes in the FSRs to find the point when ground contact is made
         * Simple check for a rising edge of either FSR during swing and returns 1 if they have.
         * The returned value should just be high for a single cycle.
         *
         * @return 1 if the foot has gone from non-contact to ground contact. 0 Otherwise
         */
        bool _check_ground_strike();

        /**
         * @brief MUST BE CALLED AFTER _check_ground_strike()! Checks for state changes in the FSRs to find 
         * the point when ground contact is lost. Simple check for a falling edge of either FSR during stance 
         * and returns 1 if they have. The returned value should just be high for a single cycle. 
         *
         * @return 1 if the foot has gone from ground contact to non-contact. 0 Otherwise
         */
        bool _check_toe_off();
		
        // data that can be accessed
        ExoData* _data;/**< Pointer to the overall exo data */
        LegData* _leg_data; /**< Pointer to the specific leg we are using.*/
        
        // joint objects for the leg.
        // The order these are listed are important as it will determine the order their constructors are called in the initializer list.
        HipJoint _hip; /**< instance of a hip joint */
        KneeJoint _knee; /**< instance of a knee joint */
        AnkleJoint _ankle; /**< instance of a ankle joint */
        
        FSR _heel_fsr; /**< heel force sensitive resistor */

		FSR _toe_fsr; /**< toe force sensitive resistor */

        InclinationDetector* inclination_detector;

        
        bool _is_left; /**< stores which side the leg is on */
        
        bool _prev_heel_contact_state; /**< Prev heel contact state used for ground strike detection */
        bool _prev_toe_contact_state; /**< Prev toe contact state used for ground strike detection */

        bool _prev_toe_contact_state_toe_off; /**< Prev toe off state used for toe off detection */
        
        static const uint8_t _num_steps_avg = 3;  /**< the number of prior steps used to estimate the expected duration, used for percent gait calculation */
        unsigned int _step_times[_num_steps_avg];  /**< stores the duration of the last N steps, used for percent gait calculation */// 
        
        unsigned int _ground_strike_timestamp;   /**< Records the time of the ground strike to determine if the next strike is within the expected window. */ 
        unsigned int _prev_ground_strike_timestamp;   /**< Stores the last value to determine the difference in strike times. */ 
        unsigned int _expected_step_duration;   /**< The expected step duration to calculate the percent gait.*/
        
};
#endif
#endif