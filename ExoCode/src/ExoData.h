/**
 * @file ExoData.h
 *
 * @brief Declares a class used to store data for the Exo to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef ExoData_h
#define ExoData_h

#include "Arduino.h"

#include "LegData.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "StatusLed.h"
#include "StatusDefs.h"
/* 
 * ExoData was broken out from the Exo class as we were originally going to have it mirrored on a second microcontroller that handled BLE.
 * It doesn't need to be done this way if we aren't, and is pretty cumbersome.
 * Just thought you might be wondering about the weirdness.
 */

// moved status values to StatusDefs.h

// Type used for the for each joint method, the function should take JointData as input and return void
typedef void (*for_each_joint_function_t) (JointData*, float*); 


/**
 * @brief Class to store all the data related to the exo
 */
class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // constructor
        
        /**
         * @brief reconfigures the the exo data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        /**
         * @brief performs a function for each joint
         * 
         * @param pointer to the function that should be done for each used joint
         */
        template <typename F>
        void for_each_joint(F &&func)
        {
                func(&left_leg.hip, NULL);
                func(&left_leg.knee, NULL);
                func(&left_leg.ankle, NULL);
                func(&right_leg.hip, NULL);
                func(&right_leg.knee, NULL);
                func(&right_leg.ankle, NULL);
        }
        template <typename F>
        void for_each_joint(F &&func, float* args)
        {
                func(&left_leg.hip, args);
                func(&left_leg.knee, args);
                func(&left_leg.ankle, args);
                func(&right_leg.hip, args);
                func(&right_leg.knee, args);
                func(&right_leg.ankle, args);
        }

        // Returns a list of all of the joint IDs that are currently being used
        uint8_t get_used_joints(uint8_t* used_joints);

        /**
         * @brief Get the joint pointer for a joint id. 
         * 
         * @param id Joint id
         * @return JointData* Pointer to JointData class for joint with id
         */
        JointData* get_joint_with(uint8_t id);
        
        /**
         * @brief Prints all the exo data
         */
        void print();

        /**
         * @brief Set the status object
         * 
         * @param status_to_set status_defs::messages::status_t
         */
        void set_status(uint16_t status_to_set);
        /**
         * @brief Get the status object
         * 
         * @return uint16_t status_defs::messages::status_t
         */
        uint16_t get_status(void);

        /**
         * @brief Set the default controller parameters for the current controller. These are the first row in the controller csv file
         * 
         */
        void set_default_parameters();
        
        
        bool sync_led_state; /**< state of the sync led */
        bool estop;/**< state of the estop */
        float battery_value; /**<Could be Voltage or SOC, depending on the battery type*/
        LegData left_leg;/**< data for the left leg */
        LegData right_leg;/**< data for the right leg */

        uint32_t mark; /**< used for timing, currently only used by the nano */

        uint8_t* config; /**< pointer to the configuration array */
        uint8_t config_len; /**< len of the configuration array */

        int error_code; /**< current error code for the system */
        int error_joint_id;
        bool user_paused; /**< if the user has paused the system */

        private:
        uint16_t _status; /**< status of the system*/
};

#endif