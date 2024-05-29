/**
 * @file LegData.h
 *
 * @brief Declares a class used to store data for leg to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef LegData_h
#define LegData_h

#include "Arduino.h"

#include "JointData.h"
#include "ParseIni.h"
#include "Board.h"
#include "InclinationDetector.h"

#include <stdint.h>

// forward declaration
class ExoData;

/**
 * @brief class to store information related to the leg.
 * 
 */
class LegData {
	   
    public:
        LegData(bool is_left, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the leg data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        JointData hip; /**< data for the hip joint */
        JointData knee; /**< data for the knee joint */
        JointData ankle; /**< data for the ankle joint */
        
        
        float percent_gait;  /**< Estimate of the percent gait based on heel strike */
        float expected_step_duration;  /**< Estimate of how long the next step will take based on the most recent step times */

        float percent_stance;
        float expected_stance_duration;

        float percent_swing;
        float expected_swing_duration;
        
        float heel_fsr;  /**< Calibrated FSR reading for the heel */
        float heel_fsr_upper_threshold;
        float heel_fsr_lower_threshold;
        float toe_fsr;  /**< Calibrated FSR reading for the toe */
        float toe_fsr_upper_threshold;
        float toe_fsr_lower_threshold;

        
        bool ground_strike;  /**< Trigger when we go from swing to one FSR making contact. */
        bool toe_strike;
        bool toe_off; /**< Trigger when we go from one FSR making contact to swing. */
        bool toe_on;
        bool heel_stance;  /**< High when the heel FSR is in ground contact */
        bool toe_stance;  /**< High when the toe FSR is in ground contact */
        bool prev_heel_stance;  /**< High when the heel FSR was in ground contact on the previous iteration */
        bool prev_toe_stance;   /**< High when the toe FSR was in ground contact on the previous iteration */
        
        bool is_left; /**< 1 if the leg is on the left, 0 otherwise */
        bool is_used; /**< 1 if the leg is used, 0 otherwise */
        bool do_calibration_toe_fsr;  /**< flag for if the toe calibration should be done */
        bool do_calibration_refinement_toe_fsr;  /**< flag for if the toe calibration refinement should be done */
        bool do_calibration_heel_fsr;  /**< flag for if the heel calibration should be done */
        bool do_calibration_refinement_heel_fsr;  /**< flag for if the heel calibration refinement should be done */

        float ankle_angle_at_ground_strike;
        float expected_duration_window_upper_coeff; /**< factor to multiply by the expected duration to get the upper limit of the window to determine if a ground strike is considered a new step. */
        float expected_duration_window_lower_coeff; /**< factor to multiply by the expected duration to get the lower limit of the window to determine if a ground strike is considered a new step. */

        Inclination inclination;

        float PHJM_state;   /**< state for the PHJM controller, else should be set to 0 (here so it can be plotted) */

        //Elbow Variables
        float elbow_cmd;                      //motor command
        float Smoothed_Sig_Flex = 0;
        float Smoothed_Sig_Ext = 0;
        float FlexSense = 0;
        float ExtenseSense = 0;
        float Smoothed_Flex_Max = 0.2;
        float Smoothed_Flex_Min = 0.1;
        float Smoothed_Ext_Max = 0.2;
        float Smoothed_Ext_Min = 0.1;
        float starttime = 0;
        float check = 0;
        float t_0 = 0;
        float t_1 = 0.001;
        float V_0 = 0;
        float V_1 = 0;
        float Angle_Max = 0;
        float Angle_Min = 0;
        float Angle = 0;

        float setpoint = 0;
        float setpoint_filtered = 0;
        bool flexState = 0;
        bool extState = 0;
        bool nullState = 1;
        float flexcount = 0;
};

#endif