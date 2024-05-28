/**
 * @file ControllerData.h
 *
 * @brief Declares a class used to store data for controllers to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Controllerdata_h
#define ControllerData_h
#include <stdint.h>

#include "Arduino.h"

#include "Board.h"
#include "ParseIni.h"
#include <stdint.h>

// forward declaration
class ExoData;


namespace controller_defs /**< stores the parameter indexes for different controllers */
{
    namespace zero_torque
    {
        const uint8_t use_pid_idx = 0;
        const uint8_t p_gain_idx = 1;
        const uint8_t i_gain_idx = 2;
        const uint8_t d_gain_idx = 3;
        const uint8_t num_parameter = 4;
    }
    
    namespace stasis
    {
        const uint8_t num_parameter = 0;
    }
    
    namespace proportional_joint_moment
    {
        const uint8_t stance_max_idx = 0;  // parameter for peak exo torque during stance
        const uint8_t swing_max_idx = 1;  // parameter for peak exo torque during swing
        const uint8_t is_assitance_idx = 2;
        const uint8_t use_pid_idx = 3;
        const uint8_t p_gain_idx = 4;
        const uint8_t i_gain_idx = 5;
        const uint8_t d_gain_idx = 6;
        const uint8_t torque_alpha_idx = 7;
        const uint8_t num_parameter = 8;
    }
    
    namespace heel_toe
    {
        const uint8_t flexion_torque_setpoint_idx = 0;
        const uint8_t extension_torque_setpoint_idx = 1;
        const uint8_t use_pid_idx = 2;
        const uint8_t p_gain_idx = 3;
        const uint8_t i_gain_idx = 4;
        const uint8_t d_gain_idx = 5;
        const uint8_t num_parameter = 6;
    }

    namespace zhang_collins
    {

        const uint8_t torque_idx = 0;                           //Magnitude of Peak Torque in Nm
        const uint8_t peak_time_idx = 1;                        //Time when peak torque occurs (% of gait cycle)
        const uint8_t rise_time_idx = 2;                        //Time from zero torque until peak torque (Expressed as a % of gait cycle)
        const uint8_t fall_time_idx = 3;                        //Time from peak torque until zero torque (Expressed as a % of gait cycle)
        const uint8_t direction_idx = 4;                        //Flag to flip torque from PF to DF
        const uint8_t use_pid_idx = 5;                          //Flag to use PID control
        const uint8_t p_gain_idx = 6;                           //Value of P Gain for PID control
        const uint8_t i_gain_idx = 7;                           //Value of I Gain for PID control
        const uint8_t d_gain_idx = 8;                           //Value of D Gain for PID control 
        const uint8_t num_parameter = 9;
    }

    namespace franks_collins_hip
    {
        const uint8_t mass_idx = 0;                             //Mass of the User in kg
        const uint8_t trough_normalized_torque_Nm_kg_idx = 1;   //Extension Torque in Nm/kg
        const uint8_t peak_normalized_torque_Nm_kg_idx = 2;     //Flexion Torque in Nm/kg
        const uint8_t start_percent_gait_idx = 3;               //Percent of Gait Cycle where the curve starts (does not start at 0 so that there is no discontinuity)
        const uint8_t trough_onset_percent_gait_idx = 4;        //Percent of Gait Cycle where curve for extension torque starts
        const uint8_t trough_percent_gait_idx = 5;              //Percent of Gait Cycle where peak extension torque occurs
        const uint8_t mid_time_idx = 6;                         //Time from when curve starts until midpoint of zero torque period between extension and flexion torques
        const uint8_t mid_duration_idx = 7;                     //Time of zero torque period between extension and flexion torques
        const uint8_t peak_percent_gait_idx = 8;                //Percent of Gait Cycle where peak flexion torque occurs
        const uint8_t peak_offset_percent_gait_idx = 9;         //Percent of Gait Cycle where torque stops being applied
        const uint8_t use_pid_idx = 10;                         //Flag to determine whether or not PID used
        const uint8_t p_gain_idx = 11;                          //Value of P Gain for PID control
        const uint8_t i_gain_idx = 12;                          //Value of I Gain for PID control
        const uint8_t d_gain_idx = 13;                          //Value of D Gain for PID control 
        const uint8_t num_parameter = 14;                   
    }

    namespace constant_torque
    {
        const uint8_t amplitude_idx = 0;
        const uint8_t direction_idx = 1;
        const uint8_t alpha_idx = 2;
        const uint8_t use_pid_idx = 3;
        const uint8_t p_gain_idx = 4;
        const uint8_t i_gain_idx = 5;
        const uint8_t d_gain_idx = 6;
        const uint8_t num_parameter = 7;

    }

    namespace elbow_min_max
    {
        const uint8_t amplitude_idx = 0;    // amplitude in Nm
        const uint8_t fsr_threshold_idx = 1;  
		const uint8_t fsr_threshold1_idx = 2;
        const uint8_t num_parameter = 3;
        
        
    }

    namespace ptb_general 
    {
        const uint8_t ptb_mode_idx = 0;
        const uint8_t ptb_settings_1_idx = 1;
        const uint8_t ptb_settings_2_idx = 2;
        const uint8_t ptb_settings_3_idx = 3;
        const uint8_t ptb_settings_4_idx = 4;
        const uint8_t use_pid_idx = 5;
        const uint8_t p_gain_idx = 6;
        const uint8_t i_gain_idx = 7;
        const uint8_t d_gain_idx = 8;
		const uint8_t ptb_settings_9_idx = 9;
        const uint8_t num_parameter = 10;
    }

    namespace propulsive_assistive 
    {
        const uint8_t plantar_scaling = 0;
        const uint8_t dorsi_scaling = 1;
        const uint8_t timing_threshold = 2;
        const uint8_t spring_stiffness = 3;
        const uint8_t neutral_angle = 4;
        const uint8_t damping = 5;
        const uint8_t propulsive_gain = 6;
        const uint8_t kp = 7;
        const uint8_t kd = 8;
		const uint8_t turn_on_peak_limiter = 9;
        const uint8_t num_parameter = 10;
    }
	
	namespace calibr_manager
	{
		const uint8_t calibr_cmd = 0;
		const uint8_t num_parameter = 1;
	}

    namespace hip_resist
    {
        // parameters for maximum exo extension and flexion torque.
        const uint8_t flexion_setpoint_idx = 0;
        const uint8_t extension_setpoint_idx = 1;
        const uint8_t direction_idx = 2;
        const uint8_t num_parameter = 3;
    }

    namespace chirp
    {
        // Parameters for Sine Wave Used in Chirp Testing
        const uint8_t amplitude_idx = 0;
        const uint8_t start_frequency_idx = 1;
        const uint8_t end_frequency_idx = 2;
        const uint8_t duration_idx = 3;
        const uint8_t yshift_idx = 4;
        const uint8_t pid_flag_idx = 5;
        const uint8_t p_gain_idx = 6;
        const uint8_t i_gain_idx = 7;
        const uint8_t d_gain_idx = 8;
        const uint8_t num_parameter = 9;
    }

    namespace step
    {
        // Parameters for step torque used in max torque capacity testing
        const uint8_t amplitude_idx = 0;
        const uint8_t duration_idx = 1;
        const uint8_t repetitions_idx = 2;
        const uint8_t spacing_idx = 3;
        const uint8_t pid_flag_idx = 4;
        const uint8_t p_gain_idx = 5;
        const uint8_t i_gain_idx = 6;
        const uint8_t d_gain_idx = 7;
        const uint8_t alpha_idx = 8;
        const uint8_t num_parameter = 9;
    }

    const uint8_t max_parameters = franks_collins_hip::num_parameter;//user_defined::num_parameter;  // this should be the largest of all the num_parameters
}

/**
 * @brief class to store information related to controllers.
 * 
 */
class ControllerData {
	public:
        ControllerData(config_defs::joint_id id, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the controller data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);

        /**
         * @brief Get the parameter length for the current controller
         * 
         * @return uint8_t parameter length 
         */
        uint8_t get_parameter_length();
        
        
        uint8_t controller; /**< id of the current controller */
        config_defs::JointType joint; /**< id of the current joint */
        // These were made floats to dummy proof the math for people but will double the data needed to be sent over SPI, we double the speed of the SPI if we move to fixed point.
        float setpoint;  /**< controller setpoint, basically the motor command. */
        float ff_setpoint; /**< feed forwared setpoint, only updated in closed loop controllers */
        float parameters[controller_defs::max_parameters];  /**< Parameter list for the controller see the controller_defs namespace for the specific controller. */
        uint8_t parameter_set; /**< temporary value used to store the parameter set while we are pulling from the sd card. */

        float filtered_torque_reading; /**< filtered torque reading, used for filtering torque signal */
        float filtered_cmd; /**< filtered command, used for filtering motor commands */
        float filtered_setpoint = 0; /**< filtered setpoint for the controller */
        
        // Variables for Auto Kf in the PID Controller
        float kf = 1; /**< gain for the controller */
        float prev_max_measured = 0; /**< previous max measured value */
        float prev_max_setpoint = 0; /**< previous max setpoint value */
        float max_measured = 0; /**< max measured value */
        float max_setpoint = 0; /**< max setpoint value */

        // Variables for GAsP Controller
        float reference_angle = 0; /**< reference angle for the spring term */
        float reference_angle_offset = 0; /**< offset for the reference angle */
        bool reference_angle_updated = false; /**< flag to indicate if the reference angle was updated this step */
        float filtered_squelched_supportive_term = 0; /**< low pass on final spring output */
        float neutral_angle = 0.0f; /**< neutral angle for the spring term */
        bool prev_calibrate_trq_sensor = false; /**< previous value of the calibrate torque sensor flag */
        const float cal_neutral_angle_alpha = 0.01f; /**< alpha for the low pass on the neutral angle calibration */
        float level_entrance_angle = 0.0f; /**< level entrance angle for the spring term */
        bool prev_calibrate_level_entrance = false; /**< previous value of the calibrate level entrance flag */
        const float cal_level_entrance_angle_alpha = 0.01f; /**< alpha for the low pass on the level entrance calibration */
		float stateless_pjmc_term = 0;
		float toeFsrThreshold = 0.2f;
		bool wait4HiHeelFSR = false;
		uint16_t iPidHiTorque = 0;
		bool pausePid = false;
		float currentTime = 0.0000f;
		float previousTime = 0.0000f;
		float itrTime = 0.0000f;
		uint8_t numBelow500 = 0;
		int maxTorqueCache = 0;
		float previousMaxCmdCache = 15;
		float previousMinCmdCache = -15;
		float currentMaxCmdCache = 0;
		float currentMinCmdCache = 0;
		uint16_t cmdCacheItr = 0;
		bool doIncrUpperLmt = false;
		bool doIncrLowerLmt = false;
		float setpoint2use = 0;
		float maxPjmcSpringDamper = 0;
		bool wasStance = false;
		float prevMaxPjmcSpringDamper = 0;
		float cmd_2nd = 0;
		float cmd_1st = 0;
		

        // Variables for the ElbowMinMax Controller
        float fsr_toe_min_elbow = 0;
        float fsr_toe_max_elbow = 0;
        float fsr_heel_min_elbow = 0;
        float fsr_heel_max_elbow = 0;
        float fsr_toe_sum_elbow = 0;
        float fsr_heel_sum_elbow = 0;
        int i_elbow = 0;
        int fsr_toe_array_elbow [50] = {0};
        int fsr_heel_array_elbow [50] = {0};
        bool is_first_run_elbow = true;
        float output_limit_elbow = 5;
        float fsr_min_max_elbow = 0;
        bool is_first_fsr_reading_elbow = true;
        float fsr_toe_previous_elbow = 0;
        float fsr_heel_previous_elbow = 0;
		float elbow_cmd = 0;

        // Variables for the Generalizable Perturbation Controller (ptbGeneral)
        bool isPerturbing = false;
        bool ptbDetermined = false;
        float time_current_ptb = 0;
        float time_previous_ptb = 0;
        //uint8_t ptbHead = 0;
        //uint8_t ptbTail = 0;
        //uint8_t iPercentGait= 0;
        bool ptbApplied = false;
        bool ptbRandomIsFirstRun = true;
        //bool ptbWait4ANewStep = true;
        //uint8_t ptbFrequency = 0;
        uint16_t ptb_iStep = 0;
		//uint16_t ptb_totalSteps = 0;
		bool ptb_newIsSwing = true;
		bool ptb_oldIsSwing = true;
		//uint16_t fsrThreshold = 0.3;
		uint8_t ptb_frequency = 0;
		uint16_t ptb_iiStep = 0;
		uint16_t ptb_setpoint = 0;
		bool ptb_fsrGotHigh = false;
		
		// Variables for the Calibration Manger "Controller"
		bool calibrComplete = false;
		uint16_t iCalibr = 0;
		int PIDMLTPLR = 0;
		bool calibrStart = false;
		float calibrSum = 0;

		
		// Variables for the Zhang-Collins Controller
		float previous_cmd = 0;
};      

#endif