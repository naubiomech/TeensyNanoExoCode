/**
 * @file ParamsFromSD.h
 *
 * @brief Declares the functions to pull controller parameters from the SD card and defines the mapping to the parameter files.
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef ParamsFromSD_h
#define ParamsFromSD_h

#include "ExoData.h"
#include "ParseIni.h"
#include "Utilities.h"

#include <SD.h>
#include <SPI.h>
#include <map>
#include <string>


// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    #ifndef SD_SELECT
        #define SD_SELECT BUILTIN_SDCARD
    #endif
    
    typedef std::map<uint8_t, std::string> ParamFilenameKey;
    /**
     * @brief types of errors when reading the SD card
     */
    namespace param_error
    {
        const uint8_t num_joint_ids = 3; /**< Number of bits the joint type ids need */
        const uint8_t SD_not_found_idx = num_joint_ids; /**< Error when SD card isn't present */
        const uint8_t file_not_found_idx = SD_not_found_idx + 1; /**< Error when file is not found on the SD card */
    }
    
    /**
     * @brief Namespace with map to between controller and file location
     */
    namespace controller_parameter_filenames
    {
        const ParamFilenameKey hip
        {
            // for disabled clear the parameters, may not want to use this if this is just a temp pause.  Same for zeroTorque
            {(uint8_t)config_defs::hip_controllers::disabled,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::zero_torque,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::heel_toe,"hipControllers/heelToe.csv"},
            {(uint8_t)config_defs::hip_controllers::extension_angle,"hipControllers/extensionAngle.csv"},
            {(uint8_t)config_defs::hip_controllers::franks_collins_hip, "hipControllers/franksCollins.csv"},
            {(uint8_t)config_defs::hip_controllers::bang_bang, "hipControllers/bangBang.csv"},
            {(uint8_t)config_defs::hip_controllers::late_stance, "hipControllers/lateStance.csv"},
            {(uint8_t)config_defs::hip_controllers::gait_phase, "hipControllers/gaitPhase.csv"},
            {(uint8_t)config_defs::hip_controllers::user_defined, "hipControllers/userDefined.csv"},
            {(uint8_t)config_defs::hip_controllers::sine, "hipControllers/sine.csv"},
            {(uint8_t)config_defs::hip_controllers::stasis, "hipControllers/stasis.csv"},
            {(uint8_t)config_defs::hip_controllers::perturbation, "hipControllers/perturbation.csv"},
            {(uint8_t)config_defs::hip_controllers::parabolic, "hipControllers/parabolic.csv"},
            {(uint8_t)config_defs::hip_controllers::constant_torque, "hipControllers/constantTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::ptb_general,"hipControllers/ptbGeneral.csv"},
        };
        
        const ParamFilenameKey knee
        {
            {(uint8_t)config_defs::knee_controllers::disabled,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::zero_torque,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::user_defined, "kneeControllers/userDefined.csv"},
            {(uint8_t)config_defs::knee_controllers::sine, "kneeControllers/sine.csv"},
            {(uint8_t)config_defs::knee_controllers::stasis, "kneeControllers/stasis.csv"},
            {(uint8_t)config_defs::knee_controllers::perturbation, "kneeControllers/perturbation.csv"},
            {(uint8_t)config_defs::knee_controllers::constant_torque, "kneeControllers/constantTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::elbow_min_max, "kneeControllers/elbowMinMax.csv"},
        };
        
        const ParamFilenameKey ankle
        {
            {(uint8_t)config_defs::ankle_controllers::disabled,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::zero_torque,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::pjmc,"ankleControllers/PJMC.csv"},
            {(uint8_t)config_defs::ankle_controllers::zhang_collins,"ankleControllers/zhangCollins.csv"},
            {(uint8_t)config_defs::ankle_controllers::user_defined, "ankleControllers/userDefined.csv"},
            {(uint8_t)config_defs::ankle_controllers::sine, "ankleControllers/sine.csv"},
            {(uint8_t)config_defs::ankle_controllers::stasis, "ankleControllers/stasis.csv"},
            {(uint8_t)config_defs::ankle_controllers::perturbation, "ankleControllers/perturbation.csv"},
            {(uint8_t)config_defs::ankle_controllers::constant_torque, "ankleControllers/constantTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::ptb_general,"ankleControllers/ptbGeneral.csv"},
            {(uint8_t)config_defs::ankle_controllers::gasp,"ankleControllers/GAsP.csv"},
			{(uint8_t)config_defs::ankle_controllers::elbow_min_max, "ankleControllers/elbowMinMax.csv"},
        };
    };
    
    /**
     * @brief prints name of error message
     *
     * @param error identifier
     */
    void print_param_error_message(uint8_t error_type);
    
    /**
     * @brief Reads files from SD card and sets them to the appropriate controller parameters in the exo_data object
     * see ParseIni for details on inputs
     * 
     * @param joint_id : the joint id 
     * @param controller_id : the controller id 
     * @param set_num : parameter set to read from the SD card
     * @param exo_data : location to put the data 
     * 
     * @return : Error int.
     */
    uint8_t set_controller_params(uint8_t joint_id, uint8_t controller_id, uint8_t set_num, ExoData* exo_data);

#endif
#endif