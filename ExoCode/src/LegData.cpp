#include "LegData.h"
#include "Logger.h"
/*
 * Constructor for the leg data.
 * Takes if the leg is left, and the array from the INI parser.
 * Stores if the leg is left, the percent gait, FSR data, and if the leg is used.
 * Uses an initializer list for the joint data, and creates the joint id for the joints. 
 */
LegData::LegData(bool is_left, uint8_t* config_to_send)
: hip((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::hip), config_to_send, is_left ? config_to_send[config_defs::left_hip_RoM_idx] : config_to_send[config_defs::right_hip_RoM_idx], config_to_send[config_defs::hip_flip_angle_dir_idx], is_left ? config_to_send[config_defs::left_hip_torque_offset_idx] : config_to_send[config_defs::right_hip_torque_offset_idx], config_to_send[config_defs::motor_pos_safety_factor_idx], is_left ? config_to_send[config_defs::left_hip_motor_RoM_idx] : config_to_send[config_defs::right_hip_motor_RoM_idx])
, knee((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::knee), config_to_send, is_left ? config_to_send[config_defs::left_knee_RoM_idx] : config_to_send[config_defs::right_knee_RoM_idx], config_to_send[config_defs::knee_flip_angle_dir_idx], is_left ? config_to_send[config_defs::left_knee_torque_offset_idx] : config_to_send[config_defs::right_knee_torque_offset_idx], config_to_send[config_defs::motor_pos_safety_factor_idx], is_left ? config_to_send[config_defs::left_knee_motor_RoM_idx] : config_to_send[config_defs::right_knee_motor_RoM_idx])
, ankle((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::ankle), config_to_send, is_left ? config_to_send[config_defs::left_ankle_RoM_idx] : config_to_send[config_defs::right_ankle_RoM_idx], config_to_send[config_defs::ankle_flip_angle_dir_idx], is_left ? config_to_send[config_defs::left_ankle_torque_offset_idx] : config_to_send[config_defs::right_ankle_torque_offset_idx], config_to_send[config_defs::motor_pos_safety_factor_idx], is_left ? config_to_send[config_defs::left_ankle_motor_RoM_idx] : config_to_send[config_defs::right_ankle_motor_RoM_idx]) 
{
    this->is_left = is_left;
    
    this->percent_gait = -1; // likely want to do fixed point
    this->percent_stance = -1;
    this->percent_swing = -1;
    this->expected_step_duration = -1; 
    this->expected_swing_duration = -1;
    this->expected_stance_duration = -1;
    this->heel_fsr = -1; // set to -1 since should always be positive once set.
    this->toe_fsr = -1;
    this->do_calibration_toe_fsr = false; 
    this->do_calibration_refinement_toe_fsr = false; 
    this->do_calibration_heel_fsr = false; 
    this->do_calibration_refinement_heel_fsr = false; 
    this->ground_strike = false; 
    this->toe_off = false;
    this->toe_strike = false;
    this->toe_on = false;
    
    this->expected_duration_window_upper_coeff = 1.75;
    this->expected_duration_window_lower_coeff = 0.25;
    this->ankle_angle_at_ground_strike = 0.5;

    this->inclination = Inclination::Level;

    this->PHJM_state = 0;

    // check if the leg is used from the config.
    if ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
        || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) & this->is_left)
        || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) & !this->is_left)
       )
    {
        this->is_used = true;
    }
    else
    {
        this->is_used = false;
    }
        
};

void LegData::reconfigure(uint8_t* config_to_send) 
{
    if ((static_cast<uint8_t>(config_defs::exo_side::bilateral) == config_to_send[config_defs::exo_side_idx]) 
        || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]) & this->is_left)
        || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]) & !this->is_left)
       )
    {
        this->is_used = true;
    }
    else
    {
        this->is_used = false;
    }
    
    // reconfigure the joints
    hip.reconfigure(config_to_send);
    knee.reconfigure(config_to_send);
    ankle.reconfigure(config_to_send);
};
 