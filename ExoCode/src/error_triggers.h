#ifndef ERROR_TRIGGERS_H
#define ERROR_TRIGGERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"

#include "Arduino.h"
#include "Utilities.h"
#include "Config.h"
#include "ParseIni.h"

// TODO: Make these device configuration dependent

namespace error_triggers_state
{
    bool triggered_error = false;

    // Motor Inertia goodies
    const float motor_inertia_threshold = 0.01;
    float left_prev_velocity = 0;
    float right_prev_velocity = 0;

    // Motor Position goodies
    const float motor_position_alpha = 0.1;
    const float motor_position_tolerance = 10; // degrees
    float filtered_motor_position_left = 0;
    float filtered_motor_position_right = 0;
    const std::pair<float, float> left_position_bounds = std::make_pair<float, float> (12-motor_position_tolerance, 10+motor_position_tolerance);
    const std::pair<float, float> right_position_bounds = std::make_pair<float, float> (12-motor_position_tolerance, 10+motor_position_tolerance);

    // Torque goodies
    const float torque_output_alpha = 0.2;
    const float torque_output_threshold = 45;
    float average_torque_output_left = 0;
    float average_torque_output_right = 0;

    // X-Sigma Rejection on sensor goodies
    const int failure_count_threshold = 2;

    const int torque_std_dev_multiple = 10;
    const float torque_window_size = 100;
    std::queue<float> torque_sensor_queue_left;
    std::queue<float> torque_sensor_queue_right;
    int torque_failure_count_left = 0;
    int torque_failure_count_right = 0;
    
    const int force_std_dev_multiple = 10;
    const float force_window_size = 100;
    std::queue<float> force_sensor_queue_left;
    std::queue<float> force_sensor_queue_right;
    int force_failure_count_left = 0;
    int force_failure_count_right = 0;

    // Tracking goodies
    const float tracking_alpha = 0.1;
    const float tracking_threshold = 15; // Nm
    float average_tracking_error_left = 0;
    float average_tracking_error_right = 0;

    // Transimission efficiency goodies
    const float transmission_efficiency_alpha = 0.1;
    const float transmission_efficiency_threshold = 0.01;
    float average_motor_torque_left = 0;
    float average_motor_torque_right = 0;

    // PJMC State goodies
    // alpha = 2 / (N + 1), where N is the equivalent moving average window size
    const float pjmc_state_time = 25; // seconds
    const float pjmc_state_alpha = 2 / ((pjmc_state_time * LOOP_FREQ_HZ) + 1);
    const float pjmc_state_threshold = 0.95;
    float average_pjmc_state_left = 0;
    float average_pjmc_state_right = 0;
}

// Returns true if error
namespace error_triggers
{
    int soft(Exo* exo, ExoData* exo_data)
    {
        // if (error_triggers_state::triggered_error)
        //     return NO_ERROR;
        return NO_ERROR;
    }
    
    int hard(Exo* exo, ExoData* exo_data)
    {
        // if (error_triggers_state::triggered_error)
        //     return NO_ERROR;
        // Check if the legs have been in stance for too long
        // error_triggers_state::average_pjmc_state_left = utils::ewma(exo_data->left_leg.toe_stance,
        //                 error_triggers_state::average_pjmc_state_left, error_triggers_state::pjmc_state_alpha);
        // error_triggers_state::average_pjmc_state_right = utils::ewma(exo_data->right_leg.toe_stance,
        //                 error_triggers_state::average_pjmc_state_right, error_triggers_state::pjmc_state_alpha);
        // bool left_stance_error = (error_triggers_state::average_pjmc_state_left > error_triggers_state::pjmc_state_threshold);
        // bool right_stance_error = (error_triggers_state::average_pjmc_state_right > error_triggers_state::pjmc_state_threshold);
        // if (left_stance_error || right_stance_error)
        // {
        //     //logger::println("Error: PJMC State too high");
        //     error_triggers_state::triggered_error = true;
        //     exo_data->error_joint_id = (left_stance_error) ? (uint8_t)config_defs::joint_id::left_ankle : (uint8_t)config_defs::joint_id::right_ankle;
        //     return POOR_STATE_VARIANCE;
        // }
        
        return NO_ERROR;
    }

    int fatal(Exo* exo, ExoData* exo_data)
    {
        // if (error_triggers_state::triggered_error)
        //     return NO_ERROR;

        // Only run for configurations with torque sensors
        if (exo_data->config[config_defs::exo_name_idx] == (uint8_t)config_defs::exo_name::bilateral_ankle ||
            exo_data->config[config_defs::exo_name_idx] == (uint8_t)config_defs::exo_name::bilateral_hip_ankle)
        {
            // Check if the torque is too high
            error_triggers_state::average_torque_output_left = utils::ewma(exo_data->left_leg.ankle.torque_reading,
                            error_triggers_state::average_torque_output_left, error_triggers_state::torque_output_alpha);
            error_triggers_state::average_torque_output_right = utils::ewma(exo_data->right_leg.ankle.torque_reading*-1,
                            error_triggers_state::average_torque_output_right, error_triggers_state::torque_output_alpha);
            bool left_torque_error = (abs(error_triggers_state::average_torque_output_left) > error_triggers_state::torque_output_threshold);
            bool right_torque_error = (abs(error_triggers_state::average_torque_output_right) > error_triggers_state::torque_output_threshold);
            if (left_torque_error || right_torque_error)
            {
                //logger::println("Error: Torque too high");
                error_triggers_state::triggered_error = true;
                exo_data->error_joint_id = (left_torque_error) ? (uint8_t)config_defs::joint_id::left_ankle : (uint8_t)config_defs::joint_id::right_ankle;
                return TORQUE_OUT_OF_BOUNDS;
            }

            // Check if the tracking error is too high
            // float tracking_error_left = exo_data->left_leg.ankle.torque_reading - exo_data->left_leg.ankle.controller.ff_setpoint;
            // float tracking_error_right = (exo_data->right_leg.ankle.torque_reading*-1) - exo_data->right_leg.ankle.controller.ff_setpoint;
            // error_triggers_state::average_tracking_error_left = utils::ewma(tracking_error_left,
            //                 error_triggers_state::average_tracking_error_left, error_triggers_state::tracking_alpha);
            // error_triggers_state::average_tracking_error_right = utils::ewma(tracking_error_right,
            //                 error_triggers_state::average_tracking_error_right, error_triggers_state::tracking_alpha);
            // bool left_tracking_error = (abs(error_triggers_state::average_tracking_error_left) > error_triggers_state::tracking_threshold);
            // bool right_tracking_error = (abs(error_triggers_state::average_tracking_error_right) > error_triggers_state::tracking_threshold);
            // if (left_tracking_error || right_tracking_error)
            // {
            //     //logger::println("Error: Tracking error too high");
            //     error_triggers_state::triggered_error = true;
            //     exo_data->error_joint_id = (left_tracking_error) ? (uint8_t)config_defs::joint_id::left_ankle : (uint8_t)config_defs::joint_id::right_ankle;
            //     return TRACKING_ERROR;
            // }
  

            // Check the torque sensor variance, if the variance is too high, then the sensor may be faulty
            error_triggers_state::torque_sensor_queue_left.push(exo_data->left_leg.ankle.torque_reading);
            error_triggers_state::torque_sensor_queue_right.push(exo_data->right_leg.ankle.torque_reading*-1);
            if (error_triggers_state::torque_sensor_queue_left.size() > error_triggers_state::torque_window_size) 
            {
                error_triggers_state::torque_sensor_queue_left.pop();
                error_triggers_state::torque_sensor_queue_right.pop();

                std::pair<float, float> left_population_vals = utils::online_std_dev(error_triggers_state::torque_sensor_queue_left);
                std::pair<float, float> right_population_vals = utils::online_std_dev(error_triggers_state::torque_sensor_queue_right);
                std::pair<float, float> left_bounds = std::make_pair(left_population_vals.first - error_triggers_state::torque_std_dev_multiple*left_population_vals.second,
                                                                    left_population_vals.first + error_triggers_state::torque_std_dev_multiple*left_population_vals.second);
                std::pair<float, float> right_bounds = std::make_pair(right_population_vals.first - error_triggers_state::torque_std_dev_multiple*right_population_vals.second,
                                                                    right_population_vals.first + error_triggers_state::torque_std_dev_multiple*right_population_vals.second);

                error_triggers_state::torque_failure_count_left += float(utils::is_outside_range(exo_data->left_leg.ankle.torque_reading, left_bounds.first, left_bounds.second));
                error_triggers_state::torque_failure_count_right += float(utils::is_outside_range(exo_data->right_leg.ankle.torque_reading*-1, right_bounds.first, right_bounds.second));
                bool left_torque_variance_error = (error_triggers_state::torque_failure_count_left > error_triggers_state::failure_count_threshold);
                bool right_torque_variance_error = (error_triggers_state::torque_failure_count_right > error_triggers_state::failure_count_threshold);
                if (left_torque_variance_error || right_torque_variance_error)
                {
                    //logger::println("Error: Torque sensor variance too high");
                    error_triggers_state::triggered_error = true;
                    exo_data->error_joint_id = (left_torque_variance_error) ? (uint8_t)config_defs::joint_id::left_ankle : (uint8_t)config_defs::joint_id::right_ankle;
                    return TORQUE_VARIANCE_ERROR;
                }
            }

            // Check the force sensor variance, if the variance is too high, then the sensor may be faulty
            error_triggers_state::force_sensor_queue_left.push(exo_data->left_leg.toe_fsr);
            error_triggers_state::force_sensor_queue_right.push(exo_data->right_leg.toe_fsr);
            if (error_triggers_state::force_sensor_queue_left.size() > error_triggers_state::force_window_size) 
            {
                error_triggers_state::force_sensor_queue_left.pop();
                error_triggers_state::force_sensor_queue_right.pop();

                std::pair<float, float> left_population_vals = utils::online_std_dev(error_triggers_state::force_sensor_queue_left);
                std::pair<float, float> right_population_vals = utils::online_std_dev(error_triggers_state::force_sensor_queue_right);
                std::pair<float, float> left_bounds = std::make_pair(left_population_vals.first - error_triggers_state::force_std_dev_multiple*left_population_vals.second,
                                                                    left_population_vals.first + error_triggers_state::force_std_dev_multiple*left_population_vals.second);
                std::pair<float, float> right_bounds = std::make_pair(right_population_vals.first - error_triggers_state::force_std_dev_multiple*right_population_vals.second,
                                                                    right_population_vals.first + error_triggers_state::force_std_dev_multiple*right_population_vals.second);

                error_triggers_state::force_failure_count_left += float(utils::is_outside_range(exo_data->left_leg.toe_fsr, left_bounds.first, left_bounds.second));
                error_triggers_state::force_failure_count_right += float(utils::is_outside_range(exo_data->right_leg.toe_fsr, right_bounds.first, right_bounds.second));
                bool left_force_outside_range = utils::is_outside_range(exo_data->left_leg.toe_fsr, left_bounds.first, left_bounds.second);
                bool right_force_outside_range = utils::is_outside_range(exo_data->right_leg.toe_fsr, right_bounds.first, right_bounds.second);
                if (error_triggers_state::force_failure_count_left > error_triggers_state::failure_count_threshold ||
                    error_triggers_state::force_failure_count_right > error_triggers_state::failure_count_threshold)
                {
                    error_triggers_state::triggered_error = true;
                    exo_data->error_joint_id = (left_force_outside_range) ? (uint8_t)config_defs::joint_id::left_ankle : (uint8_t)config_defs::joint_id::right_ankle;
                    return FORCE_VARIANCE_ERROR;
                }
            }


            // Limit the motor positions, Too much drift!
            // error_triggers_state::filtered_motor_position_left = utils::ewma(exo_data->left_leg.ankle.motor.p, 
            //             error_triggers_state::filtered_motor_position_left, error_triggers_state::motor_position_alpha);
            // error_triggers_state::filtered_motor_position_right = utils::ewma(exo_data->right_leg.ankle.motor.p,
            //             error_triggers_state::filtered_motor_position_right, error_triggers_state::motor_position_alpha);

            // // print the motor positions every 100 iterations
            // static int count = 0;
            // if (count++ % 100 == 0)
            // {
            //     logger::print("LP:"+String(error_triggers_state::filtered_motor_position_left)+"\t");
            //     logger::print("RP:"+String(error_triggers_state::filtered_motor_position_right)+"\n");
            // }

            // bool left_motor_position_error = utils::is_outside_range(error_triggers_state::filtered_motor_position_left, 
            //             error_triggers_state::left_position_bounds.first, error_triggers_state::left_position_bounds.second);
            // bool right_motor_position_error = utils::is_outside_range(error_triggers_state::filtered_motor_position_right,
            //             error_triggers_state::right_position_bounds.first, error_triggers_state::right_position_bounds.second);

            // if (left_motor_position_error || right_motor_position_error)
            // {
            //     error_triggers_state::triggered_error = true;
            //     return MOTOR_POSTION_OUT_OF_BOUNDS;
            // }

            // // Check the motors a mechanical break, if the moment of ineria drops there may be a break, Too much noise!
            // const float left_motor_torque = exo_data->left_leg.ankle.motor.i * exo->left_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::left_ankle);
            // const float right_motor_torque = exo_data->right_leg.ankle.motor.i * exo->right_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::right_ankle);
            // const float left_motor_accel = (exo_data->left_leg.ankle.motor.v - error_triggers_state::left_prev_velocity) / (1000 / LOOP_FREQ_HZ);
            // const float right_motor_accel = (exo_data->right_leg.ankle.motor.v - error_triggers_state::right_prev_velocity) / (1000 / LOOP_FREQ_HZ);

            // error_triggers_state::left_prev_velocity = exo_data->left_leg.ankle.motor.v;
            // error_triggers_state::right_prev_velocity = exo_data->right_leg.ankle.motor.v;

            // float left_motor_inertia = 0;
            // float right_motor_inertia = 0;

            // // if the motor is not accelerating, then the inertia is 0
            // if (!utils::is_close_to(left_motor_accel, 0, 0.001))
            // {
            //     left_motor_inertia = left_motor_torque / left_motor_accel;
            // }
            // if (!utils::is_close_to(right_motor_accel, 0, 0.001))
            // {
            //     right_motor_inertia = right_motor_torque / right_motor_accel;
            // }

            // // print the motor inertia every 100 loops
            // static int loop_count = 0;
            // loop_count++;
            // if (loop_count >= 99)
            // {
            //     loop_count = 0;
            //     // logger::print("LN:"+String(left_motor_torque)+"\t");
            //     // logger::print("LD:"+String(left_motor_accel)+"\t");
            //     //logger::print("LI:"+String(left_motor_inertia)+"\t");
            //     logger::print("RN:"+String(right_motor_torque)+"\t");
            //     logger::print("RD:"+String(right_motor_accel)+"\t");
            //     logger::print("RI:"+String(right_motor_inertia)+"\n");
            // }

            // if (left_motor_inertia < error_triggers_state::motor_inertia_threshold ||
            //     right_motor_inertia < error_triggers_state::motor_inertia_threshold)
                // {
                //     error_triggers_state::triggered_error = true;
                //     return MOTOR_INERTIA_ERROR;
                // }

            // Check the transmission efficiency. If its too low, the cable may be broken. Must low pass motor current to account for time delay, Too much noise!
            // float left_motor_torque = abs(exo_data->left_leg.ankle.motor.i) * exo->left_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::left_ankle);
            // error_triggers_state::average_motor_torque_left = utils::ewma(abs(left_motor_torque),
            //                 error_triggers_state::average_motor_torque_left, error_triggers_state::transmission_efficiency_alpha); 
            // float right_motor_torque = abs(exo_data->right_leg.ankle.motor.i) * exo->right_leg.get_Kt_for_joint((uint8_t) config_defs::joint_id::right_ankle);
            // error_triggers_state::average_motor_torque_right = utils::ewma(abs(right_motor_torque),
            //                 error_triggers_state::average_motor_torque_right, error_triggers_state::transmission_efficiency_alpha);
            
            // float left_transmission_efficiency = 1;
            // float right_transmission_efficiency = 1;
            // if (!utils::is_close_to(error_triggers_state::average_motor_torque_left, 0, 0.001))
            // {
            //     left_transmission_efficiency = exo_data->left_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_left;
            // }
            // if (!utils::is_close_to(error_triggers_state::average_motor_torque_right, 0, 0.001))
            // {
            //     right_transmission_efficiency = exo_data->right_leg.ankle.torque_reading / error_triggers_state::average_motor_torque_right;
            // }

            // if ((abs(left_transmission_efficiency) < error_triggers_state::transmission_efficiency_threshold) ||
            //     (abs(right_transmission_efficiency) < error_triggers_state::transmission_efficiency_threshold))
            // {
            //     logger::println("Error: Transmission efficiency too low");
            //     return POOR_TRANSMISSION_EFFICIENCY;
            // }
        }
        
        return NO_ERROR;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)