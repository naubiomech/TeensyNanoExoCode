```
ExoData
{
    reconfigure(uint8_t* config_to_send)
    for_each_joint(for_each_joint_function_t function)
    
    status
    sync_led_state
    estop  
    
    LegData : left_leg, right_leg
    {
        reconfigure(uint8_t* config_to_send)
        
        percent_gait
        expected_step_duration
        heel_fsr
        toe_fsr 
        ground_strike 
        expected_duration_window_upper_coeff
        expected_duration_window_lower_coeff
        is_left
        is_used
        do_calibration_toe_fsr
        do_calibration_refinement_toe_fsr
        do_calibration_heel_fsr
        do_calibration_refinement_heel_fsr
        
        JointData : hip, knee, ankle
        {
            reconfigure(uint8_t* config_to_send) 
            
            id
            torque_reading
            is_left
            is_used 
            flip_direction
            calibrate_torque_sensor 
            position
            velocity
            
            motor
            {
                id
                motor_type
                is_left
                is_on
                enabled
                do_zero
                flip_direction
                gearing
                p 
                p_des
                v
                v_des
                i
                t_ff
                kp
                kd
            } 
            
            controller
            {
                reconfigure(uint8_t* config_to_send) 
                
                controller
                joint
                setpoint
                parameters[controller_defs::max_parameters]
            }
        }  
    }
}
```