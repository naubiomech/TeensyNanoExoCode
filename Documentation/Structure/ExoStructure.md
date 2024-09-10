```
Exo
{
    run()
    
    *data
    
    SyncLed : sync_led
    {
        trigger()
        update_led()
        update_periods(int sync_start_stop_half_period_us, int sync_half_period_us)
        bool handler() 
        set_default_state(int new_default)
        bool get_led_is_on()
        bool get_is_blinking()
        
        _blink_start_stop()
        _blink()
        _default_state()
        
        _led_state
        _led_is_on
        _current_sync_period
        _do_blink
        _do_start_stop_sequence
        _is_blinking
        _pin
        _default_led_state
        _led_default_state_pin
        _sync_start_stop_half_period_us
        _sync_half_period_us
        _state_change_count
        _num_start_stop_blinks
        _last_state_change_timestamp_us
    }
    
    StatusLed : status_led
    {
        StatusLed(int r_pin, int g_pin, int b_pin)
        StatusLed(int r_pin, int g_pin, int b_pin, int brightness)
        update(uint16_t message)
        set_brightness(int brightness) 
        
        _set_color(int R, int G, int B)
        _solid() 
        _pulse()
        _blink()
        _rainbow_sin()
        _rainbow_hsv()
        
        _r_pin
        _g_pin
        _b_pin
        _brightness
        _current_message
        _pattern_start_timestamp
        _period_ms
        _pattern_brightness_percent
        _message_colors[n][3]
        _message_pattern[n][2]
    }
    
    Side : left_side, right_side
    {
        run_side()
        check_calibration()
        read_data()
        update_motor_cmds()
        set_controller(int joint, int controller)
        clear_step_time_estimate()
        
        _calc_percent_gait() 
        _update_expected_duration()
        _check_ground_strike()
        
        *_data
        *_side_data
        _is_left
        _prev_heel_contact_state
        _prev_toe_contact_state
        _num_steps_avg
        _step_times[_num_steps_avg]
        _ground_strike_timestamp
        _prev_ground_strike_timestamp
        _expected_step_duration
        
        FSR : _heel_fsr, _toe_fsr
        {
            bool calibrate(bool do_calibrate)
            bool refine_calibration(bool do_refinement)
            float read()
            bool get_ground_contact()
            
            _calc_ground_contact()
            
            _raw_reading 
            _calibrated_reading
            _pin
            _cal_time
            _start_time
            _last_do_calibrate
            _calibration_min
            _calibration_max
            _num_steps
            _lower_threshold_percent_calibration_refinement
            _upper_threshold_percent_calibration_refinement
            _state
            _last_do_refinement
            _step_max_sum
            _step_max
            _step_min_sum
            _step_min
            _step_count
            _calibration_refinement_min
            _calibration_refinement_max
            _ground_contact
            _lower_threshold_percent_ground_contact
            _upper_threshold_percent_ground_contact
        }
        
        Joint : _hip, _knee, _ankle
        {
            run_joint()
            read_data()
            check_calibration()
            set_controller(uint8_t controller_id)
            set_motor(_Motor* new_motor)
            static unsigned int get_torque_sensor_pin(config_defs::joint_id, ExoData*)
            static unsigned int get_motor_enable_pin(config_defs::joint_id, ExoData*)
             
            *_data
            *_joint_data
            _id
            _is_left
            
            hip, knee, or ankle specific controllers
            
            *motor
            {
                read_data()
                send_data(float torque)
                transaction(float torque)
                on_off()
                bool enable()
                bool enable(bool override)
                zero()
                bool get_is_left()
                joint_id get_id()
                
                *_data
                *_motor_data
                _id
                _is_left
                _enable_pin
                _prev_motor_enabled
                _prev_on_state
                
                Some protected Functions for specific motor type
            } 
             
            _torque_sensor
            {
                bool calibrate(bool do_calibration)
                int read()
                
                _pin
                _is_used
                _calibration
                _raw_reading
                _calibrated_reading
                _cal_time
                _start_time
                _last_do_calibrate
                _zero_sum
                _num_calibration_samples
            }
            
            *_controller
            {
               float calc_motor_cmd()
               reset_integral() 
               
               float _pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain);
        
               *_data
               * _controller_data
               * _side_data
               * _joint_data
               _id
               _integral_val
               _prev_error
               
            }
        }  
    }
}
```