/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Controller.h"
#include "Logger.h"
//#define CONTROLLER_DEBUG

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
#include <math.h>
#include <random>

_Controller::_Controller(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _data = exo_data;
    
    _t_helper = Time_Helper::get_instance();
    _t_helper_context = _t_helper->generate_new_context();
    _t_helper_delta_t = 0;
    
    // we just need to know the side to point at the right data location so it is only for the constructor
    bool is_left = utils::get_is_left(_id);
    #ifdef CONTROLLER_DEBUG
        logger::print(is_left ? "Left " : "Right ");
    #endif 
    
    _integral_val = 0;
    _prev_input = 0; 
    _prev_de_dt = 0;
        
    // set _controller_data to point to the data specific to the controller.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            #ifdef CONTROLLER_DEBUG
                logger::print("HIP ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.hip.controller);
                _joint_data = &(exo_data->left_leg.hip);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.hip.controller);
                _joint_data = &(exo_data->right_leg.hip);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            #ifdef CONTROLLER_DEBUG
                logger::print("KNEE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.knee.controller);
                _joint_data = &(exo_data->left_leg.knee);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.knee.controller);
                _joint_data = &(exo_data->right_leg.knee);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            #ifdef CONTROLLER_DEBUG
                logger::print("ANKLE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_leg.ankle.controller);
                _joint_data = &(exo_data->left_leg.ankle);
            }
            else
            {
                _controller_data = &(exo_data->right_leg.ankle.controller);
                _joint_data = &(exo_data->right_leg.ankle);
            }
            break;
    }
    #ifdef CONTROLLER_DEBUG
        logger::print("Controller : \n\t_controller_data set \n\t_joint_data set");
    #endif
    // added a pointer to the leg data as most controllers will need to access info specific to their leg.
    if (is_left)
    {
        _leg_data = &(exo_data->left_leg);
    }
    else
    {
        _leg_data = &(exo_data->right_leg);
    } 
    #ifdef CONTROLLER_DEBUG
        logger::println("\n\t_leg_data set");
    #endif
    
    // Set the parameters for cf mfac
    measurements.first = -1;
    measurements.second = 1;
    outputs.first = 0;
    outputs.second = 0;
    phi.first = 2;
    phi.second = 0;
    rho = 0.5;
    lamda = 2;
    etta = 1;
    mu = 1;
    upsilon = 1 / (pow(10, 5));
    phi_1 = phi.first;
}

float _Controller::_cf_mfac(float reference, float current_measurement)
{
    // calculate k-1 (k_0) delta
    const float du_k0 = outputs.second - outputs.first;

    // prime the state
    measurements.first = measurements.second;
    outputs.first = outputs.second;
    phi.first = phi.second;
    measurements.second = current_measurement;

    // calculate k delta
    const float dy_k = measurements.second - measurements.first;

    // calculate the new psuedo partial derivative
    const float phi_numerator = etta * du_k0 * (dy_k - (phi.first*du_k0));
    const float phi_denominator = mu + (du_k0*du_k0);
    phi.second = phi.first + (phi_numerator/phi_denominator);

    // calculate the new output
    const float error = reference - measurements.second;
    const float u_numerator = rho * phi.second * error;
    const float u_denominator = lamda + (abs(phi.second) * abs(phi.second));
    outputs.second = outputs.first + (u_numerator/u_denominator);
    return outputs.second;
}
 
float _Controller::_pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain)
{
    //check if time is ok
    bool time_good = true;
    if (_t_helper->tick(_t_helper_context) > ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        time_good = false;
    }
    float now = micros();

    //TODO: Test dynamic time step
    float dt = (now - _prev_pid_time) * 1000000;

    float error_val = cmd - measurement;  
    //_integral_val += error_val / LOOP_FREQ_HZ;     
    float de_dt = 0;
    if (time_good)
    {
       de_dt = -(measurement - _prev_input) * (1000.0f / LOOP_FREQ_HZ);  // Convert to ms
       _prev_de_dt = de_dt;
    }
    else 
    {
        //de_dt = _prev_de_dt;
        de_dt = 0;
    }

    _prev_pid_time = now;
    _prev_input = measurement;

    float p = p_gain * error_val;  
    //float i = i_gain * _integral_val;  // resetting _integral_val was crashing the system 
    float d = d_gain * de_dt; 
    
    return p + d;
}

void _Controller::reset_integral()
{
    #ifdef CONTROLLER_DEBUG
        logger::println("_Controller::reset_integral : Entered");
    #endif
    //this -> _integral_val = 0;
    
    #ifdef CONTROLLER_DEBUG
        logger::println("_Controller::reset_integral : Exited");
    #endif
    
}

//****************************************************

ZeroTorque::ZeroTorque(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    #ifdef CONTROLLER_DEBUG
        logger::println("ZeroTorque::Constructor");
    #endif
    
}

float ZeroTorque::calc_motor_cmd()
{
    float cmd_ff = 0;
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff;
    if (_controller_data->parameters[controller_defs::zero_torque::use_pid_idx])
    {
        cmd = _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::zero_torque::p_gain_idx], _controller_data->parameters[controller_defs::zero_torque::i_gain_idx], _controller_data->parameters[controller_defs::zero_torque::d_gain_idx]);
    }
    _controller_data->ff_setpoint = cmd;

     //if (_joint_data->is_left) 
     //{
     //    logger::print("ZeroTorque::calc_motor_cmd : torque_reading = ");
     //    logger::print(_joint_data->torque_reading);
     //    logger::print("\t");
     //    logger::print("ZeroTorque::calc_motor_cmd : cmd = ");
     //    logger::println(cmd);
     //}

    // Print PD gains
     logger::print("ZeroTorque::calc_motor_cmd : p_gain = ");
     logger::println(_controller_data->parameters[controller_defs::zero_torque::p_gain_idx]);
    // logger::print("ZeroTorque::calc_motor_cmd : d_gain = ");
    // logger::println(_controller_data->parameters[controller_defs::zero_torque::d_gain_idx]);
    return cmd;
}

//****************************************************

Stasis::Stasis(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    
    #ifdef CONTROLLER_DEBUG
        logger::println("Stasis::Constructor");
    #endif
    
}

float Stasis::calc_motor_cmd()
{
    float cmd = 0;
    return cmd;
}


//****************************************************
PropulsiveAssistive::PropulsiveAssistive(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("PropulsiveAssistive::Constructor");
    #endif
}

void PropulsiveAssistive::_update_reference_angles(LegData* leg_data, ControllerData* controller_data, float percent_grf)
{
    const float threshold = controller_data->parameters[controller_defs::propulsive_assistive::timing_threshold]/100;
    // When the percent_grf passes the threshold, update the reference angle
    const bool should_update = (percent_grf > threshold) && !controller_data->reference_angle_updated;
    if (should_update)
    {
        controller_data->reference_angle_updated = true;
        controller_data->reference_angle = leg_data->ankle.joint_position;
        controller_data->reference_angle_offset = leg_data->ankle.joint_global_angle;
    }

    // When the percent_grf drops below the threshold, reset the reference angle updated flag and expire the reference angle
    const bool should_reset = (percent_grf < threshold) && controller_data->reference_angle_updated;
    if (should_reset)
    {
        controller_data->reference_angle_updated = false;
        controller_data->reference_angle = 0;
        controller_data->reference_angle_offset = 0;
    }
}

float PropulsiveAssistive::calc_motor_cmd()
{
    #ifdef CONTROLLER_DEBUG
    logger::println("PropulsiveAssistive::calc_motor_cmd : start");
    #endif
    static const float sigmoid_exp_scalar{50.0f};

    // Calculate Generic Contribution
    const float plantar_setpoint = _controller_data->parameters[controller_defs::propulsive_assistive::plantar_scaling];
    const float dorsi_setpoint = -_controller_data->parameters[controller_defs::propulsive_assistive::dorsi_scaling];
    const float threshold = _controller_data->parameters[controller_defs::propulsive_assistive::timing_threshold]/100;
    const float percent_grf = min(_leg_data->toe_fsr, 1);
    const float slope = (plantar_setpoint - dorsi_setpoint)/(1 - threshold);
    const float generic = max(((slope*(percent_grf - threshold)) + dorsi_setpoint), dorsi_setpoint);

    // Assistive Contribution
    _update_reference_angles(_leg_data, _controller_data, percent_grf);
    const float k = _controller_data->parameters[controller_defs::propulsive_assistive::spring_stiffness];
    const float b = _controller_data->parameters[controller_defs::propulsive_assistive::damping];
    const float equilibrium_angle_offset = _controller_data->parameters[controller_defs::propulsive_assistive::neutral_angle]/100;
    const float delta = _controller_data->reference_angle + equilibrium_angle_offset - _leg_data->ankle.joint_position;
    const float assistive = max(k*delta - b*_leg_data->ankle.joint_velocity, 0);
    // Use a tuned sigmoid to squelch the spring output during the 'swing' phase
    const float squelch_offset = -(1.5*threshold); // 1.5 ensures that the spring activates after the new angle is captured
    const float grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))) / 
            (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))+1);
    const float squelched_supportive_term = assistive*grf_squelch_multiplier;
    // low pass the squelched supportive term
    _controller_data->filtered_squelched_supportive_term = utils::ewma(squelched_supportive_term, 
            _controller_data->filtered_squelched_supportive_term, 0.075);

    // Propulsive Contribution
    const float kProp = _controller_data->parameters[controller_defs::propulsive_assistive::propulsive_gain];
    const float saturated_velocity = _leg_data->ankle.joint_velocity > 0 ? _leg_data->ankle.joint_velocity:0;
    const float propulsive = kProp*saturated_velocity;
    // Use a symmetric sigmoid to squelch the propulsive term
    const float propulsive_squelch_offset = -1.1 + threshold;
    const float propulsive_grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))) / 
            (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))+1);
    const float squelched_propulsive_term = propulsive*propulsive_grf_squelch_multiplier;
    
    // Sum for ff
    const float cmd_ff = _controller_data->filtered_squelched_supportive_term+generic+squelched_propulsive_term;

    // low pass filter on torque_reading
    const float torque = _joint_data->torque_reading * (_leg_data->is_left ? 1 : -1);
    const float alpha = 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, 
            _controller_data->filtered_torque_reading, alpha);

    // close the loop
    const float cmd = _pid(cmd_ff, _controller_data->filtered_torque_reading,
            _controller_data->parameters[controller_defs::propulsive_assistive::kp],
            0, 
            _controller_data->parameters[controller_defs::propulsive_assistive::kd]);

    // update plotting variables
    _controller_data->ff_setpoint = cmd_ff;
    _controller_data->filtered_setpoint = squelched_propulsive_term;

    // Print the propulsive terms for the right leg
    if (!_leg_data->is_left)
    {
        Serial.print("Vel:" + String(_leg_data->ankle.joint_velocity) + "\t");
        Serial.print("kProp:" + String(kProp) + "\t");
        Serial.print("SQPM:" + String(propulsive_grf_squelch_multiplier) + "\t");
        Serial.print("SQP:" + String(squelched_propulsive_term) + "\n");
        
    }

    #ifdef CONTROLLER_DEBUG
    logger::println("PropulsiveAssistive::calc_motor_cmd : stop");
    #endif
    return cmd;
}

//****************************************************


ProportionalJointMoment::ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("ProportionalJointMoment::Constructor");
    #endif

    _stance_thresholds_left.first = exo_data->left_leg.toe_fsr_lower_threshold;
    _stance_thresholds_left.second = exo_data->left_leg.toe_fsr_upper_threshold;
    _stance_thresholds_right.first = exo_data->right_leg.toe_fsr_lower_threshold;
    _stance_thresholds_right.second = exo_data->right_leg.toe_fsr_upper_threshold;
}

float ProportionalJointMoment::calc_motor_cmd()
{
	Serial.print("\nRunning PJMC...");
    #ifdef CONTROLLER_DEBUG
        logger::println("ProportionalJointMoment::calc_motor_cmd : start");
    #endif

    float cmd_ff = 0;
    // don't calculate command when fsr is calibrating.
    if (!_leg_data->do_calibration_toe_fsr)
    {
        

        // calculate the feed forward command
        if (_leg_data->toe_stance) 
        {
            // scale the fsr values so the controller outputs zero feed forward when the FSR value is at the threshold
            float threshold = _leg_data->toe_fsr_upper_threshold;
            float scaling = threshold/(1-threshold);
            float correction = scaling-(scaling*_leg_data->toe_fsr);

            // saturate the fsr value
            float fsr = min(_leg_data->toe_fsr, 1.2);

            // calculate the feed forward command
            cmd_ff = (fsr) * _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx];          
            
            // saturate and account for assistance
            cmd_ff = max(0, cmd_ff);
            //cmd_ff = min(max(0, cmd_ff), _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx]);
            cmd_ff = -1 * cmd_ff * (_controller_data->parameters[controller_defs::proportional_joint_moment::is_assitance_idx] ? 1 : -1);
        }
        else
        {
            cmd_ff = _controller_data->parameters[controller_defs::proportional_joint_moment::swing_max_idx];
        }
    }

    // low pass filter on torque_reading
    // const float torque = _joint_data->torque_reading * (_leg_data->is_left ? 1 : -1);
	const float torque = _joint_data->torque_reading;
    const float alpha = (_controller_data->parameters[controller_defs::proportional_joint_moment::torque_alpha_idx] != 0) ? 
            _controller_data->parameters[controller_defs::proportional_joint_moment::torque_alpha_idx] : 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, 
            _controller_data->filtered_torque_reading, alpha);

    // find max measured and max setpoint during stance
    if (_leg_data->toe_stance) 
    {
       /*  const float new_torque = _controller_data->filtered_torque_reading < 0 ? _controller_data->filtered_torque_reading*(-1) : _controller_data->filtered_torque_reading;
        const float new_ff = cmd_ff < 0 ? cmd_ff*(-1) : cmd_ff; */
		const float new_torque = _controller_data->filtered_torque_reading;
		const float new_ff = cmd_ff;
		

        _controller_data->max_measured = (_controller_data->max_measured < new_torque) ? new_torque : _controller_data->max_measured;
        //_controller_data->max_measured = max(abs(_controller_data->max_measured), abs(_controller_data->filtered_torque_reading));
        _controller_data->max_setpoint = (_controller_data->max_setpoint < new_ff) ? new_ff : _controller_data->max_setpoint;
        //_controller_data->max_setpoint = max(abs(_controller_data->max_setpoint), abs(cmd_ff));
    }

    // Set previous max values on rising edge
    if (_leg_data->ground_strike)
    {
        _controller_data->prev_max_measured = _controller_data->max_measured;
        _controller_data->prev_max_setpoint = _controller_data->max_setpoint;
        _controller_data->max_measured = 0;
        _controller_data->max_setpoint = 0;

        // Calculate this steps Kf
        if ((_controller_data->prev_max_measured > 0.0f) && (_controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx] != 0.0f))
        {
            //leg->KF = leg->KF - (leg->Max_Measured_Torque - (leg->MaxPropSetpoint)) / (leg->MaxPropSetpoint)*0.6;
            _controller_data->kf = _controller_data->kf + ((_controller_data->prev_max_setpoint/_controller_data->prev_max_measured) - 1);
            // Constrain kf
            _controller_data->kf = min(1.25, _controller_data->kf);
            _controller_data->kf = max(0.75, _controller_data->kf);
        }
    }

    _controller_data->filtered_setpoint = utils::ewma(cmd_ff, _controller_data->filtered_setpoint, 0.1);
    _controller_data->ff_setpoint = _controller_data->filtered_setpoint;

    // add the PID contribution to the feed forward command
    float cmd = 0;
    float kf_cmd = (_leg_data->toe_stance) ? (_controller_data->kf * _controller_data->filtered_setpoint) : _controller_data->filtered_setpoint;
    // if (!_leg_data->is_left)
    // {
    //     kf_cmd *= -1;
    // }

    if (_controller_data->parameters[controller_defs::proportional_joint_moment::use_pid_idx])
    {

		cmd = _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::proportional_joint_moment::p_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::i_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::d_gain_idx]);

	}
    else
    {
        cmd = cmd_ff;
    }
    
    _controller_data->filtered_cmd = utils::ewma(cmd, _controller_data->filtered_cmd, 1);

    // print controller parameters
    // static float cnt = 0;
    // cnt++;
    // if (cnt > 1000)
    // {
    //     Serial.print("ID:" + String(_leg_data->is_left) + "\t");
    //     Serial.print("CID:"+String(_controller_data->controller)+"\t");
    //     for (int i=0; i<controller_defs::propulsive_assistive::num_parameter; i++)
    //     {
    //         Serial.print(String(i) + ":" + String(_controller_data->parameters[i]) + "\t");
    //     }
    //     Serial.print("\n");
    //     cnt = 0;
    // }

    #ifdef CONTROLLER_DEBUG
        logger::println("ProportionalJointMoment::calc_motor_cmd : end");
    #endif
	/* if (_joint_data->is_left) {
		Serial.print("\nLeft cmd: ");
		Serial.print(_controller_data->filtered_cmd);
	}
	else {
		Serial.print("Right cmd: ");
		Serial.print(_controller_data->filtered_cmd);
	} */
	if (!_leg_data->do_calibration_toe_fsr) {
    return _controller_data->filtered_cmd;
	}
	else {
		return 0;
	}
}


//****************************************************


HeelToe::HeelToe(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("HeelToe::Constructor");
    #endif

        /**< Initializes variables to 0 upon startup. */
        fs_previous = 0;                
        state = 0; 
        prev_state = 0;
        state_4_start = 0;              
        previous_state_4_duration;      
        swing_start = 0;                
        swing_duration = 0;   
        state_4_start_cmd = -5;
        prev_percent_gait = 0;
        x1 = 0;
        y1 = 0;
        x2 = 0;
        y2 = 0;
        x3 = 0;
        y3 = 0;
        A = 0;
        B = 0;
        C = 0;
}

float HeelToe::calc_motor_cmd()
{
    cmd_ff = 0;                                     /**< Sets the default motor command to 0. */

    percent_gait = _leg_data->percent_gait;         /**< Records what percentage of the gait cycle we are currently in. Function to estimate this can be found in Leg.cpp */

    float extension_torque = -1 * _controller_data->parameters[controller_defs::heel_toe::extension_torque_setpoint_idx]; /**< Extension Torque Setpoint, Multiplied by -1 to orient curve correctly. */
    float flexion_torque = -1 * _controller_data->parameters[controller_defs::heel_toe::flexion_torque_setpoint_idx];     /**< Flexion Torque Setpoint, Multiplied by -1 to orient the curve correctly. */

    if (percent_gait == 0 && prev_percent_gait == 100)                               /**< If we are in a new gait cycle, reset state_4_start, state_4_end, and swing_start to zero. */
    {
        state_4_start = 0;
        state_4_end = 0;
        swing_start = 0;
    }

    if (_leg_data->heel_stance == 0 && _leg_data->toe_stance == 0 && (_leg_data->prev_heel_stance == 1 || _leg_data->prev_toe_stance == 1)) /**< If neither the foot or toe is on the ground but one of them was in the previous iteration, mark the start of swing phase in terms of percent gait.  */
    {
        swing_start = percent_gait;
    }

    if (swing_start != 0)                       /**< If swing has started, the length of swing is 100% of the gait cylce minus the percentage of gait where swing started. */
    {
        swing_duration = 100 - swing_start;
    }

    m = ((0.6 * extension_torque) + (0.5 * flexion_torque)) / (0.7 * swing_duration);

    if (!_leg_data->do_calibration_toe_fsr && !_leg_data->do_calibration_heel_fsr)  /**< If the toe and heel FSRs are being calibrated then keep motor torque 0. */
    {

        fs = _leg_data->heel_fsr - (_leg_data->toe_fsr * 0.25);     /**<  Estimate of ground reaction force based on 'Bishe 2021', Goal is to create a variable that fluctuates between -1 and 1 based on users FSR readiongs. */

        /**< Basic filtering to not allow for wonky fs data. */
        if (fs > 1)
        {
            fs = 1;
        }

        if (fs < -1)
        {
            fs = -1;
        }

        /**< State Machine to determine which state, and thus which motor command, is appropriate. */
        if (fs > 0 && fs > fs_previous)             /**< If fs is positive and its derivative is positive, we are in state 1. */
        {
            state = 1;
        }
        else if (fs > 0 && fs < fs_previous)        /**< If fs is positive and its derivative is negative, we are in state 2. */
        {
            state = 2;
        }
        else if (fs < 0 && fs < fs_previous)        /**< If fs is negative and its derivative is negative, we are in state 3. */
        {
            state = 3;

        }
        else if (fs < 0 && fs > fs_previous || swing_start > 0 && (percent_gait - swing_start) < 0.3 * swing_duration) /**< If fs is negative and its derviative is postive or if we are in swing and less than 30% of the swing duration, we are in state 4. */
        {
            state = 4;

            if (state_4_start == 0)                                     /**< If this is the first time in state 4, record the current percent_gait as the starting point and mark the starting torque command for the parabola as the last command. */
            {
                state_4_start = percent_gait;
                state_4_start_cmd = prev_cmd;
            }
        }
        else if (percent_gait - swing_start >= 0.3 * swing_duration)    /**< If we are in swing and greater than 30% of the swing duration then we are in state 5. */
        {
            state = 5;

            if (state_4_end == 0)                                       /**< If this is the first time in state 5, record the previous percent_gait as the end of state 4 and calculate the duration of state 4. */
            {
                state_4_end = percent_gait - 1;
                previous_state_4_duration = state_4_end - state_4_start;
            }
        }
        else                                                            /**< If none of the above conditions are met, then state is set to 0. */
        {
            state = 0;
        }

        /**< Prevention of controller from going back down to a previous state unless the state is switching from state 5 to state 1. Essentially acts as a state-filter. */
        if (state < prev_state)
        {
            if (prev_state - state < 4)
            {
                state = prev_state;
            }
        }

        logger::print("HeelToe::calc_motor_cmd : State : ");
        logger::print(state);
        logger::print("\n");

        /**< Setup for Parabola used in State 4 */
        if (state_4_start != 0 && previous_state_4_duration != 0)       /**< If state_4_start and previous_state_4_duration both have non-zero values, assign values to coordinates. */
        {
            x1 = state_4_start;
            if (state_4_start_cmd == -5)
            {
                y1 = - 0.5 * flexion_torque;
            }
            else
            {
                y1 = state_4_start_cmd;
            }

            x2 = state_4_start + (previous_state_4_duration / 2);
            y2 = - 1* flexion_torque;

            x3 = state_4_start + previous_state_4_duration;
            y3 = - 0.5 * flexion_torque;
        }

        /**< Takes coordinates of known points and calculates constants involved in parabolic equation. */
        B = ((y2 - y3) * ((x2 * x2) - (x1 * x1)) - (y2 - y1) * ((x2 * x2) - (x3 * x3))) / (((x2 - x3) * ((x2 * x2) - (x1*x1))) - ((x2 - x1) * ((x2*x2) - (x3*x3))));
        A = ((y2 - y1) - (B * (x2 - x1))) / ((x2*x2) - (x1*x1));
        C = y1 - (A * (x1*x1)) - (B * x1);

        /**< Caculates motor command based on current state. */
        if (state == 1)
        {
            //cmd_ff = 0;
            cmd_ff = (0.6 + (fs * 0.4)) * extension_torque;
        }
        else if (state == 2)
        {
            //cmd_ff = 0;
            cmd_ff = fs * extension_torque;
        }
        else if (state == 3)
        {
            //cmd_ff = 0;
            cmd_ff = flexion_torque * 4 * fs * 0.5;
        }
        else if (state == 4)
        {
            //cmd_ff = 0;
            cmd_ff = ((A * (percent_gait*percent_gait)) + (B * percent_gait) + C);            /**< Formula for a parabola using constants calculated above. Multiplied by -1 to flip into correct direction. */
        }
        else if (state == 5)
        {
            //cmd_ff = 0;
            cmd_ff = ((percent_gait - (state_4_start + previous_state_4_duration)) * m) - (0.5 * flexion_torque);
        }
        else
        {
            cmd_ff = 0;
        }
        
    }

    logger::print("HeelToe::calc_motor_cmd : y1 : ");
    logger::print(y1);
    logger::print("\n");

    prev_state = state;
    fs_previous = fs;
    prev_cmd = cmd_ff;
    prev_percent_gait = percent_gait;

    float cmd = cmd_ff; /**< Motor command in terminology used across controllers.  */

    _controller_data->filtered_cmd = utils::ewma(cmd, _controller_data->filtered_cmd, 1);
                         
    return cmd;
};

//****************************************************


ExtensionAngle::ExtensionAngle(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("ExtensionAngle::Constructor");
    #endif
    _state = 0; // extension mode originally 
    
    _reset_angles();
    
    
}

float ExtensionAngle::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::extension_angle::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.position;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);
    
    
    float normalized_angle = 0;
    // calculate the normalized angle
    if (angle >= 0)
    {
        normalized_angle = angle / _max_angle;
    }
    else
    {
        normalized_angle = angle / _min_angle;
        
    }

    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
    //     logger::print(utils::radians_to_degrees(angle));
    //     logger::print(" ");
    //     logger::print(utils::radians_to_degrees(_max_angle));
    //     logger::print(" ");
    //     logger::print(utils::radians_to_degrees(_min_angle));
    //     logger::print(" ");
    //     logger::print(100);
    //     logger::print(" ");
    //     logger::print(-100);
    //     logger::print(" ");
    //     logger::print(normalized_angle*100);
    //     logger::print("\n");
    // }
    
    _update_state(angle);
    
    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd_ff = _controller_data->parameters[controller_defs::extension_angle::extension_setpoint_idx] * normalized_angle;
            break;
        case 1 :  // flexion
            cmd_ff = _controller_data->parameters[controller_defs::extension_angle::flexion_setpoint_idx];
            break;
    }
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::extension_angle::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::extension_angle::p_gain_idx], _controller_data->parameters[controller_defs::extension_angle::i_gain_idx], _controller_data->parameters[controller_defs::extension_angle::d_gain_idx]) 
                : 0);
    
    return cmd;
}


void ExtensionAngle::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
}


void ExtensionAngle::_update_state(float angle)
{
    switch (_state)
    {
        case 0 :  // extension assistance
            if (angle <= _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx])           //If the angle of the hip is less than 5 degrees (currently what angle_threshold is set to), then switch to flexion assistance
            {
                _state = 1;
            }
            break;
        case 1 :  // flexion assistance 
            
            if ((angle > (_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle / 100))           //If the angle exceeds 80% (currently what flexion_percent_max is set to) of the max angle, switch to extension angle
                || ((angle > _controller_data->parameters[controller_defs::extension_angle::angle_threshold_idx]+utils::degrees_to_radians(5))          //Or if the angle is greater than 10 degrees (angle_threshold currently set to 5 degrees), switch to extension 
                    && (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::extension_angle::velocity_threshold_idx])))            //or if the angular velocity is less than or equal to -0.175rad/s (currenlty what velocity_threshold is set to), switch to extension 
            {
                _state = 0;
            }
            break;
         
        
    }
    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
        // logger::print(angle);
        // logger::print(" ");
        // logger::print(_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle/100);
        // logger::print("\n");
    // }
}


//****************************************************


BangBang::BangBang(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("BangBang::Constructor");
    #endif
    _state = 0; // extension mode originally 
    
    _reset_angles();
    
}

float BangBang::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::bang_bang::clear_angle_idx])
    {
        _reset_angles();
    }
    
    float angle = _leg_data->hip.position;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);
    
    
    // float normalized_angle = 0;
    // // calculate the normalized angle
    // if (angle >= 0)
    // {
        // normalized_angle = angle / _max_angle;
    // }
    // else
    // {
        // normalized_angle = angle / _min_angle;
        
    // }

    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
    //     logger::print(utils::radians_to_degrees(angle));
    //     logger::print(" ");
    //     logger::print(utils::radians_to_degrees(_max_angle));
    //     logger::print(" ");
    //     logger::print(utils::radians_to_degrees(_min_angle));
    //     logger::print(" ");
    //     logger::print(100);
    //     logger::print(" ");
    //     logger::print(-100);
    //     logger::print(" ");
    //     logger::print(normalized_angle*100);
    //     logger::print("\n");
    // }
    
    _update_state(angle);
    
    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
        case 0 :  // extension
            cmd_ff = _controller_data->parameters[controller_defs::bang_bang::extension_setpoint_idx];
            break;
        case 1 :  // flexion
            cmd_ff = _controller_data->parameters[controller_defs::bang_bang::flexion_setpoint_idx];
            break;
    }
    
    
    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::bang_bang::use_pid_idx] 
                ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::bang_bang::p_gain_idx], _controller_data->parameters[controller_defs::bang_bang::i_gain_idx], _controller_data->parameters[controller_defs::bang_bang::d_gain_idx]) 
                : 0);
    
    return ((_controller_data->parameters[controller_defs::bang_bang::is_assitance_idx] ? 1:-1) * cmd);
}


void BangBang::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
}



void BangBang::_update_state(float angle)
{
    switch (_state)
    {
    case 0:  // extension assistance
        if (angle <= _controller_data->parameters[controller_defs::bang_bang::angle_threshold_idx])
        {
            _state = 1;
        }
        break;
    case 1:  // flexion assistance 

        if ((angle > (_controller_data->parameters[controller_defs::bang_bang::target_flexion_percent_max_idx] * _max_angle / 100))
            || ((angle > _controller_data->parameters[controller_defs::bang_bang::angle_threshold_idx] + utils::degrees_to_radians(5))
                && (_leg_data->hip.velocity <= _controller_data->parameters[controller_defs::bang_bang::velocity_threshold_idx])))
        {
            _state = 0;
        }
        break;


    }
    // int print_time_ms = 100;
    // static int last_timestamp = millis();
    // int timestamp = millis();
    // if ((timestamp-last_timestamp)>print_time_ms)
    // {
        // logger::print(angle);
        // logger::print(" ");
        // logger::print(_controller_data->parameters[controller_defs::extension_angle::target_flexion_percent_max_idx] * _max_angle/100);
        // logger::print("\n");
    // }
}


//****************************************************


LateStance::LateStance(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    logger::println("LateStance::Constructor");
#endif
    _state = 0; // extension mode originally 

    _reset_angles();

}

float LateStance::calc_motor_cmd()
{
    // check if the angle range should be reset
    if (_controller_data->parameters[controller_defs::late_stance::clear_angle_idx])
    {
        _reset_angles();
    }

    float angle = _leg_data->hip.position;
    // check the angle range
    _max_angle = max(angle, _max_angle);
    _min_angle = min(angle, _min_angle);

    _update_state(angle);

    float cmd_ff = 0;
    // calculate torque based on state
    switch (_state)
    {
    case 0:  // extension
        cmd_ff = 0;
        break;
    case 1:  // flexion
        cmd_ff = _controller_data->parameters[controller_defs::late_stance::resistance_setpoint_idx];
        break;
    }


    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::late_stance::use_pid_idx]
        ? _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::late_stance::p_gain_idx], _controller_data->parameters[controller_defs::late_stance::i_gain_idx], _controller_data->parameters[controller_defs::late_stance::d_gain_idx])
        : 0);

    return cmd;
}

/*
 * Used to reset the range of motion to the starting values.
 * There is no reset for the flag so the user must turn this off manually.
 */
void LateStance::_reset_angles()
{
    _max_angle = _initial_max_angle;
    _min_angle = _initial_min_angle;
}


/*
 *
 */
void LateStance::_update_state(float angle)
{
    switch (_state)
    {
    case 0:  // No Torque 
        if (angle <= _controller_data->parameters[controller_defs::late_stance::angle_on_off])
        {
            _state = 1;
        }
        break;
    case 1:  // Torque

        if ((angle > (_controller_data->parameters[controller_defs::late_stance::angle_on_off] * _max_angle / 100)))
        {
            _state = 0;
        }
        break;


    }
}


//****************************************************


GaitPhase::GaitPhase(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    logger::println("GaitPhase::Constructor");
#endif
}

float GaitPhase::calc_motor_cmd()
{

    const int flexion_torque = _controller_data->parameters[controller_defs::gait_phase::flexion_setpoint_idx];            //Sets the magnitude of the flexion torque applied
    const int extension_torque = _controller_data->parameters[controller_defs::gait_phase::extension_setpoint_idx];        //Sets the mangitude of the extnesion torque applied
    
    if (flexion_torque > 0 && extension_torque < 0)
    {
        slope = _controller_data->parameters[controller_defs::gait_phase::slope_idx];                                        //Sets the slope of the line connnecting transition points from the config file
    }
    else if(flexion_torque < 0 && extension_torque > 0)
    {
        slope = - 1 * _controller_data->parameters[controller_defs::gait_phase::slope_idx];
    }
    else
    {
        slope = _controller_data->parameters[controller_defs::gait_phase::slope_idx];
    }
    float width = (flexion_torque - extension_torque) / slope;                                                                  //Calculates the width of the region during which the transition in torque magnitudes occurs
    float half_width = width / 2;                                                                                               //Calculates half the width of the region during which the transition in torque magnitudes occurs

    const int flexion_start = _controller_data->parameters[controller_defs::gait_phase::flexion_start_percentage_idx];      //Sets the starting point of when flexion should be applied
    const int flexion_end = _controller_data->parameters[controller_defs::gait_phase::flexion_end_percentage_idx];          //Sets the ending point of when flexion should be applied
    const int extension_start = _controller_data->parameters[controller_defs::gait_phase::extension_start_percentage_idx];  //Sets the starting point of when extension should be applied     
    const int extension_end = _controller_data->parameters[controller_defs::gait_phase::extension_end_percentage_idx];      //Sets the ending point of when flexion should be applied

    // Initializes torque
    float cmd_ff = 0;

    // Defines % Gait Variable (obtains from Leg.cpp & LegData.cpp)
    float percent_gait = _leg_data->percent_gait;

    //Print outs if you need to debug Controller parameter issue

        //logger::print("GaitPhase::calc_motor_cmd : Flexion_Start_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::gait_phase::flexion_start_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Flexion_End_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::gait_phase::flexion_end_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Extension_Start_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::gait_phase::extension_start_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Extension_End_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::gait_phase::extension_end_percentage_idx]);
        //logger::print("\n");


    if (-1 != percent_gait) //Only runs if a valid calculation of percent gait is present
    {
              

        //If there is no zero torque range between the starting of flexion and extension
        if (extension_end == flexion_start && flexion_end == extension_start)
        {
            state = 0.5;

            if (percent_gait == 0 || percent_gait < extension_end - half_width)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Early : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if (percent_gait > extension_end - half_width && percent_gait < flexion_start + half_width)
            {
                cmd_ff = extension_torque + slope * (percent_gait - extension_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Extension to Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if (percent_gait >= flexion_start + half_width && percent_gait <= flexion_end - half_width)
            {
                cmd_ff = flexion_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if (percent_gait > flexion_end - half_width && percent_gait < flexion_end + half_width)
            {
                cmd_ff = flexion_torque - slope * (percent_gait - flexion_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Flexion to Extension : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if (percent_gait >= extension_start + half_width && percent_gait <= 100)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Late : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Unknown Condition : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }

        }
        else if(extension_end != flexion_start && flexion_end != extension_start)
        {
            state = 1;

            if (percent_gait == 0 || percent_gait <= extension_end - half_width)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Early : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > extension_end - half_width && percent_gait < extension_end + half_width)
            {
                cmd_ff = extension_torque + (slope / 2) * (percent_gait - extension_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Extension to Zero Torque : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= extension_end + half_width && percent_gait <= flexion_start - half_width)
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Zero Torque - Early : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > flexion_start - half_width && percent_gait < flexion_start + half_width)
            {
                cmd_ff = 0 + (slope / 2) * (percent_gait - flexion_start + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Zero Torque to Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= flexion_start + half_width && percent_gait <= flexion_end - half_width)
            {
                cmd_ff = flexion_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > flexion_end - half_width && percent_gait < flexion_end + half_width)
            {
                cmd_ff = flexion_torque - (slope / 2) * (percent_gait - flexion_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Flexion to Zero Torque : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= flexion_end + half_width && percent_gait <= extension_start - half_width)
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Zero Torque - Late : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > extension_start - half_width && percent_gait < extension_start + half_width)
            {
                cmd_ff = 0 - (slope / 2) * (percent_gait - extension_start + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Zero Torque to Extension : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= extension_start + half_width && percent_gait <= 100)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Late : ");
                //logger::print(percent_gait);
                //Serail.print("\n");
            }
            else
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Unknown Condition : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }

        }
        else if(extension_end != flexion_start && flexion_end == extension_start)
        {
            state = 1.5;

            if (percent_gait == 0 || percent_gait <= extension_end - half_width)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Early : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > extension_end - half_width && percent_gait < extension_end + half_width)
            {
                cmd_ff = extension_torque + (slope / 2) * (percent_gait - extension_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Extension to Zero Torque : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= extension_end + half_width && percent_gait <= flexion_start - half_width)
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Zero Torque : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > flexion_start - half_width && percent_gait < flexion_start + half_width)
            {
                cmd_ff = 0 + (slope / 2) * (percent_gait - flexion_start + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Zero Torque to Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= flexion_start + half_width && percent_gait <= flexion_end - half_width)
            {
                cmd_ff = flexion_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > flexion_end - half_width && percent_gait < flexion_end + half_width)
            {
                cmd_ff = flexion_torque - slope * (percent_gait - flexion_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Flexion to Extension : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= extension_start + half_width && percent_gait <= 100)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Late : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Unknown Condition : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }

        }
        else if(extension_end == flexion_start && flexion_end != extension_start)
        {
            state = 2;

            if (percent_gait == 0 || percent_gait < extension_end)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Early : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > extension_end - half_width && percent_gait < flexion_start + half_width)
            {
                cmd_ff = extension_torque + slope * (percent_gait - extension_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Extension to Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= flexion_start + half_width && percent_gait <= flexion_end - half_width)
            {
                cmd_ff = flexion_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Flexion : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > flexion_end - half_width && percent_gait < flexion_end + half_width)
            {
                cmd_ff = flexion_torque - (slope / 2) * (percent_gait - flexion_end + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Flexion to Zero Torque : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= flexion_end + half_width && percent_gait <= extension_start - half_width)
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Zero Torque - Late : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait > extension_start - half_width && percent_gait < extension_start + half_width)
            {
                cmd_ff = 0 - (slope / 2) * (percent_gait - extension_start + half_width);
                //logger::print("GaitPhase::calc_motor_cmd : Transition - Zero Torque to Extension : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else if(percent_gait >= extension_start + half_width && percent_gait <= 100)
            {
                cmd_ff = extension_torque;
                //logger::print("GaitPhase::calc_motor_cmd : Extension - Late : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }
            else
            {
                cmd_ff = 0;
                //logger::print("GaitPhase::calc_motor_cmd : Unknown Condition : ");
                //logger::print(percent_gait);
                //logger::print("\n");
            }

        }
        else
        {
            state = 0;
            cmd_ff = 0;
        }

    }

    if (cmd_ff > flexion_torque)
    {
        cmd_ff = flexion_torque;
    }
    else if (cmd_ff < extension_torque)
    {
        cmd_ff = extension_torque;
    }
    else
    {
        cmd_ff = cmd_ff;
    }


    // Incorporates PID control if flag is present
    float cmd = cmd_ff;
    if (_controller_data->parameters[controller_defs::gait_phase::use_pid_idx])
    {
        cmd = _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::gait_phase::p_gain_idx], _controller_data->parameters[controller_defs::gait_phase::i_gain_idx], _controller_data->parameters[controller_defs::gait_phase::d_gain_idx]);
    }
    else
    {
        cmd = cmd_ff;
    }

    return cmd;
}


//****************************************************


Parabolic::Parabolic(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    logger::println("Parabolic::Constructor");
#endif
}

float Parabolic::calc_motor_cmd()
{

    const int flexion_torque = _controller_data->parameters[controller_defs::parabolic::flexion_setpoint_idx];            //Sets the magnitude of the flexion torque applied
    const int extension_torque = _controller_data->parameters[controller_defs::parabolic::extension_setpoint_idx];        //Sets the mangitude of the extnesion torque applied

    const int flexion_start = _controller_data->parameters[controller_defs::parabolic::flexion_start_percentage_idx];      //Sets the starting point of when flexion should be applied
    const int flexion_end = _controller_data->parameters[controller_defs::parabolic::flexion_end_percentage_idx];          //Sets the ending point of when flexion should be applied
    const int extension_start = _controller_data->parameters[controller_defs::parabolic::extension_start_percentage_idx];  //Sets the starting point of when extension should be applied     
    const int extension_end = _controller_data->parameters[controller_defs::parabolic::extension_end_percentage_idx];      //Sets the ending point of when flexion should be applied

    // Initializes torque
    float cmd_ff = 0;

    // Defines % Gait Variable (obtains from Leg.cpp & LegData.cpp)
    float percent_gait = _leg_data->percent_gait;

    //Print outs if you need to debug Controller parameter issue

        //logger::print("GaitPhase::calc_motor_cmd : Flexion_Start_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::parabolic::flexion_start_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Flexion_End_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::parabolic::flexion_end_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Extension_Start_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::parabolic::extension_start_percentage_idx]);
        //logger::print("\n");

        //logger::print("GaitPhase::calc_motor_cmd : Extension_End_Percentage : ");
        //logger::print(_controller_data->parameters[controller_defs::parabolic::extension_end_percentage_idx]);
        //logger::print("\n");


    if (-1 != percent_gait) //Only runs if a valid calculation of percent gait is present
    {

        //Coordinates for the extension curve
        float x1 = extension_start;
        float y1 = 0;

        float x2 = extension_start + ((extension_end - extension_start)/2);
        float y2 = extension_torque;

        float x3 = extension_end;
        float y3 = 0;

        //Constants for the extension curve
        float B_one = ((y2 - y3) * ((x2 * x2) - (x1 * x1)) - (y2 - y1) * ((x2 * x2) - (x3 * x3))) / (((x2 - x3) * ((x2 * x2) - (x1 * x1))) - ((x2 - x1) * ((x2 * x2) - (x3 * x3))));
        float A1 = ((y2 - y1) - (B_one * (x2 - x1))) / ((x2 * x2) - (x1 * x1));
        float C1 = y1 - (A1 * (x1 * x1)) - (B_one * x1);

        //Coordinates for the flexion curve
        float x4 = flexion_start;
        float y4 = 0;

        float x5 = flexion_start + ((flexion_end - flexion_start) / 2);
        float y5 = flexion_torque;

        float x6 = flexion_end;
        float y6 = 0;

        //Constants for the flexion curve
        float B2 = ((y5 - y6) * ((x5 * x5) - (x4 * x4)) - (y5 - y4) * ((x5 * x5) - (x6 * x6))) / (((x5 - x6) * ((x5 * x5) - (x4 * x4))) - ((x5 - x4) * ((x5 * x5) - (x6 * x6))));
        float A2 = ((y5 - y4) - (B2 * (x5 - x4))) / ((x5 * x5) - (x4 * x4));
        float C2 = y1 - (A2 * (x4 * x4)) - (B2 * x4);


        //Determination of torque
        if (percent_gait >= extension_start && percent_gait <= extension_end)
        {
            cmd_ff = ((A1 * (percent_gait * percent_gait)) + (B_one * percent_gait) + C1);
        }
        else if (percent_gait >= flexion_start && percent_gait <= flexion_end)
        {
            cmd_ff = ((A2 * (percent_gait * percent_gait)) + (B2 * percent_gait) + C2);
        }
        else
        {
            cmd_ff = 0;
        }
    }

    float cmd = cmd_ff;

    return cmd;
}

//****************************************************


/*
 * Constructor for the controller
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer.
 */
ZhangCollins::ZhangCollins(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("ZhangCollins::Constructor");
    #endif

        torque_cmd = 0;
		cmd = 0;

};

float ZhangCollins::calc_motor_cmd()
{
    
    //Define the variables
    float percent_gait = _leg_data->percent_stance;
/* 	_controller_data->ptb_iStep++;
	_controller_data->ptb_iiStep++;
	if (_controller_data->ptb_iStep > 100) {
		_controller_data->ptb_iStep = 0;
	}
	float percent_gait = _controller_data->ptb_iStep; */
			

    //Pull in user defined parameter values
    float peak_torque_Nm = _controller_data->parameters[controller_defs::zhang_collins::torque_idx];
    float peak_time = _controller_data->parameters[controller_defs::zhang_collins::peak_time_idx];
    float rise_time = _controller_data->parameters[controller_defs::zhang_collins::rise_time_idx];
    float fall_time = _controller_data->parameters[controller_defs::zhang_collins::fall_time_idx];

    //Calculates Nodes for Spline Generation
    float node1 = peak_time - rise_time;
    float node2 = peak_time;
    float node3 = peak_time + fall_time;

    //Calculates Torque Command
    torque_cmd = -1* _spline_generation(node1, node2, node3, peak_torque_Nm, percent_gait);
    _controller_data->ff_setpoint = torque_cmd;
    
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);

    //Low Pass Filter for Torque Sensor
/*     if (_joint_data->is_left == 0)
    {
        _controller_data->filtered_torque_reading = -1 * _controller_data->filtered_torque_reading;
    } */


    //PID Control
	if (_controller_data->parameters[controller_defs::zhang_collins::use_pid_idx])
	{
		cmd = _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::zhang_collins::p_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::i_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::d_gain_idx]);
/* 		if (_joint_data->is_left) {
			cmd = _pid(torque_cmd, -1*_controller_data->PIDMLTPLR * _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::zhang_collins::p_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::i_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::d_gain_idx]);
		} */
	}
	else
	{
		cmd = torque_cmd;
	}
	if (cmd != cmd) {
		// cmd = 0;
		cmd = (_controller_data->previous_cmd)/2;
	}
/* 	if ((_controller_data->ptb_iiStep < 202) && (_joint_data->is_left)) {
	Serial.print("\n Percent gait: ");
	Serial.print(percent_gait);
	Serial.print("  |  Left leg cmd: ");
	Serial.print(cmd);
	} */
	_controller_data->previous_cmd = cmd;
	
	/* if (_joint_data->is_left) {
		Serial.print("\nLeft cmd: ");
		Serial.print(cmd);
		Serial.print("Left torque_cmd: ");
		Serial.print(torque_cmd);
		Serial.print("  |  Left torque: ");
		Serial.print(_controller_data->filtered_torque_reading);
	}
	else {
		Serial.print("Right cmd: ");
		Serial.print(cmd);
		Serial.print("Right torque_cmd: ");
		Serial.print(torque_cmd);
		Serial.print("  |  Right torque: ");
		Serial.print(_controller_data->filtered_torque_reading);
	} */
	
    return cmd;
};

float ZhangCollins::_spline_generation(float node1, float node2, float node3, float torque_magnitude, float percent_gait)
{
    float u;

    float x[3] = { node1, node2, node3 };
    float y[3] = { 0, torque_magnitude, 0 };

    float h[2] = { (x[1] - x[0]), (x[2] - x[1]) };
    float delta[2] = { ((y[1] - y[0]) / h[0]), ((y[2] - y[1]) / h[1]) };

    float dy[3] = { 0, 0, ((3 * delta[1]) / 2) };

    if (percent_gait < x[0] || percent_gait > x[2])
    {
        u = 0;
    }
    else
    {
        int k = -1;
        if (percent_gait >= x[0] && percent_gait < (x[1]))
        {
            k = 0;
        }
        else if (percent_gait >= x[1] && percent_gait < x[2])
        {
            k = 1;
        }

        float a = delta[k];
        float b = (a - dy[k]) / h[k];
        float c = (dy[k + 1] - a) / h[k];
        float d = (c - b) / h[k];

        u = y[k] + (percent_gait - x[k]) * (dy[k] + (percent_gait - x[k]) * (b + (percent_gait - x[k + 1]) * d));
    }

    return u;
};


//****************************************************


FranksCollinsHip::FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("FranksCollinsHip::Constructor");
    #endif

    last_percent_gait = -1;
    last_start_time = -1;

}

float FranksCollinsHip::calc_motor_cmd()
{
    float start_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::start_percent_gait_idx];

    // based on the percent gait find the torque to apply.
    // convert to floats so we don't need to modify everywhere in the code.
    float percent_gait = _leg_data->percent_gait;
    float expected_duration = _leg_data->expected_step_duration;

    // calculate the offset percent gait so there isn't a hold at 100% regular if the step goes long or short.
    // restart when the percent gait crosses the _start_percent_gait
    if ((percent_gait >= start_percent_gait) && last_percent_gait < start_percent_gait)
    {
        last_start_time = millis();
    }

    // logger::print("Franks::calc_motor_cmd : _last_start_time = ");
    // logger::print(_last_start_time);
    // logger::print("\n");
    // logger::print("Franks::calc_motor_cmd : percent_gait = ");
    // logger::print(percent_gait);
    // logger::print("\n");
    // logger::print("Franks::calc_motor_cmd : _last_percent_gait = ");
    // logger::print(_last_percent_gait);
    // logger::print("\n");

    last_percent_gait = percent_gait;


    // return 0 torque till we hit the correct percent gait.
    if (last_start_time == -1)
    {
        return 0;
    }

    float shifted_percent_gait = (millis() - last_start_time) / expected_duration * 100;

    // logger::print("Franks::calc_motor_cmd : shifted_percent_gait = ");
    // logger::print(shifted_percent_gait);
    // logger::print("\n");

    float torque_cmd = 0;

    float mass = _controller_data->parameters[controller_defs::franks_collins_hip::mass_idx];
    float extension_torque_peak = _controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx];
    float flexion_torque_peak = _controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx];

    float extension_torque_magnitude_Nm = extension_torque_peak;
    float flexion_torque_magnitude_Nm = flexion_torque_peak;

    float mid_time_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx];
    float mid_duration_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx];

    float extension_peak_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx];
    float extension_rise_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx];
    float extension_fall_percent_gait = (mid_time_percent_gait - (mid_duration_percent_gait / 2)) - extension_peak_percent_gait;

    float flexion_peak_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx];
    float flexion_rise_percent_gait = flexion_peak_percent_gait - (mid_time_percent_gait + (mid_duration_percent_gait / 2));
    float flexion_fall_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx];

    //logger::print("Franks::calc_motor_cmd : flexion_peak_percent_gait = ");
    //logger::print(flexion_peak_percent_gait);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : flexion_rise_percent_gait = ");
    //logger::print(flexion_rise_percent_gait);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : flexion_fall_percent_gait = ");
    //logger::print(flexion_fall_percent_gait);
    //logger::print("\n");

    float extension_node1 = extension_peak_percent_gait - extension_rise_percent_gait;
    float extension_node2 = extension_peak_percent_gait;
    float extension_node3 = extension_peak_percent_gait + extension_fall_percent_gait;

    float flexion_node1 = flexion_peak_percent_gait - flexion_rise_percent_gait - (100 - start_percent_gait);
    float flexion_node2 = flexion_peak_percent_gait - (100 - start_percent_gait);
    float flexion_node3 = flexion_peak_percent_gait + flexion_fall_percent_gait - (100 - start_percent_gait); //(100 - flexion_fall_percent_gait - flexion_peak_percent_gait);

     //logger::print("Franks::calc_motor_cmd : node1 = ");
     //logger::print(flexion_node1);
     //logger::print("\n");

     //logger::print("Franks::calc_motor_cmd : node2 = ");
     //logger::print(flexion_node2);
     //logger::print("\n");

     //logger::print("Franks::calc_motor_cmd : node3 = ");
     //logger::print(flexion_node3);
     //logger::print("\n");

    torque_cmd = _spline_generation(extension_node1, extension_node2, extension_node3, extension_torque_magnitude_Nm, shifted_percent_gait) + _spline_generation(flexion_node1, flexion_node2, flexion_node3, flexion_torque_magnitude_Nm, percent_gait);

    //if (shifted_percent_gait >= extension_node1 && shifted_percent_gait <= extension_node3)
    //{
    //    torque_cmd = _spline_generation(extension_node1, extension_node2, extension_node3, extension_torque_magnitude_Nm, shifted_percent_gait);
    //}
    //else if (shifted_percent_gait >= flexion_node1 && shifted_percent_gait <= flexion_node3)
    //{
    //    torque_cmd = _spline_generation(flexion_node1, flexion_node2, flexion_node3, flexion_torque_magnitude_Nm, shifted_percent_gait);
    //}
    //else
    //{
    //    torque_cmd = 0;
    //}

    // add the PID contribution to the feed forward command
    float cmd = torque_cmd + (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx]
        ? _pid(torque_cmd, _joint_data->torque_reading, _controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::i_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx])
        : 0);

    return cmd;
}

float FranksCollinsHip::_spline_generation(float node1, float node2, float node3, float torque_magnitude, float shifted_percent_gait)
{
    float u;

    float x[3] = {node1, node2, node3};
    float y[3] = {0, torque_magnitude, 0};

    float h[2] = { (x[1] - x[0]), (x[2] - x[1]) };
    float delta[2] = { ((y[1] - y[0]) / h[0]), ((y[2] - y[1]) / h[1]) };

    float dy[3] = { 0, 0, ((3 * delta[1]) / 2) };

    if (shifted_percent_gait < x[0] || shifted_percent_gait > x[2])
    {
       u = 0;
    }
    else
    {
        int k = - 1;
        if (shifted_percent_gait >= x[0] && shifted_percent_gait < (x[1]))
        {
            k = 0;
        }
        else if (shifted_percent_gait >= x[1] && shifted_percent_gait < x[2])
        {
            k = 1;
        }

        float a = delta[k];
        float b = (a - dy[k]) / h[k];
        float c = (dy[k + 1] - a) / h[k];
        float d = (c - b) / h[k];

        u = y[k] + (shifted_percent_gait - x[k]) * (dy[k] + (shifted_percent_gait - x[k]) * (b + (shifted_percent_gait - x[k + 1]) * d));
    }

    return u;
}

//****************************************************
// 
// 
// UserDefined::UserDefined(config_defs::joint_id id, ExoData* exo_data)
// : _Controller(id, exo_data)
// {
    // #ifdef CONTROLLER_DEBUG
        // logger::println("UserDefined::Constructor");
    // #endif
    
    
    
    // for(int i = 0; i < controller_defs::user_defined::num_sample_points; i++) 
    // {
        // _percent_x[i] = i * _step_size;
    // } 
// }

// float UserDefined::calc_motor_cmd()
// {
    
    // int lower_idx = _leg_data->percent_gait/_step_size; //rounds down  
    // float cmd_ff = 0;
    // //  yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
    // if(lower_idx < controller_defs::user_defined::num_sample_points - 1)
    // {
        // cmd_ff = _controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx] + ((_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx+1]-_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx])/(_percent_x[lower_idx+1]-_percent_x[lower_idx])) * (_leg_data->percent_gait - _percent_x[lower_idx]);
    // } 
    // else
    // {
        // cmd_ff = _controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx] + ((_controller_data->parameters[0]-_controller_data->parameters[controller_defs::user_defined::curve_start_idx+lower_idx])/(100-_percent_x[lower_idx])) * (_leg_data->percent_gait - _percent_x[lower_idx]);
    // }
    // // add the PID contribution to the feed forward command
    // float cmd = cmd_ff + (_controller_data->parameters[controller_defs::user_defined::use_pid_idx] 
                // ? _pid(cmd_ff, _joint_data->torque_reading,_controller_data->parameters[controller_defs::user_defined::p_gain_idx], _controller_data->parameters[controller_defs::user_defined::i_gain_idx], _controller_data->parameters[controller_defs::user_defined::d_gain_idx]) 
                // : 0);
    
    // return 0;
// }


//****************************************************


Sine::Sine(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("Sine::Constructor");
    #endif
}

float Sine::calc_motor_cmd()
{
    //  converts period to int so % will work, but is only a float for convenience
    float cmd_ff = _controller_data->parameters[controller_defs::sine::amplitude_idx] * sin((millis() % (int)_controller_data->parameters[controller_defs::sine::period_idx]) / _controller_data->parameters[controller_defs::sine::period_idx] * 2 * M_PI + _controller_data->parameters[controller_defs::sine::phase_shift_idx]);

    // add the PID contribution to the feed forward command
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::sine::use_pid_idx]
        ? _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::sine::p_gain_idx], _controller_data->parameters[controller_defs::sine::i_gain_idx], _controller_data->parameters[controller_defs::sine::d_gain_idx])
        : 0);
    return cmd;
}


//****************************************************


Perturbation::Perturbation(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("Perturbation::Constructor");
    #endif
    
};

float Perturbation::calc_motor_cmd()
{

    float cmd = 0;     //Creates the cmd variable and initializes it to 0;

    float percent_gait = _leg_data->percent_gait;           //Calculates the percentage of the gait cycle

    if (percent_gait >= _controller_data->parameters[controller_defs::perturbation::threshold_start_idx] && percent_gait <= _controller_data->parameters[controller_defs::perturbation::threshold_end_idx])
    {
        if (_controller_data->parameters[controller_defs::perturbation::direction_idx] > 0)
        {
            cmd = -1 * _controller_data->parameters[controller_defs::perturbation::amplitude_idx];
        }
    }
    else
    {
        cmd = 0;
    }
 
    return -1*cmd;
};



//****************************************************


ConstantTorque::ConstantTorque(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    logger::println("ConstantTorque::Constructor");
#endif

    //current_torque = _controller_data->parameters[controller_defs::constant_torque::upper_idx];
    //counter = 0;

}

float ConstantTorque::calc_motor_cmd()
{
    //if (counter < _controller_data->parameters[controller_defs::constant_torque::iterations_idx])
    //{
    //    counter++;
    //}
    //else
    //{ 
    //    if (current_torque == _controller_data->parameters[controller_defs::constant_torque::upper_idx])
    //    {
    //        current_torque = _controller_data->parameters[controller_defs::constant_torque::lower_idx];
    //    }
    //    else if (current_torque == _controller_data->parameters[controller_defs::constant_torque::lower_idx])
    //    {
    //        current_torque = _controller_data->parameters[controller_defs::constant_torque::upper_idx];
    //    }
    //    else
    //    {
    //        current_torque = current_torque;
    //    }

    //    counter = 0;
    //}

    //float cmd = current_torque;

    float cmd = 0;     //Creates the cmd variable and initializes it to 0;

        if (_leg_data->do_calibration_toe_fsr)// || _leg_data->toe_stance == 1)                      //If the FSRs are being calibrated or if the toe fsr is 0, send a command of zero
        {
            cmd = 0;    
        }
        else
        {
            cmd = _controller_data->parameters[controller_defs::constant_torque::amplitude_idx];

            if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 0)                            //If the user wants to send a PF/Flexion torque
            {
                cmd = 1 * cmd;
            }
            else if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 1)                       //If the user wants to send a DF/Extension torque
            {
                cmd = -1 * cmd;
            }
            else
            {
                cmd = cmd;                                                                                                  //If the direction flag is something other than 0 or 1, do nothing to the motor command
            }
        }
    
    return cmd;
}

//****************************************************


ElbowMinMax::ElbowMinMax(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("ElbowMinMax::Constructor");
#endif

}

float ElbowMinMax::calc_motor_cmd()
{
	// Serial.print("\n ElbowMinMax selected... ");
	// Serial.print("  |  1st Level Toe fsr: ");
	// Serial.print(_leg_data->toe_fsr);
	// Serial.print("  |  2nd Level Heel fsr: ");
	// Serial.print(_leg_data->heel_fsr);
    float cmd = 0;     //Creates the cmd variable and initializes it to 0;
    // float cmd_toe_elbow = 0;
    // float cmd_heel_elbow = 0;
	
	if ((_leg_data->toe_fsr > _leg_data->heel_fsr) && (_leg_data->toe_fsr > 0.1*_controller_data->parameters[controller_defs::elbow_min_max::fsr_threshold_idx])) {
		cmd = _controller_data->parameters[controller_defs::elbow_min_max::amplitude_idx];
	}
	else if ((_leg_data->heel_fsr > _leg_data->toe_fsr) && (_leg_data->heel_fsr > 0.1*_controller_data->parameters[controller_defs::elbow_min_max::fsr_threshold1_idx])) {
		cmd = -1*_controller_data->parameters[controller_defs::elbow_min_max::amplitude_idx];
	}
	else {
		cmd = 0;
	}
	if (!_joint_data->is_left) {
		Serial.print("\nToe fsr: ");
		Serial.print(_leg_data->toe_fsr);
		Serial.print("  |  Heel fsr: ");
		Serial.print(_leg_data->heel_fsr);
		Serial.print("  |  cmd is set to: ");
		Serial.print(cmd);
	}
	
    /* float fsr_toe_current_elbow = _leg_data->toe_fsr;
    float fsr_heel_current_elbow = _leg_data->heel_fsr;
    // float fsr_toe_current_elbow = 0.8;
    // float fsr_heel_current_elbow = 2;
    if (_controller_data->is_first_fsr_reading_elbow) {
        //fsr_toe_current_elbow = utils::ewma(_leg_data->toe_fsr, _leg_data->toe_fsr, 0.5);
        //fsr_heel_current_elbow = utils::ewma(_leg_data->heel_fsr, _leg_data->heel_fsr, 0.5);
        _controller_data->fsr_toe_previous_elbow = _leg_data->toe_fsr;
        _controller_data->fsr_heel_previous_elbow = _leg_data->heel_fsr;
        _controller_data->is_first_fsr_reading_elbow = false;
        Serial.println("First fsr read");
    }
    else {
        //fsr_toe_current_elbow = utils::ewma(_leg_data->toe_fsr, _controller_data->fsr_toe_previous_elbow, 0.5);
        //fsr_heel_current_elbow = utils::ewma(_leg_data->heel_fsr, _controller_data->fsr_heel_previous_elbow, 0.5);
        _controller_data->fsr_toe_previous_elbow = fsr_toe_current_elbow;
        _controller_data->fsr_heel_previous_elbow = fsr_heel_current_elbow;
    }
    

    _controller_data->fsr_toe_array_elbow[_controller_data->i_elbow] = fsr_toe_current_elbow;
    _controller_data->fsr_heel_array_elbow[_controller_data->i_elbow] = fsr_heel_current_elbow;
    
    if (_controller_data->is_first_run_elbow) {
        if (_controller_data->i_elbow == 50) {
            _controller_data->i_elbow = 0;
            _controller_data->is_first_run_elbow = false;
        }
        else {
              _controller_data->fsr_toe_sum_elbow += fsr_toe_current_elbow;
              _controller_data->fsr_heel_sum_elbow += fsr_heel_current_elbow;
              if (fsr_toe_current_elbow > _controller_data->fsr_toe_max_elbow) {
                     _controller_data->fsr_toe_max_elbow = fsr_toe_current_elbow;
              }
              else if (fsr_toe_current_elbow < _controller_data->fsr_toe_min_elbow) {
                     _controller_data->fsr_toe_min_elbow = fsr_toe_current_elbow;
                 }
              if (fsr_heel_current_elbow > _controller_data->fsr_heel_max_elbow) {
                     _controller_data->fsr_heel_max_elbow = fsr_heel_current_elbow;
                 }
              else if (fsr_heel_current_elbow < _controller_data->fsr_heel_min_elbow) {
                     _controller_data->fsr_heel_min_elbow = fsr_heel_current_elbow;
                 }
        }
    }


    else {
        if (_controller_data->i_elbow == 50){
        _controller_data->i_elbow = 0;
            Serial.println("50 iterations done.");            
        }

        _controller_data->fsr_toe_sum_elbow -= _controller_data->fsr_toe_array_elbow[0];
        _controller_data->fsr_toe_sum_elbow += fsr_toe_current_elbow;
        _controller_data->fsr_heel_sum_elbow -= _controller_data->fsr_heel_array_elbow[0];
        _controller_data->fsr_heel_sum_elbow += fsr_heel_current_elbow;
        _controller_data->fsr_toe_array_elbow[_controller_data->i_elbow] = fsr_toe_current_elbow;
        _controller_data->fsr_heel_array_elbow[_controller_data->i_elbow] = fsr_heel_current_elbow;

    }

    
    for (int i = 0; i < 50; i++) {
        if (_controller_data->fsr_toe_array_elbow[_controller_data->i_elbow] > _controller_data->fsr_toe_max_elbow) {
            _controller_data->fsr_toe_max_elbow = _controller_data->fsr_toe_array_elbow[_controller_data->i_elbow];
        }
        else if (_controller_data->fsr_toe_array_elbow[_controller_data->i_elbow] < _controller_data->fsr_toe_min_elbow) {
            _controller_data->fsr_toe_min_elbow = _controller_data->fsr_toe_array_elbow[_controller_data->i_elbow];
        }
        if (_controller_data->fsr_heel_array_elbow[_controller_data->i_elbow] > _controller_data->fsr_heel_max_elbow) {
            _controller_data->fsr_heel_max_elbow = _controller_data->fsr_heel_array_elbow[_controller_data->i_elbow];
        }
        else if (_controller_data->fsr_heel_array_elbow[_controller_data->i_elbow] < _controller_data->fsr_heel_min_elbow) {
            _controller_data->fsr_heel_min_elbow = _controller_data->fsr_heel_array_elbow[_controller_data->i_elbow];
        }
    } 
    

    _controller_data->i_elbow++;

    // output_limit_elbow = _controller_data->parameters[controller_defs::elbow_min_max::output_limit_elbow];
    if (_controller_data->fsr_toe_min_elbow != _controller_data->fsr_toe_max_elbow) {
        _controller_data->fsr_min_max_elbow = (fsr_toe_current_elbow - _controller_data->fsr_toe_min_elbow)/(_controller_data->fsr_toe_max_elbow - _controller_data->fsr_toe_min_elbow);
        cmd_toe_elbow = _controller_data->output_limit_elbow * _controller_data->fsr_min_max_elbow;      
        // Serial.print("Toe: Current filtered FSR ");
        // Serial.print(fsr_toe_current_elbow, 2);
        // Serial.print(" |Current raw FSR ");
        // Serial.print(_leg_data->toe_fsr, 2);
        // Serial.print(" |Min ");
        // Serial.print(_controller_data->fsr_toe_min_elbow,2);
        // Serial.print("  |Max ");
        // Serial.print(_controller_data->fsr_toe_max_elbow,2);
        // Serial.print(" |Constant ");
        // Serial.print(_controller_data->output_limit_elbow);
        // Serial.print(" |Current ratio ");
        // Serial.print(_controller_data->fsr_min_max_elbow);
        // Serial.print("\n");
    }
    else {
        cmd_toe_elbow = 0;
        // Serial.print("Toe fsr readings equal.\n");
    }
    if (_controller_data->fsr_heel_min_elbow != _controller_data->fsr_heel_max_elbow) {
        _controller_data->fsr_min_max_elbow = (fsr_heel_current_elbow - _controller_data->fsr_heel_min_elbow)/(_controller_data->fsr_heel_max_elbow - _controller_data->fsr_heel_min_elbow);
        cmd_heel_elbow = _controller_data->output_limit_elbow * _controller_data->fsr_min_max_elbow;      
        // Serial.print("Heel: Min ");
        // Serial.print(_controller_data->fsr_heel_min_elbow,2);
        // Serial.print(" Max ");
        // Serial.print(_controller_data->fsr_heel_max_elbow,2);
        // Serial.print(" Constant ");
        // Serial.print(_controller_data->output_limit_elbow);
        // Serial.print(" Current ratio ");
        // Serial.print(_controller_data->fsr_min_max_elbow);
        // Serial.print("\n");
    }
    else {
        cmd_heel_elbow = 0;
        // Serial.println("Heel fsr readings equal.\n");
    }

    cmd = cmd_toe_elbow;
	cmd = 0;
    // Serial.print("cmd is ");
    // Serial.print(cmd, 2);
    // Serial.print("\n"); */
	
	// low pass filter on torque_reading
    /* const float torque = _joint_data->torque_reading * (_leg_data->is_left ? 1 : -1); */

    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);
	
	
	
	Serial.print("\n Pre PID cmd: ");
	Serial.print(cmd);
	
	cmd = _pid(cmd, _joint_data->torque_reading,10,0,200);
/* 	Serial.print("  |  Post PID cmd: ");
	Serial.print(cmd); */
	_controller_data->elbow_cmd = cmd;
    return cmd;
}
//****************************************************


PtbGeneral::PtbGeneral(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("PtbGeneral::Constructor");
#endif

}

float PtbGeneral::calc_motor_cmd()
{
    // float ptb_fsrToe = 0.25; // _leg_data->toe_fsr
    // float ptb_fsrHeel = 0.25; // _leg_data->heel_fsr
    float ptb_fsrToe = _leg_data->toe_fsr;
    float ptb_fsrHeel = _leg_data->heel_fsr;

    float cmd_ff = 0;
    //Serial.print("\n");
    //Serial.print("Index: ");
     //Serial.print((_controller_data->parameters[controller_defs::ptb_general::ptb_mode_idx]));
     //Serial.print(" | ");
    switch ((int)_controller_data->parameters[controller_defs::ptb_general::ptb_mode_idx]) {
		case 1: // constant torque during swing phase; the exact percent gait will be random
        Serial.print("\nIs in Mode 1 (continuous rotation). idx: 1 |  ");
        Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx]);
        Serial.print(" | ");
		Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_2_idx]);
		Serial.print(" | ");
		Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_3_idx]);
		Serial.print(" | ");
		Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_4_idx]);
        cmd_ff = -1 * _controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx];
        break;
		
		case 2: //FSR triggered
		_controller_data->ptb_iiStep++;
		if (_controller_data->ptbRandomIsFirstRun) {
			randomSeed(analogRead(A16));
			_controller_data->ptbRandomIsFirstRun = false;
		}
		_controller_data->ptb_oldIsSwing = _controller_data->ptb_newIsSwing;
		if (_leg_data->toe_stance) {
			_controller_data->ptb_newIsSwing = false;
		}
		else {
			_controller_data->ptb_newIsSwing = true;
		}
		if ((_controller_data->ptb_oldIsSwing == false) && (_controller_data->ptb_newIsSwing == true)) {
			_controller_data->ptb_iStep++;
			_controller_data->ptb_iiStep = 0;
		}
		if ((_controller_data->ptb_oldIsSwing)&&(_leg_data->toe_stance)) {//new stance started
		_controller_data->ptb_fsrGotHigh = false;//reset the HighFSR cursor
		}
		
		if (_leg_data->toe_fsr >= _controller_data->parameters[controller_defs::ptb_general::ptb_settings_9_idx]) {//FSR has reached high in the current step
			_controller_data->ptb_fsrGotHigh = true;
		}
		
		
		if (_controller_data->ptb_iStep == _controller_data->ptb_frequency) {
			if ((_controller_data->ptb_fsrGotHigh) && (_leg_data->toe_stance < _controller_data->parameters[controller_defs::ptb_general::ptb_settings_2_idx]) && (_controller_data->ptb_iiStep < 100*_controller_data->parameters[controller_defs::ptb_general::ptb_settings_3_idx])) {
			cmd_ff = -1 * _controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx];
			}
			else {
				cmd_ff = 0;
				if (!_leg_data->toe_stance) {
				cmd_ff = 2;
			}
			}
		}
		else {
			cmd_ff = 0;
			if (!_leg_data->toe_stance) {
				cmd_ff = 2;
			}
		}
		if (_controller_data->ptb_iStep > _controller_data->ptb_frequency) {
			_controller_data->ptb_iStep = 0;
			_controller_data->ptb_frequency = random(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_4_idx]);
		}
		break;		
			
		case 3://iteration oriented
		_controller_data->ptb_iiStep++;
		if (_controller_data->ptbRandomIsFirstRun) {
			randomSeed(analogRead(A16));
			_controller_data->ptbRandomIsFirstRun = false;
		}
		_controller_data->ptb_oldIsSwing = _controller_data->ptb_newIsSwing;
		if (_leg_data->toe_stance) {
			
			_controller_data->ptb_newIsSwing = false;
		}
		else {
			_controller_data->ptb_newIsSwing = true;
		}
		if ((_controller_data->ptb_oldIsSwing == false) && (_controller_data->ptb_newIsSwing == true)) {
			_controller_data->ptb_iStep++;
			//_controller_data->ptb_totalSteps++;
/* 				if (_joint_data->is_left) {
			Serial.print("\niStep: ");
			Serial.print(_controller_data->ptb_iStep);
			Serial.print(" | ");
			Serial.print(_controller_data->ptb_frequency);
			} */
			_controller_data->ptb_iiStep = 0;
		}		
		if (_controller_data->ptb_iStep == _controller_data->ptb_frequency) {
			if ((_controller_data->ptb_iiStep > 100*_controller_data->parameters[controller_defs::ptb_general::ptb_settings_2_idx]) && (_controller_data->ptb_iiStep < 100*_controller_data->parameters[controller_defs::ptb_general::ptb_settings_3_idx])){
			cmd_ff = -1 * _controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx];
			}
			else {
				cmd_ff = 0;
			}
		}
		else {
			cmd_ff = 0;
		}
		if (_controller_data->ptb_iStep > _controller_data->ptb_frequency) {
			_controller_data->ptb_iStep = 0;
			_controller_data->ptb_frequency = random(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_4_idx]);
		}
		break;
		

		case 4://percent gait oriented
		_controller_data->ptb_iiStep++;
		if (_controller_data->ptbRandomIsFirstRun) {
			randomSeed(analogRead(A16));
			_controller_data->ptbRandomIsFirstRun = false;
		}
		_controller_data->ptb_oldIsSwing = _controller_data->ptb_newIsSwing;
		if (_leg_data->toe_stance) {
			
			_controller_data->ptb_newIsSwing = false;
		}
		else {
			_controller_data->ptb_newIsSwing = true;
		}
		if ((_controller_data->ptb_oldIsSwing == false) && (_controller_data->ptb_newIsSwing == true)) {
			_controller_data->ptb_iStep++;
			_controller_data->ptb_iiStep = 0;
		}
		if (_controller_data->ptb_iStep == _controller_data->ptb_frequency) {
			if ((_leg_data->percent_gait > _controller_data->parameters[controller_defs::ptb_general::ptb_settings_2_idx]) && (_leg_data->percent_gait < _controller_data->parameters[controller_defs::ptb_general::ptb_settings_3_idx])){
			cmd_ff = -1 * _controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx];
			}
			else {
				cmd_ff = 0;
			}
		}
		else {
			cmd_ff = 0;
		}
		if (_controller_data->ptb_iStep > _controller_data->ptb_frequency) {
			_controller_data->ptb_iStep = 0;
			_controller_data->ptb_frequency = random(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_4_idx]);
		}
		break;

		default:
		Serial.print("\n");
		Serial.print("Is in default mode. Toe FSR: ");
		Serial.print(ptb_fsrToe);
		_controller_data->isPerturbing = false;
		_controller_data->ptbApplied = false;
		_controller_data->ptbDetermined = false;
		cmd_ff = 0;

    }

    // Uncomment the next paragraph to evaluate code running speed
/*     _controller_data->time_current_ptb = micros();
    float run_time_ptb = _controller_data->time_current_ptb - _controller_data->time_previous_ptb;
    _controller_data->time_previous_ptb = _controller_data->time_current_ptb;
    Serial.print("\n | Iteration run time: ");
    Serial.print(run_time_ptb); */
	
	if ((_joint_data->is_left)&&(cmd_ff != 0 )) {
		Serial.print("\n Percent gait (left side): ");
		Serial.print(_leg_data->percent_gait);
	}
	Serial.print("\ncmd: ");
	Serial.print(cmd_ff);

    _controller_data->filtered_setpoint = utils::ewma(cmd_ff, _controller_data->filtered_setpoint, 0.1);
    _controller_data->ff_setpoint = _controller_data->filtered_setpoint;

    //Low Pass Filter for Torque Sensor
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);

    //PID Control
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::ptb_general::use_pid_idx]
        ? _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::ptb_general::p_gain_idx], _controller_data->parameters[controller_defs::ptb_general::i_gain_idx], _controller_data->parameters[controller_defs::ptb_general::d_gain_idx])
        : 0);

	Serial.print(" | cmd_pid: ");
	Serial.print(cmd);

    _controller_data->filtered_cmd = utils::ewma(cmd, _controller_data->filtered_cmd, 1);
    return _controller_data->filtered_cmd;
}


//****************************************************


CalibrManager::CalibrManager(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("CalibrManager::CalibrManager");
#endif

}

float CalibrManager::calc_motor_cmd()
{
	float cmd;
	//float calibrSum;
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
		
		if (_joint_data->is_left) {
		Serial.print("\nLeft angle: ");
		Serial.print(_leg_data->ankle.joint_position);
		Serial.print("  |  Left torque: ");
		Serial.print(_joint_data->torque_reading);
		// cmd = _controller_data->parameters[controller_defs::calibr_manager::calibr_cmd];
		cmd = 3.5;
		Serial.print("  |  Left cmd: ");
		Serial.print(cmd);
	}
	else {
		Serial.print("  |  Right angle: ");
		Serial.print(_leg_data->ankle.joint_position);
		Serial.print("  |  Right torque: ");
		Serial.print(_joint_data->torque_reading);
		cmd = 3.5;
		Serial.print("  |  Right cmd: ");
		Serial.print(cmd);
	}
		
/*     if (_data->user_paused || !active_trial)
    {
        return;
    } */
/* _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);
if (_leg_data->do_calibration_toe_fsr || _leg_data->do_calibration_refinement_toe_fsr) {
	cmd = 0;
}
else {
	_controller_data->iCalibr++;
	if (_controller_data->iCalibr < 100) {
		cmd = -3.5;
		_controller_data->calibrSum = _controller_data->calibrSum + _controller_data->filtered_torque_reading * cmd;
		if (_controller_data->calibrSum > 0) {
			_controller_data->PIDMLTPLR = 1;
		}
		if (_controller_data->calibrSum < 0) {
			_controller_data->PIDMLTPLR = -1;
		}
	}
	else {
		_controller_data->iCalibr = 0;
		_controller_data->calibrSum = 0;
	} */
/* 	if (!_joint_data->is_left) {
	Serial.print(_controller_data->PIDMLTPLR);
	Serial.print("  |  Torque reading: ");
	Serial.print(_controller_data->filtered_torque_reading);
	Serial.print("  |  cmd: ");
	Serial.print(cmd);
	Serial.print("\n");
	} */
	
//}
return cmd;
	
}
#endif