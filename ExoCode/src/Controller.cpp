/*
 * P. Stegall Jan. 2022
*/

#include "Controller.h"
#include "Logger.h"
//#define CONTROLLER_DEBUG          //Uncomment to enable debug statements to be printed to the serial monitor

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
#include <math.h>
#include <random>
#include <cmath>
#include <Servo.h>
Servo myservo;//TO-DO: Move servo definition code out of controller.cpp

_Controller::_Controller(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _data = exo_data;
    
    _t_helper = Time_Helper::get_instance();
    _t_helper_context = _t_helper->generate_new_context();
    _t_helper_delta_t = 0;
    
    //We just need to know the side to point at the right data location so it is only for the constructor
    bool is_left = utils::get_is_left(_id);
    
    #ifdef CONTROLLER_DEBUG
        logger::print(is_left ? "Left " : "Right ");
    #endif 

    _prev_input = 0; 
    _prev_de_dt = 0;
        
    //Set _controller_data to point to the data specific to the controller.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            #ifdef CONTROLLER_DEBUG
                logger::print("HIP ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_side.hip.controller);
                _joint_data = &(exo_data->left_side.hip);
            }
            else
            {
                _controller_data = &(exo_data->right_side.hip.controller);
                _joint_data = &(exo_data->right_side.hip);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            #ifdef CONTROLLER_DEBUG
                logger::print("KNEE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_side.knee.controller);
                _joint_data = &(exo_data->left_side.knee);
            }
            else
            {
                _controller_data = &(exo_data->right_side.knee.controller);
                _joint_data = &(exo_data->right_side.knee);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            #ifdef CONTROLLER_DEBUG
                logger::print("ANKLE ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_side.ankle.controller);
                _joint_data = &(exo_data->left_side.ankle);
            }
            else
            {
                _controller_data = &(exo_data->right_side.ankle.controller);
                _joint_data = &(exo_data->right_side.ankle);
            }
            break;
        case (uint8_t)config_defs::joint_id::elbow:
            #ifdef CONTROLLER_DEBUG
                        logger::print("ELBOW ");
            #endif
            if (is_left)
            {
                _controller_data = &(exo_data->left_side.elbow.controller);
                _joint_data = &(exo_data->left_side.elbow);
            }
            else
            {
                _controller_data = &(exo_data->right_side.elbow.controller);
                _joint_data = &(exo_data->right_side.elbow);
            }
            break;
    }

    #ifdef CONTROLLER_DEBUG
        logger::print("Controller : \n\t_controller_data set \n\t_joint_data set");
    #endif

    //Added a pointer to the side data as most controllers will need to access info specific to their side.
    if (is_left)
    {
        _side_data = &(exo_data->left_side);
    }
    else
    {
        _side_data = &(exo_data->right_side);
    } 

    #ifdef CONTROLLER_DEBUG
        logger::println("\n\t_side_data set");
    #endif
    
    //Set the parameters for cf mfac
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

//****************************************************

float _Controller::_cf_mfac(float reference, float current_measurement)         //Compact Form Model Free Adaptive Controller (In-development, not yet employed)
{
    //Calculate k-1 (k_0) delta
    const float du_k0 = outputs.second - outputs.first;

    //Prime the state
    measurements.first = measurements.second;
    outputs.first = outputs.second;
    phi.first = phi.second;
    measurements.second = current_measurement;

    //Calculate k delta
    const float dy_k = measurements.second - measurements.first;

    //Calculate the new psuedo partial derivative
    const float phi_numerator = etta * du_k0 * (dy_k - (phi.first*du_k0));
    const float phi_denominator = mu + (du_k0*du_k0);
    phi.second = phi.first + (phi_numerator/phi_denominator);

    //Calculate the new output
    const float error = reference - measurements.second;
    const float u_numerator = rho * phi.second * error;
    const float u_denominator = lamda + (abs(phi.second) * abs(phi.second));
    outputs.second = outputs.first + (u_numerator/u_denominator);
    return outputs.second;
}

//****************************************************
 
float _Controller::_pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain)
{	
    //Check if time is ok
    bool time_good = true;

    if (_t_helper->tick(_t_helper_context) > ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        time_good = false;
    }

    //Record the current time
    float now = micros();

    //Record the change in time 
    float dt = (now - _prev_pid_time) * 1000000;

    //Calculate the difference in the prescribed and measured torque 
    float error_val = cmd - measurement;  

    //If we want to to include the integral term (Note: We generally do not like to use the I gain but we have it here for completeness) 
    if (i_gain != 0)
    {
        _pid_error_sum += error_val / LOOP_FREQ_HZ;
    }
    else
    {
        _pid_error_sum = 0;
    }

    //Get the current status of the exskleton 
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || (exo_status == status_defs::messages::fsr_calibration) || (exo_status == status_defs::messages::fsr_refinement);

    //Reset the integral term if the user pauses the trial or we are no longer in an active trial
    if (_data->user_paused || !active_trial)
    {
        _pid_error_sum = 0;
    }

    //Initialize the derivative of the error 
    float de_dt = 0;

    //Calculate the derivative of the erro
    if (time_good)
    {
       de_dt = -(measurement - _prev_input) * (1000.0f / LOOP_FREQ_HZ);  //Convert to ms
       _prev_de_dt = de_dt;
    }
    else 
    {
        de_dt = 0;
    }

    //Set the previous times for the next loop through the controller
    _prev_pid_time = now;
    _prev_input = measurement;

    //Calculate the individual P,I,and D Terms
    float p = p_gain * error_val;  
    float i = i_gain * _pid_error_sum; 
    float d = d_gain * de_dt; 

    //Return the summed PID
    return p + i + d;

}

//****************************************************
 
float _Controller::_pjmc_generic(float current_fsr, float fsr_threshold, float setpoint_positive, float setpoint_negative)
{
	if (fsr_threshold == 1) {
		return 0;//fsr_threshold shouldn't be set to 1
	}
	float slope = (setpoint_positive - setpoint_negative)/(1 - fsr_threshold);
	float prescribed_val = setpoint_positive - slope * (current_fsr - fsr_threshold);
	return prescribed_val;//this prescribed value is not capped
}

//****************************************************

int _Controller::_servo_runner(uint8_t servo_pin, uint8_t speed_level, long angle_initial, long angle_final)
{
	if (!((pos1 == angle_initial)&&(pos2 == angle_final))) {
		if (!myservo.attached()) {
			myservo.attach(servo_pin,500,2500);//attach the servo object
		}
		pos1 = angle_initial;
		pos2 = angle_final;
		pos = pos1;
		myservo.write(pos);
		_do_stop_servo = false;
		servoWatch = millis();
	}
	if ((millis() - servoWatch > 15)&&(!_do_stop_servo)) {
		myservo.write(pos);
		if (pos1 < pos2) {
			pos += speed_level;
			pos = constrain(pos, pos1, pos2);
		}
		else if (pos1 > pos2) {
			pos -= speed_level;
			pos = constrain(pos, pos2, pos1);
		}
		servoWatch = millis();
		if (pos == pos2) {	
			_do_stop_servo = true;
		}
	}
	return pos;//report the current feedforward "pos";
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
    //Set feed-forward command to zero
    float cmd_ff = 0;

    //Set the motor command to the feed-forward command
    float cmd = cmd_ff;
    
    //Add the PID contribution to the motor command if desired 
    if (_controller_data->parameters[controller_defs::zero_torque::use_pid_idx])
    {
        cmd = cmd_ff + _pid(cmd_ff, _joint_data->torque_reading, _controller_data->parameters[controller_defs::zero_torque::p_gain_idx], _controller_data->parameters[controller_defs::zero_torque::i_gain_idx], _controller_data->parameters[controller_defs::zero_torque::d_gain_idx]);
    }

    //Set the feed-forward setpoint to the feed-forward command
    _controller_data->ff_setpoint = cmd_ff;

    //Send the motor command
    return cmd;
}

//****************************************************

TREC::TREC(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("TREC::Constructor");
    #endif
}

void TREC::_update_reference_angles(SideData* side_data, ControllerData* controller_data, float percent_grf, float percent_grf_heel)
{
    //When the percent_grf passes the threshold, update the reference angle
    const float threshold = controller_data->parameters[controller_defs::trec::timing_threshold]/100;
    const bool should_update = (percent_grf > controller_data->toeFsrThreshold) && !controller_data->reference_angle_updated;
    const bool should_capture_level_entrance = side_data->do_calibration_refinement_toe_fsr && !side_data->do_calibration_toe_fsr;
    const bool should_reset_level_entrance_angle = controller_data->prev_calibrate_level_entrance < should_capture_level_entrance;

    if (should_reset_level_entrance_angle)
    {
        controller_data->level_entrance_angle = 0.5;
    }

    if (should_update)
    {
        if (should_capture_level_entrance)
        {
            controller_data->level_entrance_angle = utils::ewma(side_data->ankle.joint_position, controller_data->level_entrance_angle, controller_data->cal_level_entrance_angle_alpha);
        }

        controller_data->reference_angle_updated = true;
        controller_data->reference_angle = side_data->ankle.joint_position;
        
        //controller_data->reference_angle_offset = side_data->ankle.joint_global_angle;
    }

    //When the percent_grf drops below the threshold, reset the reference angle updated flag and expire the reference angle
    const bool should_reset = (percent_grf < controller_data->toeFsrThreshold) && controller_data->reference_angle_updated;
    
    if (should_reset)
    {
        controller_data->reference_angle_updated = false;
        controller_data->reference_angle = 0;
        controller_data->reference_angle_offset = 0;
    }

    controller_data->prev_calibrate_level_entrance = should_capture_level_entrance;
}

void TREC::_capture_neutral_angle(SideData* side_data, ControllerData* controller_data)
{
    //On the start of torque calibration reset the neutral angle
    if (controller_data->prev_calibrate_trq_sensor < side_data->ankle.calibrate_torque_sensor)
    {
        controller_data->neutral_angle = side_data->ankle.joint_position;
    }

    if (side_data->ankle.calibrate_torque_sensor) 
    {
        //Update the neutral angle with an ema filter
        controller_data->neutral_angle = utils::ewma(side_data->ankle.joint_position, controller_data->neutral_angle, controller_data->cal_neutral_angle_alpha);
    }

    controller_data->prev_calibrate_trq_sensor = side_data->ankle.calibrate_torque_sensor;
}

void TREC::_grf_threshold_dynamic_tuner(SideData* side_data, ControllerData* controller_data, float threshold, float percent_grf_heel)
{
	//If it's swing phase, set wait4HiHeelFSR to True, and increase the toeFSR threshold; when wait4HiHeelFSR is true and heelFSR > a pre-defined threshold, reduce the toeFSR threshold; when it's stance phase, set wait4HiHeelFSR to False
	if (!side_data->toe_stance)
    {
		controller_data->wait4HiHeelFSR = true;
	}
	else
    {
		controller_data->wait4HiHeelFSR = false;
		controller_data->toeFsrThreshold = threshold*0.01;
	}
	if (controller_data->wait4HiHeelFSR) 
    {
		if (percent_grf_heel > threshold) 
        {
			controller_data->toeFsrThreshold = threshold*0.1;
		}
		else 
        {
			controller_data->toeFsrThreshold = threshold;
		}
	}
}

void TREC::_plantar_setpoint_adjuster(SideData* side_data, ControllerData* controller_data, float pjmcSpringDamper)
{
	if(_side_data->toe_stance) 
    {	
		//Update peak values
		_controller_data->maxPjmcSpringDamper = max(_controller_data->maxPjmcSpringDamper, pjmcSpringDamper);
		_controller_data->wasStance = true;
	}
	else 
    {
		if (_controller_data->wasStance) 
        {
			_controller_data->prevMaxPjmcSpringDamper = _controller_data->maxPjmcSpringDamper;
			_controller_data->maxPjmcSpringDamper = 0;
			
			if (_controller_data->prevMaxPjmcSpringDamper < _controller_data->parameters[controller_defs::trec::plantar_scaling]) 
            {
			_controller_data->setpoint2use ++;
			}
			else 
            {
				_controller_data->setpoint2use --;
			}

			_controller_data->setpoint2use = min(_controller_data->setpoint2use, 35);
			_controller_data->setpoint2use = max(_controller_data->setpoint2use, 0);
			_controller_data->wasStance = false;
		}
	}
	if (_controller_data->prevMaxPjmcSpringDamper == 0) 
    {
		_controller_data->setpoint2use = _controller_data->parameters[controller_defs::trec::plantar_scaling];
	}
}


float TREC::calc_motor_cmd()
{
    #ifdef CONTROLLER_DEBUG
        logger::println("TREC::calc_motor_cmd : start");
    #endif

    static const float sigmoid_exp_scalar{50.0f};

    //Calculate Generic Contribution
	float plantar_setpoint = 0;

    if (_controller_data->parameters[controller_defs::trec::turn_on_peak_limiter]) 
    {
		plantar_setpoint = _controller_data->setpoint2use;
	}
	else 
    {
		plantar_setpoint = _controller_data->parameters[controller_defs::trec::plantar_scaling];
		_controller_data->setpoint2use = plantar_setpoint;
	}

	const float dorsi_setpoint = -_controller_data->parameters[controller_defs::trec::dorsi_scaling];
    const float threshold = _controller_data->parameters[controller_defs::trec::timing_threshold]/100;
    const float percent_grf = min(_side_data->toe_fsr, 1);
	const float percent_grf_heel = min(_side_data->heel_fsr, 1);
    const float slope = (plantar_setpoint - dorsi_setpoint)/(1 - threshold);
    const float generic = max(((slope*(percent_grf - threshold)) + dorsi_setpoint), dorsi_setpoint);                    //Stateless "PJMC" stateless
	_controller_data->stateless_pjmc_term = generic;

    //Assistive Contribution (a.k.a: Suspension; this term consists of a "Spring term" and a "Damper term" as the suspension)
    _capture_neutral_angle(_side_data, _controller_data);
	_grf_threshold_dynamic_tuner(_side_data, _controller_data, threshold, percent_grf_heel);
    _update_reference_angles(_side_data, _controller_data, percent_grf, percent_grf_heel);   //When current toe FSR > set threshold, use the current ankle angle as the "reference angle"
    const float k = 0.01 * _controller_data->parameters[controller_defs::trec::spring_stiffness];
    const float b = 0.01 * _controller_data->parameters[controller_defs::trec::damping];
    const float equilibrium_angle_offset = _controller_data->parameters[controller_defs::trec::neutral_angle]/100;
    const float deviation_from_level = (_controller_data->reference_angle - _controller_data->level_entrance_angle);
    const float delta = _controller_data->reference_angle + deviation_from_level - _side_data->ankle.joint_position + equilibrium_angle_offset;//describes the amount of dorsi flexion since toe FSR > set threshold (negative at more plantarflexed angles)
    const float assistive = max(k*delta - b*_side_data->ankle.joint_velocity, 0);//Dorsi velocity: Negative

    //Use a tuned sigmoid to squelch the spring output during the 'swing' phase
    const float squelch_offset = -(1.5*_controller_data->toeFsrThreshold);                                                                                  //1.5 ensures that the spring activates after the new angle is captured
    const float grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))) / (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))+1);
    const float squelched_supportive_term = assistive*grf_squelch_multiplier;                                                                               //Finalized suspension term
    
    //Low pass the squelched supportive term
    _controller_data->filtered_squelched_supportive_term = utils::ewma(squelched_supportive_term, _controller_data->filtered_squelched_supportive_term, 0.075);

    //Propulsive Contribution
    const float kProp = 0.01 * _controller_data->parameters[controller_defs::trec::propulsive_gain];
    const float saturated_velocity = _side_data->ankle.joint_velocity > 0 ? _side_data->ankle.joint_velocity:0;
    const float propulsive = kProp*saturated_velocity;

    //Use a symmetric sigmoid to squelch the propulsive term
    const float propulsive_squelch_offset = -1.1 + threshold;
    const float propulsive_grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))) / (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))+1);
    const float squelched_propulsive_term = propulsive*propulsive_grf_squelch_multiplier;
    
	//PJMC reducer
	if (_controller_data->parameters[controller_defs::trec::turn_on_peak_limiter]) 
    {
		_plantar_setpoint_adjuster(_side_data, _controller_data, _controller_data->filtered_squelched_supportive_term+generic);
	}
	
    //Sum for ff
    const float cmd_ff = -(_controller_data->filtered_squelched_supportive_term+generic+squelched_propulsive_term); //According to the new motor command direction definitions, at the ankle, positive for dorsi, and negative for plantar.

    //Low pass filter on torque_reading
    const float torque = _joint_data->torque_reading;
    const float alpha = 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, _controller_data->filtered_torque_reading, alpha);

    //Close the loop
    float cmd = _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::trec::kp], 0, _controller_data->parameters[controller_defs::trec::kd]);
			
    //Satuarte command to prevent if from going beyond desired limits of torque    
	cmd = min(cmd, cmd_ff + 35);
	cmd = max(cmd, cmd_ff - 35);
	
    //Update plotting variables
    _controller_data->ff_setpoint = cmd_ff;
	_controller_data->setpoint = cmd;
    _controller_data->filtered_setpoint = squelched_propulsive_term;

    #ifdef CONTROLLER_DEBUG
        logger::println("TREC::calc_motor_cmd : stop");
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

    /* Set FSR thresholds to engage controller -> Tells it foot is on the ground*/
    _stance_thresholds_left.first = exo_data->left_side.toe_fsr_lower_threshold;
    _stance_thresholds_left.second = exo_data->left_side.toe_fsr_upper_threshold;
    _stance_thresholds_right.first = exo_data->right_side.toe_fsr_lower_threshold;
    _stance_thresholds_right.second = exo_data->right_side.toe_fsr_upper_threshold;
}

float ProportionalJointMoment::calc_motor_cmd()
{
	
    #ifdef CONTROLLER_DEBUG
        logger::println("ProportionalJointMoment::calc_motor_cmd : start");
    #endif

    float cmd_ff = 0;

    /* If the toe is on the ground, calculate the feed-forward command */
    if (_side_data->toe_stance) 
    {
        /* Scale the fsr values so the controller outputs zero feed forward when the FSR value is at the threshold */ 
        float threshold = _side_data->toe_fsr_upper_threshold;       /* Get the upper threshold for the FSR to be considered in stance */
        float fsr = min(_side_data->toe_fsr, 1.2);                   /* Stores a saturated FSR signal from the device, saturates in order to avoid producing a large torque. */
        float scaled_fsr = (fsr - threshold) / (1 - threshold);     /* Re-scales FSR signal */

        /* Calculate FeedForward Command */
        cmd_ff = (scaled_fsr) * _controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx];          
            
        /* Saturates Command so that it never is opposite of intent (e.g., going into resistance when wanting assistance) */
        cmd_ff = max(0, cmd_ff);
        
        /* Sets to Assistance or Resistance Based on Controller Parameter Flag */
        cmd_ff = -1 * cmd_ff * (_controller_data->parameters[controller_defs::proportional_joint_moment::is_assitance_idx] ? 1 : -1);
    }
    else /* If the foot is not on the ground, calculate the swing phase command. */
    {
        /* Sets the Assistance to a Constant, user defined torque. */
        cmd_ff = _controller_data->parameters[controller_defs::proportional_joint_moment::swing_max_idx];
    }

    /* Low-Pass Filter Measured Torque */
	const float torque = _joint_data->torque_reading;
    const float alpha = (_controller_data->parameters[controller_defs::proportional_joint_moment::torque_alpha_idx] != 0) ? _controller_data->parameters[controller_defs::proportional_joint_moment::torque_alpha_idx] : 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, _controller_data->filtered_torque_reading, 1); //NOTE: Currently hard coded to not filer, can do so by replacing 1 with alpha

    /* Find the maximum measured torque and maximum setpoint during stance */
    if (_side_data->toe_stance) 
    {
        /* Store the current command and measured torque into set variables */
		const float new_torque = _controller_data->filtered_torque_reading;
		const float new_ff = cmd_ff;
		
        /* Compares to previous maxiums during this step and overwrights those variables if current is larger. */
        _controller_data->max_measured = (_controller_data->max_measured < new_torque) ? new_torque : _controller_data->max_measured;
        _controller_data->max_setpoint = (_controller_data->max_setpoint < new_ff) ? new_ff : _controller_data->max_setpoint;
    }

    /* Set previous max values on rising edge */
    if (_side_data->ground_strike) /* If a ground strike is detected */
    {
        /* Set the previous maximums to the max measured from the previous step, reset those variables to zero. */
        _controller_data->prev_max_measured = _controller_data->max_measured;
        _controller_data->prev_max_setpoint = _controller_data->max_setpoint;
        _controller_data->max_measured = 0;
        _controller_data->max_setpoint = 0;

        /* Caluculate the Kf (velocity feed-forward gain) for this step */
        if ((_controller_data->prev_max_measured > 0.0f) && (_controller_data->parameters[controller_defs::proportional_joint_moment::stance_max_idx] != 0.0f)) /* If the previous maximum was not zero and the controller maximum is not zero. */
        {
            /* Kf is the last set Kf + the ratio of the previous steps max setpoint to the previous steps max measured normalized by direction. */
            _controller_data->kf = _controller_data->kf + ((_controller_data->prev_max_setpoint/_controller_data->prev_max_measured) - 1);

            /* Contrains the Kf to be within 0.75 - 1.25 */
            _controller_data->kf = min(1.25, _controller_data->kf);
            _controller_data->kf = max(0.75, _controller_data->kf);
        }
    }

    /* Filters the Setpoint */
    _controller_data->filtered_setpoint = utils::ewma(cmd_ff, _controller_data->filtered_setpoint, 1);
    _controller_data->ff_setpoint = _controller_data->filtered_setpoint;

    /* Add the PID contribution to the feed forward command */
    float cmd = 0;
    float kf_cmd = (_side_data->toe_stance) ? (_controller_data->kf * _controller_data->filtered_setpoint) : _controller_data->filtered_setpoint;

    /* If the PID flag is enalbed, do PID control, otherwise just send feed-forward command. */
    if (_controller_data->parameters[controller_defs::proportional_joint_moment::use_pid_idx])
    {

		cmd = cmd_ff + _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::proportional_joint_moment::p_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::i_gain_idx], _controller_data->parameters[controller_defs::proportional_joint_moment::d_gain_idx]);

	}
    else
    {
        cmd = cmd_ff;
    }
    
    /* Filter the commnad being sent to the motor. */
    _controller_data->filtered_cmd = utils::ewma(cmd, _controller_data->filtered_cmd, 1);

    /* Send the motor the command. */
    return _controller_data->filtered_cmd;
}

//****************************************************

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
    
    //Calculates Percent Gait
    float percent_gait = _side_data->percent_stance;
			
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
    
    //Sets the feed-forward setpoint to the desired command
    _controller_data->ff_setpoint = torque_cmd;
    
    //Filters the torque
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);

    //Adds PID Control if desired 
	if (_controller_data->parameters[controller_defs::zhang_collins::use_pid_idx])
	{
		cmd = torque_cmd + _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::zhang_collins::p_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::i_gain_idx], _controller_data->parameters[controller_defs::zhang_collins::d_gain_idx]);
	}
	else
	{
		cmd = torque_cmd;
	}

    //Sets previous command for next loop of controller
	_controller_data->previous_cmd = cmd;
	
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

    //Calculates the percent gait
    float percent_gait = _side_data->percent_gait;
    float expected_duration = _side_data->expected_step_duration;

    //Determines the time when the user exceeds the defined startpoint of the shifted gait cycle (done to avoid discontinuties realted to heel strike)
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

    //Stores the percent gait for the next loop
    last_percent_gait = percent_gait;

    //Return 0 torque until we have completed a full gait cycle
    if (last_start_time == -1)
    {
        return 0;
    }

    //Calcualtes the shifted percent gait cycle to avoid discontinuity at heel strike
    float shifted_percent_gait = (millis() - last_start_time) / expected_duration * 100;

    // logger::print("Franks::calc_motor_cmd : shifted_percent_gait = ");
    // logger::print(shifted_percent_gait);
    // logger::print("\n");

    float torque_cmd = 0;

    //Pulls in User Defined Controller Parameters
    float mass = _controller_data->parameters[controller_defs::franks_collins_hip::mass_idx];                                               /* User bodymass, currently not used but available if you want to normalize torque mangitude. */
    float extension_torque_peak = _controller_data->parameters[controller_defs::franks_collins_hip::trough_normalized_torque_Nm_kg_idx];    /* Extension torque setpoint. */
    float flexion_torque_peak = _controller_data->parameters[controller_defs::franks_collins_hip::peak_normalized_torque_Nm_kg_idx];        /* Flexion torque setpoint. */

    float extension_torque_magnitude_Nm = -1 * extension_torque_peak;   /* Sign corrected extension torque magnitude. */
    float flexion_torque_magnitude_Nm = flexion_torque_peak;            /* Sign corrected flexion torque magnitude. */

    float mid_time_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::mid_time_idx];                          /* % gait cycle of the middle of the zero torque region of the curve. */
    float mid_duration_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::mid_duration_idx];                  /* Duration (in % gait cycle) that zero torque is applied */

    float extension_peak_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::trough_percent_gait_idx];         /* % gait cycle where the extension torque curve starts. */
    float extension_rise_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::trough_onset_percent_gait_idx];   /* % gait cycle where the extension torque curve peaks. */
    float extension_fall_percent_gait = (mid_time_percent_gait - (mid_duration_percent_gait / 2)) - extension_peak_percent_gait;            /* % gait cycle where the extension torque curve ends. */

    float flexion_peak_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::peak_percent_gait_idx];             /* % gait cycle where the flexion torque curve starts. */
    float flexion_rise_percent_gait = flexion_peak_percent_gait - (mid_time_percent_gait + (mid_duration_percent_gait / 2));                /* % gait cycle where the flexion torque curve peaks. */
    float flexion_fall_percent_gait = _controller_data->parameters[controller_defs::franks_collins_hip::peak_offset_percent_gait_idx];      /* % gait cycle where the flexion torque curve ends. */

    //logger::print("Franks::calc_motor_cmd : flexion_peak_percent_gait = ");
    //logger::print(flexion_peak_percent_gait);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : flexion_rise_percent_gait = ");
    //logger::print(flexion_rise_percent_gait);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : flexion_fall_percent_gait = ");
    //logger::print(flexion_fall_percent_gait);
    //logger::print("\n");

    //Calculates the nodes for the flexion and extension curves for spline generation
    float extension_node1 = extension_peak_percent_gait - extension_rise_percent_gait;
    float extension_node2 = extension_peak_percent_gait;
    float extension_node3 = extension_peak_percent_gait + extension_fall_percent_gait;

    float flexion_node1 = flexion_peak_percent_gait - flexion_rise_percent_gait - (100 - start_percent_gait);
    float flexion_node2 = flexion_peak_percent_gait - (100 - start_percent_gait);
    float flexion_node3 = flexion_peak_percent_gait + flexion_fall_percent_gait - (100 - start_percent_gait);

    //logger::print("Franks::calc_motor_cmd : node1 = ");
    //logger::print(flexion_node1);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : node2 = ");
    //logger::print(flexion_node2);
    //logger::print("\n");

    //logger::print("Franks::calc_motor_cmd : node3 = ");
    //logger::print(flexion_node3);
    //logger::print("\n");

    //Calculates the feed-forward command by generating the spline curve based on where the user is estimated to be in their gait cycle
    torque_cmd = _spline_generation(extension_node1, extension_node2, extension_node3, extension_torque_magnitude_Nm, shifted_percent_gait) + _spline_generation(flexion_node1, flexion_node2, flexion_node3, flexion_torque_magnitude_Nm, percent_gait);

    //Filter Torque Reading
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);

    //Define the Feed-Forward Setpoint
    _controller_data->ff_setpoint = torque_cmd;

    float cmd = 0;

    //Determine if it should be open or closed loop control and calculate accordingly
    if (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx] > 0)
    {
        cmd = torque_cmd + _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], 0, _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx]);
    }
    else
    {
        cmd = torque_cmd;
    }

    return cmd;
}

float FranksCollinsHip::_spline_generation(float node1, float node2, float node3, float torque_magnitude, float shifted_percent_gait)
{
    float u;

    float x[3] = {node1, node2, node3};
    float y[3] = {0, torque_magnitude, 0};

    float h[2] = { (x[1] - x[0]), (x[2] - x[1]) };
    float delta[2] = { ((y[1] - y[0]) / h[0]), ((y[2] - y[1]) / h[1]) };

    float dy[3] = { 0, 0, 0};

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

ConstantTorque::ConstantTorque(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    logger::println("ConstantTorque::Constructor");
#endif

    //Initializes variables upon startup
    previous_torque_reading = 0;
    previous_command = 0;
    flag = 0;
    difference = 0;

}

float ConstantTorque::calc_motor_cmd()
{

    if ((_joint_data->is_left))
    {
        //Creates the cmd variable and initializes it to 0;
        float cmd_ff = 0;     

        if (_side_data->do_calibration_toe_fsr)          //If the FSRs are being calibrated or if the toe fsr is 0, send a command of zero
        {
            cmd_ff = 0;
        }
        else
        {
            cmd_ff = _controller_data->parameters[controller_defs::constant_torque::amplitude_idx];         //Send a command at the specified amplitude

            if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 0)         //If the user wants to send a PF/Flexion torque
            {
                cmd_ff = 1 * cmd_ff;
            }
            else if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 1)    //If the user wants to send a DF/Extension torque
            {
                cmd_ff = -1 * cmd_ff;
            }
            else
            {
                cmd_ff = cmd_ff;                                                                            //If the direction flag is something other than 0 or 1, do nothing to the motor command
            }
        }

        //If the command changes
        if (cmd_ff != previous_command)
        {
            flag = 1;                                   //Set the filter flag to 1
            difference = cmd_ff - previous_command;     //Determine the sign of the change in command 
        }

        //If the command is to send a larger torque
        if (difference > 0)
        {
            if (flag == 1 && previous_torque_reading >= cmd_ff)   //Set the flag to 0 when the measured torque reaches the desired setpoint
            {
                flag = 0;
            }
        }

        //If the command is to send a smaller torque 
        if (difference < 0)
        {
            if (flag == 1 && previous_torque_reading <= cmd_ff)   //Set the flag to 0 when the measured torque reaches the desired setpoint 
            {
                flag = 0;
            }
        }

        if (flag == 0)   //If the torque is not changing to meet a new prescribed torque, filter the data
        {
            _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, (_controller_data->parameters[controller_defs::constant_torque::alpha_idx]) / 100);
        }
        else            //If the torque is changing to meet a new prescribed torque, filter the data
        {
            _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);
        }

       //Set the feed-forward setpoint
        _controller_data->ff_setpoint = cmd_ff;

        float cmd = 0;

        //Perform PID control if desired 
        if (_controller_data->parameters[controller_defs::constant_torque::use_pid_idx] > 0)
        {
            cmd = cmd_ff + _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::constant_torque::p_gain_idx], _controller_data->parameters[controller_defs::constant_torque::i_gain_idx], _controller_data->parameters[controller_defs::constant_torque::d_gain_idx]);
        }
        else
        {
            cmd = cmd_ff;
        }

        previous_command = cmd_ff;

        previous_torque_reading = _controller_data->filtered_torque_reading;

        return cmd;
    }
}

//****************************************************

ElbowMinMax::ElbowMinMax(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("ElbowMinMax::Constructor");
    #endif

    alpha0 = 0.125;     // Initial FSR Smoothing before searchinng for new max/min - smooths FSR sensor signal noise - only used when doing a manual calibration
    alpha1 = 0.2;       // FSR Sensor Smoothing to Finilize input signal - smooths FSR sensor signal noise
    alpha2 = 0.2;       // Torque Sensor Smoothing before entering PID - smooths torque sensor signal noise
    alpha3 = 0.03;      // Setpoint smoothing to reduce abrupt / jerky applications of torque - lower numbers produce a slower response in torque rise time but increase comfort

    cmd = 0;                    //Initalize Command to 0

    //Smoothing Variables
    Smoothed_Sig_Flex = 0;
    Smoothed_Sig_Ext = 0;
    Smoothed_Flex_Max = 0.2;
    Smoothed_Flex_Min = 0.1;
    Smoothed_Ext_Max = 0.2;
    Smoothed_Ext_Min = 0.1;

    starttime = 0;              //Records the start time for the calibration 

    check = 0;                  //Flag for Calibration

    //Angle Parameters
    Angle_Max = 0;
    Angle_Min = 0;
    Angle = 0;

    //State Parameters
    flexState = 0;
    extState = 0;
    nullState = 0;

    previous_setpoint = 0;  //Stores Previous Setpoint 

    SpringEffect = 0;       //Spring Term Contribution
}

float ElbowMinMax::calc_motor_cmd()
{

    alpha3 = _controller_data->parameters[controller_defs::elbow_min_max::FiltStrength_idx] * 0.01;

    float Sig_Flex = _side_data->toe_fsr;   //(Sensor 1)
    float Sig_Ext = _side_data->heel_fsr;   //(Sensor 2)

    //Filter the incoming FSR signals
    Smoothed_Sig_Flex = ((alpha0 * Sig_Flex) + ((1 - alpha0) * Smoothed_Sig_Flex));
    Smoothed_Sig_Ext = ((alpha0 * Sig_Ext) + ((1 - alpha0) * Smoothed_Sig_Ext));

    // ============================================ Start Manual Calibration Loop: FSR & Angle Sensing ============================================ //

    if ((_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 1) || (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 2)) 
    {

        //Initialization loop - This loop includes initialization parameters and should only run once when a manual calibration is requested (i.e. the CaliRequest_idx = 1)
        if (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 1) 
        {

            //Initialize Calibration Start Time
            starttime = millis();

            //Initialize Manual Calibration Parameters    
            Smoothed_Sig_Flex = 0;
            Smoothed_Sig_Ext = 0;
            Smoothed_Flex_Max = 0.2;
            Smoothed_Ext_Max = 0.2;
            Smoothed_Flex_Min = 0.1;
            Smoothed_Ext_Min = 0.1;

            //Flag that calibration has been inialized and plotting FSR signals can begin
            check = 1;
        }

        //Update Calibration Timer
        float timer = millis() - starttime;

        //Check if calibration timer is past 10 seconds, stop is so.
        if (timer > 10000) 
        {
            //Disable this calibration loop, which ensures it get's skipped in the following loop iterations
            _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 0;
        }

        //MIN/MAX Loop - If still in calibration time window (10 seconds), look for new max and min sensor readings
        else if (timer <= 10000) 
        {

            //Set new FSR Sensor Max/Min when found
            Smoothed_Flex_Max = max(Smoothed_Sig_Flex, Smoothed_Flex_Max);
            Smoothed_Flex_Min = min(Smoothed_Sig_Flex, Smoothed_Flex_Min);
            Smoothed_Ext_Max = max(Smoothed_Sig_Ext, Smoothed_Ext_Max);
            Smoothed_Ext_Min = min(Smoothed_Sig_Ext, Smoothed_Ext_Min);


            //Set new Joint Angle Sensor Max/Min when found
            Angle_Max = max(_side_data->ankle.position, Angle_Max);
            Angle_Min = min(_side_data->ankle.position, Angle_Min);

            //Switches the next calibration iteration loop to exclude the initialation loop, which ensures it get's skipped in the following loop iterations
            _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 2;
        }

    }

    // -------------------------------------------- End Manual Calibration Loop: FSR & Angle Sensing --------------------------------------------- //

    // ========================================= Start Automatic Calibration Loop: FSR & Angle Sensing =========================================== //

    //With the default setup (CaliRequest = 3), the program will enter this loop once during startup to determine FSR Min/Max calibration parameters

    else if (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 3) 
    {


        //Right Glove Predefined Calibration Min & Max Parameters
        if (!_joint_data->is_left) 
        {
            Smoothed_Flex_Max = 4.1;
            Smoothed_Ext_Max = 3.9;
            Smoothed_Flex_Min = 0.1;
            Smoothed_Ext_Min = 0.1;
        }

        //Left Glove Predefined Calibration Min & Max Parameters
        if (_joint_data->is_left) 
        {
            Smoothed_Flex_Max = 4;
            Smoothed_Ext_Max = 2.2;
            Smoothed_Flex_Min = 0.1;
            Smoothed_Ext_Min = 0.1;
        }

        //Disable this calibration loop, which ensures it get's skipped in the following loop iterations
        _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 0;

        //Flag that calibration has been completed
        check = 1;

    }

    // ----------------------------------------- End - Automatic Calibration Loop: FSR & Angle Sensing ------------------------------------------------ //

    // ================================================== FSR and ANGLE Normalization ============================================================== //
    
    //If calibration check has been completed and flagged; Creating signals between 0 and 100% or 0 amd 1.
    if (check == 1) 
    {

        //Normalize FSR data - Smooth the signal with an EMA filter
        _controller_data->FlexSense = (Smoothed_Sig_Flex - Smoothed_Flex_Min) / (Smoothed_Flex_Max - Smoothed_Flex_Min);
        _controller_data->ExtenseSense = (Smoothed_Sig_Ext - Smoothed_Ext_Min) / (Smoothed_Ext_Max - Smoothed_Ext_Min);

        //Normalize Angle Sensor Data
        Angle = (_side_data->ankle.position - Angle_Min) / (Angle_Max - Angle_Min);
    }

    //If calibration has not been completed - just filter the reading without normalization
    else 
    {
        Smoothed_Sig_Flex = ((alpha1 * Sig_Flex) + ((1 - alpha1) * Smoothed_Sig_Flex));
        Smoothed_Sig_Ext = ((alpha1 * Sig_Ext) + ((1 - alpha1) * Smoothed_Sig_Ext));
    }

    // =========================================================== Start: State Detection ======================================================================= //

    //Flexion Condition
    if (_controller_data->FlexSense > (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::DigitFSR_threshold_idx]) && _controller_data->FlexSense > _controller_data->ExtenseSense) 
    {

        _controller_data->ff_setpoint = _controller_data->parameters[controller_defs::elbow_min_max::FLEXamplitude_idx];

        //Update State booleens for torque modifier loop
        flexState = 1;
        extState = 0;
        nullState = 0;

    }

    //Extension Condition
    else if (_controller_data->ExtenseSense > (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::PalmFSR_threshold_idx])) 
    { 

        _controller_data->ff_setpoint = -1 * _controller_data->parameters[controller_defs::elbow_min_max::EXTamplitude_idx];

        //Update State booleens for torque modifier loop
        flexState = 0;
        extState = 1;
        nullState = 0;
    }


    //Zero Torque Condition
    else if (_controller_data->FlexSense < (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::DigitFSR_LOWthreshold_idx]) && _controller_data->ExtenseSense < (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::PalmFSR_LOWthreshold_idx])) 
    {

        _controller_data->ff_setpoint = 0;

        //Update State booleens for torque modifier loop
        flexState = 0;
        extState = 0;
        nullState = 1;
    }

    //Just incase condition
    else 
    {
        _controller_data->ff_setpoint = previous_setpoint;
    }

    //Update previous setpoint for the "Just incase condition"
    previous_setpoint = _controller_data->ff_setpoint;

    // --------------------------------------------------------------- End: State Detection ----------------------------------------------------------------------------- //

    // =========================================================== Start: Torque Profile Modifier ======================================================================= //

    //Setpoint modification - Spring Torque Profile
    if (_controller_data->parameters[controller_defs::elbow_min_max::TrqProfile_idx] == 1) 
    {
        //Flexion Modifier
        if (flexState) 
        {
            //This equation came from a polyfit in excel that maps the desired increase in torque with respect to the normalized angle - specifically for flexion          
            SpringEffect = ((3.1702 * pow(Angle, 3)) - (4.6572 * pow(Angle, 2)) + (0.49 * Angle) + 1.0006) * _controller_data->parameters[controller_defs::elbow_min_max::SpringPkTorque_idx];

            //This sums the selected setpoint (12 Nm) with the torque modifier, to determine the desired setpoint
            _controller_data->ff_setpoint = SpringEffect + _controller_data->ff_setpoint;
        }

        //Extension Modifier
        else if (extState) 
        {
            //This equation came from a polyfit in excel that maps the desired increase in torque with respect to the normalized angle - specifically for extension
            SpringEffect = ((-3.1702 * pow(Angle, 3)) + (4.8521 * pow(Angle, 2)) - (0.6858 * Angle) + (0.0034)) * _controller_data->parameters[controller_defs::elbow_min_max::SpringPkTorque_idx];

            //This sums the selected setpoint (12 Nm) with the torque modifier, to determine the desired setpoint
            _controller_data->ff_setpoint = -1 * SpringEffect + _controller_data->ff_setpoint;
        }

        //Otherwise
        else 
        {
            _controller_data->ff_setpoint = 0;
        }
    }

    // ================================================================== End: Torque Profile Modifier ======================================================================= //

    //Get Filtered torque reading, and setpoint for PID input
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, alpha2);
    _controller_data->filtered_setpoint = utils::ewma(_controller_data->ff_setpoint, _controller_data->filtered_setpoint, alpha3);   // Was 0.01 for most trials

    //Saftey Feature - Saturate Torque Setpoint at the max if the modifier gets a wild angle reading (Max Torque Limit)
    if (_controller_data->filtered_setpoint < -1 * _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx])
    {
        _controller_data->filtered_setpoint = -1 * _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx];
    }

    if (_controller_data->filtered_setpoint > _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx])
    {
        _controller_data->filtered_setpoint = _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx];
    }

    //Get motor command based on PID
    cmd = _controller_data->filtered_setpoint + _pid(_controller_data->filtered_setpoint, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::elbow_min_max::P_gain_idx], _controller_data->parameters[controller_defs::elbow_min_max::I_gain_idx], _controller_data->parameters[controller_defs::elbow_min_max::D_gain_idx]);       //originally, (10, 0, 200)

    return cmd;
}

//****************************************************

CalibrManager::CalibrManager(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        Serial.println("CalibrManager::CalibrManager");
    #endif

}

float CalibrManager::calc_motor_cmd()//The calibration manager "controller" is a self-test tool designed for exo developers to verify the exoskeleton's status. By default, this controller does not use feedback torque control.
{
	float cmd;

	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
	
    if (active_trial)
    {
        if (_joint_data->is_left)
        {
            Serial.print("\nLeft angle: ");
            Serial.print(_side_data->ankle.joint_position);
            Serial.print("  |  Left torque: ");
            Serial.print(_joint_data->torque_reading);
            cmd = _controller_data->parameters[controller_defs::calibr_manager::calibr_cmd];
            cmd = 3.5;
            Serial.print("  |  Left cmd: ");
            Serial.print(cmd);
        }
        else
        {
            Serial.print("  |  Right angle: ");
            Serial.print(_side_data->ankle.joint_position);
            Serial.print("  |  Right torque: ");
            Serial.print(_joint_data->torque_reading);
            cmd = 3.5;
            Serial.print("  |  Right cmd: ");
            Serial.print(cmd);
            Serial.print("  |  doToeRefinement: ");
            Serial.print(String(_side_data->do_calibration_refinement_toe_fsr));
            Serial.print("  |  Exo status: ");
            uint16_t exo_status = _data->get_status();
            Serial.print(String(exo_status));
        }
    }
		
    return cmd;
	
}

//****************************************************

Chirp::Chirp(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("Chirp::Chirp");
#endif

    start_flag = 1;
    start_time = 0;
    current_time = 0;
    previous_amplitude = 0;
}

float Chirp::calc_motor_cmd()
{
    if (_joint_data->is_left)
    {
        float cmd_ff = 0;

        if (start_flag == 1)                    //If this is the first instance of the controller
        {
            start_time = millis();              //Record start time
            start_flag = 0;                     //Disable Flag -> tells us that it is no longer the first instance 
        }

        current_time = millis();                //Records the current time

        float t = (current_time - start_time) / 1000;    //Measure of the time since the start, converts to seconds. 

        float amplitude = _controller_data->parameters[controller_defs::chirp::amplitude_idx];              //Amplitude of the sine wave
        float start_frequency = _controller_data->parameters[controller_defs::chirp::start_frequency_idx];  //Start frequency of the sine wave
        float end_frequency = _controller_data->parameters[controller_defs::chirp::end_frequency_idx];      //End frequency of the sine wave
        float duration = _controller_data->parameters[controller_defs::chirp::duration_idx];                //Duration of the controller
        float yshift = _controller_data->parameters[controller_defs::chirp::yshift_idx];                    //Shifts the center of the sine wave

        float phi = (0 * M_PI / 2);

        float frequency = 0;

        if (t <= duration)                                                                                  //If time is less than the set duration
        {
            frequency = start_frequency + ((end_frequency - start_frequency) * (t / duration));       //Frequency, linearly increases with each iteration of the controller.
            cmd_ff = amplitude * sin(2 * M_PI * frequency * t + phi) + yshift;                                    //Torque command as a sine wave

            if (std::isnan(frequency))                  //If it detects Nan, sets the values to 0, prevents the exo from throwing a fit.
            {
                frequency = start_frequency;
                cmd_ff = 0;
            }
        }

        if (previous_amplitude == 0 && amplitude != 0)  //Flag to restart sine wave without exiting the controller (switch amplitude to zero and then switch it back to what amplitude you want restarts the wave). 
        {
            start_flag = 1;
        }

        previous_amplitude = amplitude;                 //Stores the amplitude to be used in next iteration of the controller. 

        _controller_data->ff_setpoint = cmd_ff;
        _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);

        //PID
        float cmd = cmd_ff +(_controller_data->parameters[controller_defs::chirp::pid_flag_idx]
           ? _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::chirp::p_gain_idx], _controller_data->parameters[controller_defs::chirp::i_gain_idx], _controller_data->parameters[controller_defs::chirp::d_gain_idx])
           : 0);

        //uint16_t exo_status = _data->get_status();

        //bool active_trial = (exo_status == status_defs::messages::trial_on) || (exo_status == status_defs::messages::fsr_calibration) || (exo_status == status_defs::messages::fsr_refinement);

        //if (active_trial)
        //{

        //    if (_joint_data->is_left)
        //    {

        //        Serial.print(_controller_data->ff_setpoint);
        //        Serial.print(',');
        //        Serial.print(100);
        //        Serial.print("\n");

        //        Serial.print(_controller_data->filtered_torque_reading);
        //        Serial.print(',');
        //        Serial.print(200);
        //        Serial.print("\n");

        //        Serial.print(t * 1000);
        //        Serial.print(',');
        //        Serial.print(300);
        //        Serial.print("\n");

        //        Serial.print(frequency);
        //        Serial.print(',');
        //        Serial.print(400);
        //        Serial.print("\n");

        //    }
        //}

        return cmd;
    }
    else
    {
        return 0;
    }

}

//****************************************************

Step::Step(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("Step::Step");
#endif

    //Initializes Values
    n = 1;
    start_flag = 1;
    start_time = 0;
    cmd_ff = 0;
    end_time = 0;

    previous_command = 0;
    previous_torque_reading = 0;
    flag = 0;
    difference = 0;
    turn = 0;
    flag_time = 0;
    change_time = 0;

}

float Step::calc_motor_cmd()
{
    
    float Amplitude = _controller_data->parameters[controller_defs::step::amplitude_idx];           //Magnitude of Step Response
    float Duration = _controller_data->parameters[controller_defs::step::duration_idx];             //Duration of Step Response
    int Repetitions = _controller_data->parameters[controller_defs::step::repetitions_idx];         //Number of Step Responses
    float Spacing = _controller_data->parameters[controller_defs::step::spacing_idx];               //Time Between Each Step Response

    float tt = 0;

    if (n <= Repetitions)                                          //If we are less than the number of desired repetitions
    {
        if (start_flag == 1)                                        //If this is the start of this loop
        {
            start_time = millis();                                  //Record the start time
            start_flag = 0;                                         //Set the flag so that we don't continue to record start time
        }

        float current_time = millis();                              //Measure the current time

        tt = (current_time - start_time) / 1000;            //Determine the time since the begining of the control iteration, converted to seconds

        if (tt <= Duration)                                       //If the time is less than the desired duration of the step
        {
            cmd_ff = Amplitude;                                        //Apply a torque at the desired magnitude 
        }
        else
        {
            cmd_ff = 0;                                                //Set the torque to 0

            if (previous_time <= Duration && tt > Duration)       //Calculate the time that the amplitude ended
            {
                end_time = millis();
            }

            if (((current_time - end_time)/1000) >= Spacing)        //If the time since ending the step has exceeded our desired spacing
            {
                n = n + 1;                                          //Update the iteration count
                start_flag = 1;                                     //Update the start flag to get a new start time and begin a new cycle
            }
        }

        previous_time = tt;                                       //Record time to be used as previous time in next loop. 

    }
    else
    {
        cmd_ff = 0;
    }

    //Real-Time Torque Filtering if Using Torque Transducer
    //if (cmd_ff != previous_command)
    //{
    //    flag = 1;
    //    difference = cmd_ff - previous_command;
    //    turn = millis();;
    //}

    //if (difference > 0)
    //{
    //    if (flag == 1 && (previous_torque_reading >=  0.9 * cmd_ff))
    //    {
    //        flag = 0;
    //    }
    //}

    //if (difference < 0)
    //{
    //    //if (flag == 1 && (previous_torque_reading <= (1 - 0.9) * cmd_ff))
    //    //{
    //    //    flag = 0;
    //    //}
    //}

    //if (flag == 0)
    //{
    //    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, (_controller_data->parameters[controller_defs::step::alpha_idx] / 100));
    //}
    //else
    //{
    //    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);
    //}


    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, (_controller_data->parameters[controller_defs::step::alpha_idx])/100);

    _controller_data->ff_setpoint = cmd_ff;

    float cmd = cmd_ff;

    if (_controller_data->parameters[controller_defs::step::pid_flag_idx] > 0)
    {
        cmd = cmd_ff + _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::step::p_gain_idx], _controller_data->parameters[controller_defs::step::i_gain_idx], _controller_data->parameters[controller_defs::step::d_gain_idx]);
    }
    else
    {
        cmd = cmd_ff;
    }

    previous_command = cmd_ff;

    previous_torque_reading = _controller_data->filtered_torque_reading;

    uint16_t exo_status = _data->get_status();
    
    bool active_trial = (exo_status == status_defs::messages::trial_on) || (exo_status == status_defs::messages::fsr_calibration) || (exo_status == status_defs::messages::fsr_refinement);

    //if (active_trial)
    //{
    //    if (!_joint_data->is_left)
    //    {
    //        Serial.print(_controller_data->ff_setpoint);
    //        Serial.print(',');
    //        Serial.print(100);
    //        Serial.print("\n");

    //        Serial.print(_controller_data->filtered_torque_reading);
    //        Serial.print(',');
    //        Serial.print(200);
    //        Serial.print("\n");

    //        Serial.print(tt*1000);
    //        Serial.print(',');
    //        Serial.print(300);
    //        Serial.print("\n");
    //    }
    //}

    return cmd;
}

//****************************************************
SPV2::SPV2(config_defs::joint_id id, ExoData* exo_data)
: _Controller(id, exo_data)
{
    #ifdef CONTROLLER_DEBUG
        logger::println("SPV2::Constructor");
    #endif
}
float SPV2::calc_motor_cmd()
{
	
	
	// Calculate Generic Contribution
	float plantar_setpoint = _controller_data->parameters[controller_defs::spv2::plantar_scaling];
	float dorsi_setpoint = _controller_data->parameters[controller_defs::spv2::dorsi_scaling];
	float threshold = constrain(_controller_data->parameters[controller_defs::spv2::timing_threshold]/100, 0, 99);
	float percent_grf = constrain(_side_data->toe_fsr, 0, 1.2);
	float percent_grf_heel = constrain(_side_data->heel_fsr, 0, 1.2);
	float cmd_ff = _pjmc_generic(percent_grf, threshold, dorsi_setpoint, -plantar_setpoint);
	if (!_joint_data->is_left){
		Serial.print("\nRunning SPV2...");
		Serial.print(cmd_ff);
	}
        // low pass filter on torque_reading
    const float torque = _joint_data->torque_reading;
	// const float torque = _joint_data->torque_reading_microSD;
    const float alpha = 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, 
            _controller_data->filtered_torque_reading, alpha);
	
		float cmd;
		if (!_joint_data->is_left){
			if (cmd_ff < -6) {
				cmd = cmd_ff + _pid(cmd_ff, _controller_data->filtered_torque_reading,
					20 * _controller_data->parameters[controller_defs::spv2::kp],
					80 * _controller_data->parameters[controller_defs::spv2::ki], 
					20 * _controller_data->parameters[controller_defs::spv2::kd]);
			}
			else {
				cmd = cmd_ff + _pid(cmd_ff, _controller_data->filtered_torque_reading,
					10 * _controller_data->parameters[controller_defs::spv2::kp],
					80 * _controller_data->parameters[controller_defs::spv2::ki], 
					20 * _controller_data->parameters[controller_defs::spv2::kd]);
			}
		}
		else {
			cmd = 0;
		}
			
	
	_controller_data->ff_setpoint = cmd_ff; 
	_controller_data->setpoint = cmd;
    _controller_data->filtered_setpoint = cmd;

    #ifdef CONTROLLER_DEBUG
    logger::println("SPV2::calc_motor_cmd : stop");
    #endif
	
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

	int servoOutput;	
	bool servo_switch = _controller_data->parameters[controller_defs::spv2::do_use_servo];
	float servo_fsr_threshold = 0.01 * _controller_data->parameters[controller_defs::spv2::fsr_servo_threshold];
	uint8_t servo_home = _controller_data->parameters[controller_defs::spv2::servo_origin];
	uint8_t servo_target = _controller_data->parameters[controller_defs::spv2::servo_terminal];
		
	// if (!_joint_data->is_left) {
		// Serial.print("\nheel fsr threshold: ");
		// Serial.print(_controller_data->parameters[controller_defs::spv2::fsr_servo_threshold]);
	// }
	if (_data->user_paused || !active_trial)
	{
		if (!_joint_data->is_left) {
			
			// Serial.print("\nInitializing...   |  Heel FSR: ");
			// Serial.print(analogRead(A3));
			servoOutput = _servo_runner(27, 3, servo_target, servo_home);
		}

		
	}
	else {
		if (!servo_switch) {
			servoOutput = _servo_runner(27, 3, servo_target, servo_home);
		}
		if (exo_status == status_defs::messages::fsr_refinement) {
			if (!_joint_data->is_left) {
				// Serial.print("\npercent_grf_heel: ");
				// Serial.print(percent_grf_heel);
//Servo movement
//// When does the arm go DOWN?////
				//reset only after toe FSR drops below a threshold
				if ((percent_grf_heel + percent_grf > servo_fsr_threshold) && (!_controller_data->servo_did_go_down)) {
					if (servo_switch) {
					_controller_data->servo_get_ready = true;
					_controller_data->servo_departure_time = millis();
					}
				}
				if (percent_grf_heel + percent_grf < servo_fsr_threshold) {
					_controller_data->servo_did_go_down = false;
				}
				
				if (_controller_data->servo_get_ready){
					if (millis() - _controller_data->servo_departure_time < 200) {
						servoOutput = _servo_runner(27, 3, servo_home, servo_target);//servo goes to the target position (DOWN)
						_controller_data->servo_did_go_down = true;
					}
					else {	
						_controller_data->servo_get_ready = false;
					}
				}
				else {
					servoOutput = _servo_runner(27, 3, servo_target, servo_home);//servo goes back to the home position (UP)
				}
			}
		}	
	}
////Turn of the motor////
	//When do we turn the motor OFF?
	if (!_joint_data->is_left) {
		//limit post-PID motor command for dorsiflexion torque
		if (cmd_ff >= 0) {
			cmd = constrain(cmd, -300, 300);
		}
		
		//if ((cmd_ff<_controller_data->parameters[controller_defs::propulsive_assistive::dorsi_scaling])&&((_controller_data->filtered_torque_reading - cmd_ff) < 0)) {
		if ((servo_switch) && (percent_grf_heel > servo_fsr_threshold) && (_controller_data->filtered_torque_reading - cmd_ff) < 0){
			cmd = _pid(0, 0, 0, 0, 0);//reset the PID error sum by sending a 0 I gain
			cmd = 0;//send 0 Nm torque command to "turn off" the motor to extend the battery life
			}
	}
		
		
		// if (maxon_standby) {
			// _controller_data->plotting_scalar = -1;
			//return;
		// }
		// else {
			// _controller_data->plotting_scalar = 1;
		// }
		// if (_joint_data->is_left) {
			//analogWrite(A8,cmdMaxon);//Left motor: A8; Right motor: A9
		// }
		// else {
			// analogWrite(A9,cmdMaxon);//Left motor: A8; Right motor: A9
		// }
	
	if (!_joint_data->is_left) {
		// Serial.print("\ncmd = ");
		// Serial.print(cmd);
		// Serial.print("  |  filtered_torque_reading - cmd_ff: ");
		// Serial.print(_controller_data->filtered_torque_reading - cmd_ff);
		return cmd;
	}
	else
	{
		return 0;
	}
}
#endif