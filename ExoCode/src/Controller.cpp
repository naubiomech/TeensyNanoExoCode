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

_Controller::_Controller(config_defs::joint_id id, ExoData* exo_data)
{
    _id = id;
    _data = exo_data;
    
    _t_helper = Time_Helper::get_instance();
    _t_helper_context = _t_helper->generate_new_context();
    _t_helper_delta_t = 0;
    
    // We just need to know the side to point at the right data location so it is only for the constructor
    bool is_left = utils::get_is_left(_id);
    
    #ifdef CONTROLLER_DEBUG
        logger::print(is_left ? "Left " : "Right ");
    #endif 
    
    _integral_val = 0;
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

    //Added a pointer to the leg data as most controllers will need to access info specific to their leg.
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

float _Controller::_cf_mfac(float reference, float current_measurement)
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
 
float _Controller::_pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain)
{
	//Serial.print("\nPID Running...");
	/* _controller_data->currentTime = millis();
	_controller_data->itrTime = _controller_data->currentTime - _controller_data->previousTime;
	_controller_data->previousTime = _controller_data->currentTime; */
	//measurement = 55;
	//measurement = min(measurement, 35);
	//measurement = max(measurement, -35);
	/* if (!_leg_data->is_left) {
		/* Serial.print("\nIteration time: " + String(_controller_data->itrTime) + "Previous Time: " + String(_controller_data->previousTime) + " Current Time: " + String(_controller_data->currentTime)); */
		//Serial.print("\nPre-measurement: " + String(measurement));
		//measurement = min(measurement, 35);
		//measurement = max(measurement, -35);
		//Serial.print(". Post-measurement: " + String(measurement));
	//} */
	#if USE_ANGLE_SENSORS
	//Serial.println("USE_ANGLE_SENSORS");
	#endif

	//if ((_leg_data->do_calibration_refinement_toe_fsr) || (_leg_data->do_calibration_toe_fsr)) {
		// Serial.print("\nCalibrating toe FSR... PID returns 0.");
		//return 0;
	//}
	//else if (_leg_data->ankle.joint_position > 0.9) {
		/* Serial.print("\nankle.joint_position at ");
		Serial.print(String(_leg_data->ankle.joint_position));
		Serial.print(". PID is off."); */
	//	return 3.5;
	//}
	//else if (_leg_data->ankle.joint_position < 0.1) {
		/* Serial.print("\nankle.joint_position at ");
		Serial.print(String(_leg_data->ankle.joint_position));
		Serial.print(". PID is off."); */
	//	return -3.5;
	//}
	/* bool active_trial;
	uint16_t exo_status = _data->get_status(); */
	
	/* if (exo_status == 6) {
		active_trial = true;
	}
	if (active_trial) {
		if (abs(measurement - cmd) > 10) {
			_controller_data->pausePid = true;
		}
	} */
	/* Serial.print("\nExo status: " + String(exo_status) + "pausePid = " + String(_controller_data->pausePid) + "iPidHiTorque = " + String(_controller_data->iPidHiTorque)); */
	/* if (_controller_data->pausePid) {
		if (_controller_data->iPidHiTorque < 200) {
			_controller_data->iPidHiTorque = _controller_data->iPidHiTorque + 1;
		return -5;
		}
		else {
			_controller_data->iPidHiTorque = 0;
			_controller_data->pausePid = false;
		}
	} */
	
	
	/* if (measurement > _controller_data->maxTorqueCache) {
		_controller_data->maxTorqueCache = measurement;
	}
	if (measurement < _controller_data->minTorqueCache) {
		_controller_data->minTorqueCache = measurement;
	}
	if (_controller_data->maxTorqueCache - _controller_data->minTorqueCache > 50) {
		return cmd;
	} */
	
    //check if time is ok
    bool time_good = true;
    if (_t_helper->tick(_t_helper_context) > ((float) 1/LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)))
    {
        time_good = false;
    }
    float now = micros();
	
/* 	uint16_t exo_status = _data->get_status();
	bool status_satisfied = (exo_status == status_defs::messages::fsr_refinement);
	if (status_satisfied) {
		_pid_on = 1;
	}
	if (!(_pid_on == 1)) {
		return 0;
	} */
	//if (_data->user_paused || !trial_begun) {
	/* if (trial_begun) {
		_do_wait = 1;
		_previously_paused = 1;
		Serial.print("\nNot started yet. Exo status: ");
		Serial.print(String(exo_status));
		return 0;
	}
	else {
		if (_previously_paused == 1) {
			_previously_paused = 0;
			_start_time = millis();
		}
		if ((now - _start_time) > 5000) {
			_do_wait = 0;
		}
		else {
			_do_wait = 1;
		}
	}
	if (_do_wait == 1) {
		Serial.print("\nHolding...");
		return 0;
	} */

    //TODO: Test dynamic time step
    float dt = (now - _prev_pid_time) * 1000000;

    float error_val = cmd - measurement;  
    //_integral_val += error_val / LOOP_FREQ_HZ; 

    if (i_gain != 0)
    {
        _pid_error_sum += error_val / LOOP_FREQ_HZ;
    }
    else
    {
        _pid_error_sum = 0;
    }


    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || (exo_status == status_defs::messages::fsr_calibration) || (exo_status == status_defs::messages::fsr_refinement);

    if (_data->user_paused || !active_trial)
    {
        _pid_error_sum = 0;
    }

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
    float i = i_gain * _pid_error_sum; //float i = i_gain * _integral_val;  // resetting _integral_val was crashing the system 
    float d = d_gain * de_dt; 

    //Serial.print("I = ");
    //Serial.print(i);
    //Serial.print("\n");
    
	//float pNd = min(p + d, 25);
	//pNd = max(p + d, -25);
    return p + i + d;
	//return pNd;
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

void PropulsiveAssistive::_update_reference_angles(LegData* leg_data, ControllerData* controller_data, float percent_grf, float percent_grf_heel)
{
    const float threshold = controller_data->parameters[controller_defs::propulsive_assistive::timing_threshold]/100;
    // When the percent_grf passes the threshold, update the reference angle
    const bool should_update = (percent_grf > controller_data->toeFsrThreshold) && !controller_data->reference_angle_updated;
    const bool should_capture_level_entrance = leg_data->do_calibration_refinement_toe_fsr && !leg_data->do_calibration_toe_fsr;
    const bool should_reset_level_entrance_angle = controller_data->prev_calibrate_level_entrance < should_capture_level_entrance;

    if (should_reset_level_entrance_angle)
    {
        controller_data->level_entrance_angle = 0.5;
    }

    if (should_update)
    {
        if (should_capture_level_entrance)
        {
            controller_data->level_entrance_angle = utils::ewma(leg_data->ankle.joint_position, 
                controller_data->level_entrance_angle, controller_data->cal_level_entrance_angle_alpha);
        }

        controller_data->reference_angle_updated = true;
        controller_data->reference_angle = leg_data->ankle.joint_position;

        if (leg_data->is_left)
        {
            // Serial.print("level_entrance_angle: ");
            // Serial.println(controller_data->level_entrance_angle);
            // Serial.print("joint_position: ");
            // Serial.println(leg_data->ankle.joint_position);
            // Serial.print("reference_angle: ");
            // Serial.println(controller_data->reference_angle);
            // // Print delimiting newline
            // Serial.println();
        }
        
        //controller_data->reference_angle_offset = leg_data->ankle.joint_global_angle;
    }

    // When the percent_grf drops below the threshold, reset the reference angle updated flag and expire the reference angle
    const bool should_reset = (percent_grf < controller_data->toeFsrThreshold) && controller_data->reference_angle_updated;
    if (should_reset)
    {
        controller_data->reference_angle_updated = false;
        controller_data->reference_angle = 0;
        controller_data->reference_angle_offset = 0;
    }

    controller_data->prev_calibrate_level_entrance = should_capture_level_entrance;
}

void PropulsiveAssistive::_capture_neutral_angle(LegData* leg_data, ControllerData* controller_data)
{
    // On the start of torque calibration reset the neutral angle
    if (controller_data->prev_calibrate_trq_sensor < leg_data->ankle.calibrate_torque_sensor)
    {
        controller_data->neutral_angle = leg_data->ankle.joint_position;
    }

    if (leg_data->ankle.calibrate_torque_sensor) 
    {
        // Update the neutral angle with an ema filter
        controller_data->neutral_angle = utils::ewma(leg_data->ankle.joint_position, 
            controller_data->neutral_angle, controller_data->cal_neutral_angle_alpha);
    }

    controller_data->prev_calibrate_trq_sensor = leg_data->ankle.calibrate_torque_sensor;
}

void PropulsiveAssistive::_grf_threshold_dynamic_tuner(LegData* leg_data, ControllerData* controller_data, float threshold, float percent_grf_heel)
{
	//if it's swing phase, set wait4HiHeelFSR to True, and increase the toeFSR threshold
	//when wait4HiHeelFSR is true and heelFSR > a pre-defined threshold, reduce the toeFSR threshold
	//when it's stance phase, set wait4HiHeelFSR to False
	if (!leg_data->toe_stance) {
		controller_data->wait4HiHeelFSR = true;
	}
	else {
		controller_data->wait4HiHeelFSR = false;
		controller_data->toeFsrThreshold = threshold*0.01;
	}
	if (controller_data->wait4HiHeelFSR) {
		if (percent_grf_heel > threshold) {
			controller_data->toeFsrThreshold = threshold*0.1;
		}
		else {
			controller_data->toeFsrThreshold = threshold;
		}
	}
}

void PropulsiveAssistive::_plantar_setpoint_adjuster(LegData* leg_data, ControllerData* controller_data, float pjmcSpringDamper)
{
	if(_leg_data->toe_stance) {
			
		//Update peak values
		_controller_data->maxPjmcSpringDamper = max(_controller_data->maxPjmcSpringDamper, pjmcSpringDamper);
		_controller_data->wasStance = true;
	}
	else {
		if (_controller_data->wasStance) {
			_controller_data->prevMaxPjmcSpringDamper = _controller_data->maxPjmcSpringDamper;
			_controller_data->maxPjmcSpringDamper = 0;
			
			if (_controller_data->prevMaxPjmcSpringDamper < _controller_data->parameters[controller_defs::propulsive_assistive::plantar_scaling]) {
			_controller_data->setpoint2use ++;
			}
			else {
				_controller_data->setpoint2use --;
			}
			_controller_data->setpoint2use = min(_controller_data->setpoint2use, 35);
			_controller_data->setpoint2use = max(_controller_data->setpoint2use, 0);
			_controller_data->wasStance = false;
		}
	}
	if (_controller_data->prevMaxPjmcSpringDamper == 0) {
		_controller_data->setpoint2use = _controller_data->parameters[controller_defs::propulsive_assistive::plantar_scaling];
	}
}


float PropulsiveAssistive::calc_motor_cmd()
{
    #ifdef CONTROLLER_DEBUG
    logger::println("PropulsiveAssistive::calc_motor_cmd : start");
    #endif
    static const float sigmoid_exp_scalar{50.0f};

    // Calculate Generic Contribution
	float plantar_setpoint = 0;
    if (_controller_data->parameters[controller_defs::propulsive_assistive::turn_on_peak_limiter]) {
		plantar_setpoint = _controller_data->setpoint2use;
	}
	else {
		plantar_setpoint = _controller_data->parameters[controller_defs::propulsive_assistive::plantar_scaling];
		_controller_data->setpoint2use = plantar_setpoint;
	}
	const float dorsi_setpoint = -_controller_data->parameters[controller_defs::propulsive_assistive::dorsi_scaling];
    const float threshold = _controller_data->parameters[controller_defs::propulsive_assistive::timing_threshold]/100;
    const float percent_grf = min(_leg_data->toe_fsr, 1);
	const float percent_grf_heel = min(_leg_data->heel_fsr, 1);
    const float slope = (plantar_setpoint - dorsi_setpoint)/(1 - threshold);
    const float generic = max(((slope*(percent_grf - threshold)) + dorsi_setpoint), dorsi_setpoint);//Stateless "PJMC" stateless
	_controller_data->stateless_pjmc_term = generic;

    // Assistive Contribution (a.k.a: Suspension; this term consists of a "Spring term" and a "Damper term" as the suspension)
    _capture_neutral_angle(_leg_data, _controller_data);
	_grf_threshold_dynamic_tuner(_leg_data, _controller_data, threshold, percent_grf_heel);
    _update_reference_angles(_leg_data, _controller_data, percent_grf, percent_grf_heel);//When current toe FSR > set threshold, use the current ankle angle as the "reference angle"
    const float k = 0.01 * _controller_data->parameters[controller_defs::propulsive_assistive::spring_stiffness];
    const float b = 0.01 * _controller_data->parameters[controller_defs::propulsive_assistive::damping];
    const float equilibrium_angle_offset = _controller_data->parameters[controller_defs::propulsive_assistive::neutral_angle]/100;
    const float deviation_from_level = (_controller_data->reference_angle - _controller_data->level_entrance_angle);
    const float delta = _controller_data->reference_angle + deviation_from_level - _leg_data->ankle.joint_position + equilibrium_angle_offset;//describes the amount of dorsi flexion since toe FSR > set threshold (negative at more plantarflexed angles)
    const float assistive = max(k*delta - b*_leg_data->ankle.joint_velocity, 0);//Dorsi velocity: Negative
    // print assistive 
    // Serial.print("assistive: ");
    // Serial.println(assistive);
    // Use a tuned sigmoid to squelch the spring output during the 'swing' phase
    const float squelch_offset = -(1.5*_controller_data->toeFsrThreshold); // 1.5 ensures that the spring activates after the new angle is captured
    const float grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))) / 
            (exp(sigmoid_exp_scalar*(percent_grf+squelch_offset))+1);
    const float squelched_supportive_term = assistive*grf_squelch_multiplier;//Finalized suspension term
    // low pass the squelched supportive term
    _controller_data->filtered_squelched_supportive_term = utils::ewma(squelched_supportive_term, 
            _controller_data->filtered_squelched_supportive_term, 0.075);

    // Propulsive Contribution
    const float kProp = 0.01 * _controller_data->parameters[controller_defs::propulsive_assistive::propulsive_gain];
    const float saturated_velocity = _leg_data->ankle.joint_velocity > 0 ? _leg_data->ankle.joint_velocity:0;
    const float propulsive = kProp*saturated_velocity;
    // Use a symmetric sigmoid to squelch the propulsive term
    const float propulsive_squelch_offset = -1.1 + threshold;
    const float propulsive_grf_squelch_multiplier = (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))) / 
            (exp(sigmoid_exp_scalar*(percent_grf+propulsive_squelch_offset))+1);
    const float squelched_propulsive_term = propulsive*propulsive_grf_squelch_multiplier;
    
	// PJMC reducer
	if (_controller_data->parameters[controller_defs::propulsive_assistive::turn_on_peak_limiter]) {
		_plantar_setpoint_adjuster(_leg_data, _controller_data, _controller_data->filtered_squelched_supportive_term+generic);
	}
	
    // Sum for ff
    const float cmd_ff = -(_controller_data->filtered_squelched_supportive_term+generic+squelched_propulsive_term);//According to the new motor command direction definitions, at the ankle, positive for dorsi, and negative for plantar.

    // low pass filter on torque_reading
    const float torque = _joint_data->torque_reading;
    const float alpha = 0.5;
    _controller_data->filtered_torque_reading = utils::ewma(torque, 
            _controller_data->filtered_torque_reading, alpha);

    // close the loop
    float cmd = _pid(cmd_ff, _controller_data->filtered_torque_reading,
            _controller_data->parameters[controller_defs::propulsive_assistive::kp],
            0, 
            _controller_data->parameters[controller_defs::propulsive_assistive::kd]);
			

	cmd = min(cmd, cmd_ff + 35);
	cmd = max(cmd, cmd_ff - 35);
	
	
	/* if (!_leg_data->is_left) {
		Serial.print("\nRight ankle.joint_position at ");
		Serial.print(String(_leg_data->ankle.joint_position));
		
		//Serial.print("\ncmdCacheItr: " + String(_controller_data->cmdCacheItr) + "previousMaxCmdCache: " + String(_controller_data->previousMaxCmdCache) + "currentMaxCmdCache: " + String(_controller_data->currentMaxCmdCache));
		
		//Serial.print("\nRight user defined setpoint: " + String(_controller_data->parameters[controller_defs::propulsive_assistive::plantar_scaling]) + "  |  adjusted setpoint: " + String(cmd_ff));
	} 
	else {
		Serial.print("  | Left ankle.joint_position at ");
		Serial.print(String(_leg_data->ankle.joint_position));
		
	} */
	
	
	
    // update plotting variables
    _controller_data->ff_setpoint = cmd_ff;
	_controller_data->setpoint = cmd;
    _controller_data->filtered_setpoint = squelched_propulsive_term;

    // Print the propulsive terms for the right leg
  /*   if (!_leg_data->is_left)
    {
        // Serial.print("Vel:" + String(_leg_data->ankle.joint_velocity) + "\t");
        // Serial.print("kProp:" + String(kProp) + "\t");
        // Serial.print("SQPM:" + String(propulsive_grf_squelch_multiplier) + "\t");
        // Serial.print("SQP:" + String(squelched_propulsive_term) + "\n");
		//Serial.print("Right Toe FSR Thre: " + String(_controller_data->toeFsrThreshold) + "  |  Assistive: " + String(assistive) + "  |  Reference ang: " + String(_controller_data->reference_angle) + "  |  ToeFSR: " + String(percent_grf) + "\n");
        
    } */

    #ifdef CONTROLLER_DEBUG
    logger::println("PropulsiveAssistive::calc_motor_cmd : stop");
    #endif
	/* if (!_leg_data->is_left) {
	Serial.print("\ncmd_ff: " + String(cmd_ff) + ", raw torque reading: " + String(_controller_data->filtered_torque_reading) + " cmd: " + String(cmd)); 
	} */
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
    _stance_thresholds_left.first = exo_data->left_leg.toe_fsr_lower_threshold;
    _stance_thresholds_left.second = exo_data->left_leg.toe_fsr_upper_threshold;
    _stance_thresholds_right.first = exo_data->right_leg.toe_fsr_lower_threshold;
    _stance_thresholds_right.second = exo_data->right_leg.toe_fsr_upper_threshold;
}

float ProportionalJointMoment::calc_motor_cmd()
{
	
    #ifdef CONTROLLER_DEBUG
        logger::println("ProportionalJointMoment::calc_motor_cmd : start");
    #endif

    float cmd_ff = 0;

    /* If the toe is on the ground, calculate the feed-forward command */
    if (_leg_data->toe_stance) 
    {
        /* Scale the fsr values so the controller outputs zero feed forward when the FSR value is at the threshold */ 
        float threshold = _leg_data->toe_fsr_upper_threshold;       /* Get the upper threshold for the FSR to be considered in stance */
        float fsr = min(_leg_data->toe_fsr, 1.2);                   /* Stores a saturated FSR signal from the device, saturates in order to avoid producing a large torque. */
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
    if (_leg_data->toe_stance) 
    {
        /* Store the current command and measured torque into set variables */
		const float new_torque = _controller_data->filtered_torque_reading;
		const float new_ff = cmd_ff;
		
        /* Compares to previous maxiums during this step and overwrights those variables if current is larger. */
        _controller_data->max_measured = (_controller_data->max_measured < new_torque) ? new_torque : _controller_data->max_measured;
        _controller_data->max_setpoint = (_controller_data->max_setpoint < new_ff) ? new_ff : _controller_data->max_setpoint;
    }

    /* Set previous max values on rising edge */
    if (_leg_data->ground_strike) /* If a ground strike is detected */
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
    float kf_cmd = (_leg_data->toe_stance) ? (_controller_data->kf * _controller_data->filtered_setpoint) : _controller_data->filtered_setpoint;

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

    float extension_torque_magnitude_Nm = -1 * extension_torque_peak;
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
    //float extension_node3 = 100 - (mid_duration_percent_gait / 2); // extension_peak_percent_gait + extension_fall_percent_gait;

    float flexion_node1 = flexion_peak_percent_gait - flexion_rise_percent_gait - (100 - start_percent_gait);
    float flexion_node2 = flexion_peak_percent_gait - (100 - start_percent_gait);
    float flexion_node3 = flexion_peak_percent_gait + flexion_fall_percent_gait - (100 - start_percent_gait); //(100 - flexion_fall_percent_gait - flexion_peak_percent_gait);

    //float flexion_node1 = (mid_duration_percent_gait / 2); // flexion_peak_percent_gait - flexion_rise_percent_gait - (100 - start_percent_gait);
    //float flexion_node2 = flexion_peak_percent_gait; // flexion_peak_percent_gait - (100 - start_percent_gait);
    //float flexion_node3 = flexion_peak_percent_gait + flexion_fall_percent_gait; //flexion_peak_percent_gait + flexion_fall_percent_gait - (100 - start_percent_gait); //(100 - flexion_fall_percent_gait - flexion_peak_percent_gait);

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

    //torque_cmd = _spline_generation(extension_node1, extension_node2, extension_node3, extension_torque_magnitude_Nm, shifted_percent_gait) + _spline_generation(flexion_node1, flexion_node2, flexion_node3, flexion_torque_magnitude_Nm, shifted_percent_gait);

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


    //Filter Torque Reading
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);

    //Define the Setpoint
    _controller_data->ff_setpoint = torque_cmd;

    float cmd = 0;

    //Determine if it should be open or closed loop control
    if (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx] > 0)
    {
        cmd = torque_cmd + _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], 0, _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx]);
    }
    else
    {
        cmd = torque_cmd;
    }
    ////Filter Feed-Foward Setpoint
    //_controller_data->filtered_setpoint = utils::ewma(torque_cmd, _controller_data->filtered_setpoint, 0.1);
    //_controller_data->ff_setpoint = torque_cmd; // _controller_data->filtered_setpoint;

    ////Add PID Contribution to the Feed Forward Command
    //float cmd = 0;

    //if (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx] > 0)
    //{
    //    cmd = _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::i_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx]);
    //}
    //else
    //{
    //    cmd = torque_cmd;
    //}

    

    //float cmd = torque_cmd;

    //float cmd = torque_cmd + (_controller_data->parameters[controller_defs::franks_collins_hip::use_pid_idx]
    //    ? _pid(torque_cmd, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::franks_collins_hip::p_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::i_gain_idx], _controller_data->parameters[controller_defs::franks_collins_hip::d_gain_idx])
    //    : 0);

    return cmd;
}

float FranksCollinsHip::_spline_generation(float node1, float node2, float node3, float torque_magnitude, float shifted_percent_gait)
{
    float u;

    float x[3] = {node1, node2, node3};
    float y[3] = {0, torque_magnitude, 0};

    float h[2] = { (x[1] - x[0]), (x[2] - x[1]) };
    float delta[2] = { ((y[1] - y[0]) / h[0]), ((y[2] - y[1]) / h[1]) };

    //float dy[3] = { 0, 0, ((3 * delta[1]) / 2) };
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

        float cmd_ff = 0;     //Creates the cmd variable and initializes it to 0;

        if (_leg_data->do_calibration_toe_fsr)                      //If the FSRs are being calibrated or if the toe fsr is 0, send a command of zero
        {
            cmd_ff = 0;
        }
        else
        {
            cmd_ff = _controller_data->parameters[controller_defs::constant_torque::amplitude_idx];         //Send a command at the specified amplitude

            if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 0)                            //If the user wants to send a PF/Flexion torque
            {
                cmd_ff = 1 * cmd_ff;
            }
            else if (_controller_data->parameters[controller_defs::constant_torque::direction_idx] == 1)                       //If the user wants to send a DF/Extension torque
            {
                cmd_ff = -1 * cmd_ff;
            }
            else
            {
                cmd_ff = cmd_ff;                                                                                                  //If the direction flag is something other than 0 or 1, do nothing to the motor command
            }
        }

        //If the command changes
        if (cmd_ff != previous_command)
        {
            flag = 1;                                   //Set the filter flag to 1
            difference = cmd_ff - previous_command;    //Determine the sign of the change in command 
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
        else        //If the torque is changing to meet a new prescribed torque, filter the data
        {
            _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1);
        }

        //_controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, (_controller_data->parameters[controller_defs::constant_torque::alpha_idx]) / 100);

        _controller_data->ff_setpoint = cmd_ff;

        float cmd = 0;

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
}

float ElbowMinMax::calc_motor_cmd()
{

    alpha3 = _controller_data->parameters[controller_defs::elbow_min_max::FiltStrength_idx] * 0.01;
    float cmd;
    float Sig_Flex = _leg_data->toe_fsr;    //analogRead(A2);  //analogRead(A2); //_leg_data->heel_fsr; (Sensor 1)
    float Sig_Ext = _leg_data->heel_fsr;    //analogRead(A3); // //analogRead(A3); //_leg_data->toe_fsr; //not working (Sensor 2)

    // Filter the incoming FSR signals
    _leg_data->Smoothed_Sig_Flex = ((alpha0 * Sig_Flex) + ((1 - alpha0) * _leg_data->Smoothed_Sig_Flex));
    _leg_data->Smoothed_Sig_Ext = ((alpha0 * Sig_Ext) + ((1 - alpha0) * _leg_data->Smoothed_Sig_Ext));


    // ============================================ Start Manual Calibration Loop: FSR & Angle Sensing ============================================ //

    if ((_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 1) || (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 2)) 
    {

        // Initialization loop - This loop includes initialization parameters and should only run once when a manual calibration is requested (i.e. the CaliRequest_idx = 1)
        if (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 1) 
        {

            //Initialize Calibration Start Time
            _leg_data->starttime = millis();

            //Initialize Manual Calibration Parameters    
            _leg_data->Smoothed_Sig_Flex = 0;
            _leg_data->Smoothed_Sig_Ext = 0;
            _leg_data->Smoothed_Flex_Max = 0.2;
            _leg_data->Smoothed_Ext_Max = 0.2;
            _leg_data->Smoothed_Flex_Min = 0.1;
            _leg_data->Smoothed_Ext_Min = 0.1;

            // Flag that calibration has been inialized and plotting FSR signals can begin
            _leg_data->check = 1;
        }

        //Update Calibration Timer
        float timer = millis() - _leg_data->starttime;

        //Check if calibration timer is past 10 seconds, stop is so.
        if (timer > 10000) 
        {
            // Disable this calibration loop, which ensures it get's skipped in the following loop iterations
            _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 0;
        }

        // MIN/MAX Loop - If still in calibration time window (10 seconds), look for new max and min sensor readings
        else if (timer <= 10000) 
        {

            // Set new EMG/FSR Sensor Max/Min when found
            _leg_data->Smoothed_Flex_Max = max(_leg_data->Smoothed_Sig_Flex, _leg_data->Smoothed_Flex_Max);
            _leg_data->Smoothed_Flex_Min = min(_leg_data->Smoothed_Sig_Flex, _leg_data->Smoothed_Flex_Min);
            _leg_data->Smoothed_Ext_Max = max(_leg_data->Smoothed_Sig_Ext, _leg_data->Smoothed_Ext_Max);
            _leg_data->Smoothed_Ext_Min = min(_leg_data->Smoothed_Sig_Ext, _leg_data->Smoothed_Ext_Min);


            // Set new Joint Angle Sensor Max/Min when found
            _leg_data->Angle_Max = max(_leg_data->ankle.position, _leg_data->Angle_Max);
            _leg_data->Angle_Min = min(_leg_data->ankle.position, _leg_data->Angle_Min);

            // Switches the next calibration iteration loop to exclude the initialation loop, which ensures it get's skipped in the following loop iterations
            _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 2;
        }

    }

    // -------------------------------------------- End Manual Calibration Loop: FSR & Angle Sensing --------------------------------------------- //



    // ========================================= Start Automatic Calibration Loop: FSR & Angle Sensing =========================================== //

        // With the default setup (CaliRequest = 3), the program will enter this loop once during startup to determine FSR Min/Max calibration parameters

    else if (_controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] == 3) 
    {


        // Right Glove Predefined Calibration Min & Max Parameters
        if (!_joint_data->is_left) 
        {
            _leg_data->Smoothed_Flex_Max = 4.1;
            _leg_data->Smoothed_Ext_Max = 3.9;
            _leg_data->Smoothed_Flex_Min = 0.1;
            _leg_data->Smoothed_Ext_Min = 0.1;

            // old
            /*_leg_data->Smoothed_Flex_Max = 10;
            _leg_data->Smoothed_Ext_Max = 6.5;
            _leg_data->Smoothed_Flex_Min = 0.1;
            _leg_data->Smoothed_Ext_Min = 0.1;*/
        }

        // Left Glove Predefined Calibration Min & Max Parameters
        if (_joint_data->is_left) 
        {
            _leg_data->Smoothed_Flex_Max = 4;
            _leg_data->Smoothed_Ext_Max = 2.2;
            _leg_data->Smoothed_Flex_Min = 0.1;
            _leg_data->Smoothed_Ext_Min = 0.1;

            /* old
            _leg_data->Smoothed_Flex_Max = 6;
            _leg_data->Smoothed_Ext_Max = 2;
            _leg_data->Smoothed_Flex_Min = 0.1;
            _leg_data->Smoothed_Ext_Min = 0.1;*/
        }

        // Disable this calibration loop, which ensures it get's skipped in the following loop iterations
        _controller_data->parameters[controller_defs::elbow_min_max::CaliRequest_idx] = 0;

        // Flag that calibration has been completed
        _leg_data->check = 1;

    }

    // ----------------------------------------- End - Automatic Calibration Loop: FSR & Angle Sensing ------------------------------------------------ //


    // ================================================== FSR and ANGLE Normalization ============================================================== //
        //  creating signals between 0 and 100% or 0 amd 1.
        // If calibration check has been completed and flagged
    if (_leg_data->check == 1) 
    {

        // Normalize FSR/EMG data -
        _leg_data->FlexSense = (_leg_data->Smoothed_Sig_Flex - _leg_data->Smoothed_Flex_Min) / (_leg_data->Smoothed_Flex_Max - _leg_data->Smoothed_Flex_Min);
        _leg_data->ExtenseSense = (_leg_data->Smoothed_Sig_Ext - _leg_data->Smoothed_Ext_Min) / (_leg_data->Smoothed_Ext_Max - _leg_data->Smoothed_Ext_Min);

        // Smooth the signal with an EMA filter
        //_leg_data->FlexSense = ((alpha1 * _leg_data->Smoothed_Sig_Flex) + ((1 - alpha1) * _leg_data->FlexSense));
        //_leg_data->ExtenseSense = ((alpha1 * _leg_data->Smoothed_Sig_Ext) + ((1 - alpha1) * _leg_data->ExtenseSense));

        // Normalize Angle Sensor Data
        _leg_data->Angle = (_leg_data->ankle.position - _leg_data->Angle_Min) / (_leg_data->Angle_Max - _leg_data->Angle_Min);
    }

    // If calibration has not been completed - just filter the reading without normalization
    else 
    {
        _leg_data->Smoothed_Sig_Flex = ((alpha1 * Sig_Flex) + ((1 - alpha1) * _leg_data->Smoothed_Sig_Flex));
        _leg_data->Smoothed_Sig_Ext = ((alpha1 * Sig_Ext) + ((1 - alpha1) * _leg_data->Smoothed_Sig_Ext));
    }



    // =========================================================== Start: State Detection ======================================================================= //

    // Flexion Condition
    if (_leg_data->FlexSense > (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::DigitFSR_threshold_idx]) && _leg_data->FlexSense > _leg_data->ExtenseSense) 
    {

        _leg_data->setpoint = _controller_data->parameters[controller_defs::elbow_min_max::FLEXamplitude_idx];

        // Update State booleens for torque modifier loop
        _leg_data->flexState = 1;
        _leg_data->extState = 0;
        _leg_data->nullState = 0;

    }

    // Extension Condition
    else if (_leg_data->ExtenseSense > (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::PalmFSR_threshold_idx])) 
    { 

        _leg_data->setpoint = -1 * _controller_data->parameters[controller_defs::elbow_min_max::EXTamplitude_idx];

        // Update State booleens for torque modifier loop
        _leg_data->flexState = 0;
        _leg_data->extState = 1;
        _leg_data->nullState = 0;
    }


    // Zero Torque Condition
    else if (_leg_data->FlexSense < (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::DigitFSR_LOWthreshold_idx]) && _leg_data->ExtenseSense < (0.05 * _controller_data->parameters[controller_defs::elbow_min_max::PalmFSR_LOWthreshold_idx])) 
    {

        _leg_data->setpoint = 0;    //_controller_data->parameters[controller_defs::elbow_min_max::amplitude_idx];

        // Update State booleens for torque modifier loop
        _leg_data->flexState = 0;
        _leg_data->extState = 0;
        _leg_data->nullState = 1;
    }

    // Just incase condition
    else 
    {

        _leg_data->setpoint = _controller_data->previous_setpoint;
    }

    // Update previous setpoint for the "Just incase condition"
    _controller_data->previous_setpoint = _leg_data->setpoint;


    // --------------------------------------------------------------- End: State Detection ----------------------------------------------------------------------------- //



    // =========================================================== Start: Torque Profile Modifier ======================================================================= //

      // Setpoint modification - Spring Torque Profile
    if (_controller_data->parameters[controller_defs::elbow_min_max::TrqProfile_idx] == 1) 
    {

        // Flexion Modifier
        if (_leg_data->flexState) 
        {

            // This equation came from a polyfit in excel that maps the desired increase in torque with respect to the normalized angle - specifically for flexion          
            _controller_data->SpringEffect = ((3.1702 * pow(_leg_data->Angle, 3)) - (4.6572 * pow(_leg_data->Angle, 2)) + (0.49 * _leg_data->Angle) + 1.0006) * _controller_data->parameters[controller_defs::elbow_min_max::SpringPkTorque_idx];

            // This sums the selected setpoint (12 Nm) with the torque modifier, to determine the desired setpoint
            _leg_data->setpoint = _controller_data->SpringEffect + _leg_data->setpoint;

        }

        // Extension Modifier
        else if (_leg_data->extState) 
        {

            // This equation came from a polyfit in excel that maps the desired increase in torque with respect to the normalized angle - specifically for extension
            _controller_data->SpringEffect = ((-3.1702 * pow(_leg_data->Angle, 3)) + (4.8521 * pow(_leg_data->Angle, 2)) - (0.6858 * _leg_data->Angle) + (0.0034)) * _controller_data->parameters[controller_defs::elbow_min_max::SpringPkTorque_idx];

            // This sums the selected setpoint (12 Nm) with the torque modifier, to determine the desired setpoint
            _leg_data->setpoint = -1 * _controller_data->SpringEffect + _leg_data->setpoint;

        }

        else 
        {
            _leg_data->setpoint = 0;
        }
    }
    // ================================================================== End: Torque Profile Modifier ======================================================================= //


        // EXAMPLE: Serial Printing (Inside main Loop) for the left side

    if (_joint_data->is_left) 
    {
        Serial.print("Left Digits:: ");
        Serial.print(_leg_data->FlexSense);
        Serial.print(", ");
        Serial.print("Left Palm:: ");
        Serial.print(_leg_data->ExtenseSense);
    }
    if (!_joint_data->is_left) 
    {
        Serial.print("Right Digits:: ");
        Serial.print(_leg_data->FlexSense);
        Serial.print(", ");
        Serial.print("Right Palm:: ");
        Serial.println(_leg_data->ExtenseSense);
    }



    // Get Filtered torque reading, and setpoint for PID input
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, alpha2);
    _leg_data->setpoint_filtered = utils::ewma(_leg_data->setpoint, _leg_data->setpoint_filtered, alpha3);   // Was 0.01 for most trials

    // Saftey Feature - Saturate Torque Setpoint at the max if the modifier gets a wild angle reading (Max Torque Limit)
    if (_leg_data->setpoint_filtered < -1 * _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx]) 
    {
        _leg_data->setpoint_filtered = -1 * _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx];
    }
    if (_leg_data->setpoint_filtered > _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx]) 
    {
        _leg_data->setpoint_filtered = _controller_data->parameters[controller_defs::elbow_min_max::TorqueLimit_idx];
    }

    // Get motor command based on PID
    cmd = _leg_data->setpoint_filtered + _pid(_leg_data->setpoint_filtered, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::elbow_min_max::P_gain_idx], _controller_data->parameters[controller_defs::elbow_min_max::I_gain_idx], _controller_data->parameters[controller_defs::elbow_min_max::D_gain_idx]);       //originally, (10, 0, 200)

    return cmd;
}

//****************************************************


PtbGeneral::PtbGeneral(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    // Serial.println("PtbGeneral::Constructor");
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
        // Serial.print("\nIs in Mode 1 (continuous rotation). idx: 1 |  ");
        // Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_1_idx]);
        // Serial.print(" | ");
		// Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_2_idx]);
		// Serial.print(" | ");
		// Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_3_idx]);
		// Serial.print(" | ");
		// Serial.print(_controller_data->parameters[controller_defs::ptb_general::ptb_settings_4_idx]);
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
		// Serial.print("\n");
		// Serial.print("Is in default mode. Toe FSR: ");
		// Serial.print(ptb_fsrToe);
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
		// Serial.print("\n Percent gait (left side): ");
		// Serial.print(_leg_data->percent_gait);
	}
	// Serial.print("\ncmd: ");
	// Serial.print(cmd_ff);

    _controller_data->filtered_setpoint = utils::ewma(cmd_ff, _controller_data->filtered_setpoint, 0.1);
    _controller_data->ff_setpoint = _controller_data->filtered_setpoint;

    //Low Pass Filter for Torque Sensor
    _controller_data->filtered_torque_reading = utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5);

    //PID Control
    float cmd = cmd_ff + (_controller_data->parameters[controller_defs::ptb_general::use_pid_idx]
        ? _pid(cmd_ff, _controller_data->filtered_torque_reading, _controller_data->parameters[controller_defs::ptb_general::p_gain_idx], _controller_data->parameters[controller_defs::ptb_general::i_gain_idx], _controller_data->parameters[controller_defs::ptb_general::d_gain_idx])
        : 0);

	// Serial.print(" | cmd_pid: ");
	// Serial.print(cmd);

    _controller_data->filtered_cmd = utils::ewma(cmd, _controller_data->filtered_cmd, 1);
    return _controller_data->filtered_cmd;
}


//****************************************************


CalibrManager::CalibrManager(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    // Serial.println("CalibrManager::CalibrManager");
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
		cmd = _controller_data->parameters[controller_defs::calibr_manager::calibr_cmd];
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
		Serial.print("  |  doToeRefinement: ");
		Serial.print(String(_leg_data->do_calibration_refinement_toe_fsr));
		Serial.print("  |  Exo status: ");
		uint16_t exo_status = _data->get_status();
		Serial.print(String(exo_status));
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


//****************************************************


HipResist::HipResist(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
#ifdef CONTROLLER_DEBUG
    Serial.println("HipResist::HipResist");
#endif

    /* Initializes all our variables of interest to zero upon start up. */
    cmd = 0;
    angle = 0;
    previous_angle = 0;
    angular_change = 0;
    previous_angular_change = 0;
    previous_cmd = 0;
    
    /* Initializes max and min angles to extremes to allow for proper updating*/
    max_angle = -100;
    min_angle = 100;

    /* State default set to zero to prevent motor command without being in proper state. */
    state = 0;

    flag = 1;
    initial_angle = 0;
}

float HipResist::calc_motor_cmd()
{

    //cmd = _controller_data->parameters[controller_defs::hip_resist::flexion_setpoint_idx];

    //if (_controller_data->parameters[controller_defs::hip_resist::direction_idx] > 0)
    //{
    //    cmd = -1 * cmd;
    //}

    angle = (_leg_data->hip.position) * (180/(M_PI));                  /* Gets the hip postion from the motors. */

    if (flag == 1 || initial_angle == 0)
    {
        initial_angle = angle;
        flag = 0;
    }

    float current_angle = round(angle - initial_angle);

    angular_change = current_angle - previous_angle;            /* Calculates the change in angle from the previous iteration, use sign to help detect state change. */

    if (current_angle > max_angle)                              /* Updates max angle if current angle exceeds previously detected maximum angle. */
    {
        max_angle = current_angle;
    }

    if (current_angle < min_angle)                              /* Updates min angle if the current angle exceeds previously detected minimum angle. */
    {
        min_angle = current_angle;
    }

    if (current_angle >= 0.90 * max_angle && angular_change < 0 && previous_angular_change >= 0)    /* If we are within 10% of the max angle and detect a change in direction of the angular slope, update state to 1. */
    {
        state = 1;
    }
    else if (abs(current_angle) >= abs(0.9 * min_angle) && angular_change > 0 && previous_angular_change <= 0)     /* If we are within 10% of the min angle and detect a change in direction of the angular slope, update state to 2. */
    {
        state = 2;
    }
    else
    {
        state = state;
    }

    if (state == 0)                                                                        /* If the state is stuck in default, don't send a motor command. */
    {
        cmd = 0;
    }

    if (state == 1)                                                                                 /* Sends flexion command if we are extending our hip. */
    {
        cmd = _controller_data->parameters[controller_defs::hip_resist::flexion_setpoint_idx];
    }

    if (state == 2)                                                                                 /* Sends extension command if we are flexing our hip. */
    {
        cmd = _controller_data->parameters[controller_defs::hip_resist::extension_setpoint_idx];
    }
 
    previous_angle = current_angle;                                                                         /* Stores angle from this iteration into the previous angle variable. */
    previous_angular_change = angular_change;                                                       /* Stores the angular change from this iteration into the previous angular change variable. */

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

    if (active_trial)
    {

        if (!_joint_data->is_left)
        {

            Serial.print(_controller_data->ff_setpoint);
            Serial.print(',');
            Serial.print(100);
            Serial.print("\n");

            Serial.print(_controller_data->filtered_torque_reading);
            Serial.print(',');
            Serial.print(200);
            Serial.print("\n");

            Serial.print(tt*1000);
            Serial.print(',');
            Serial.print(300);
            Serial.print("\n");

        }

    }

    return cmd;

}

#endif