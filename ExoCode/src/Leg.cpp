#include "Leg.h"
#include "Logger.h"
//#define LEG_DEBUG 1

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
/*
 * Constructor for the leg
 * Takes in if the leg is the left one and a pointer to the exo_data
 * Uses initializer list for hip, knee, and ankle joint; and the FSRs.
 * Only stores these objects, exo_data pointer, and if it is left (for easy access)
 */
Leg::Leg(bool is_left, ExoData* exo_data)
: _hip((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::hip), exo_data)  // we need to cast to uint8_t to do bitwise or, then we have to cast it back to joint_id
, _knee((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::knee), exo_data)
, _ankle((config_defs::joint_id)((uint8_t)(is_left ? config_defs::joint_id::left : config_defs::joint_id::right) | (uint8_t)config_defs::joint_id::ankle), exo_data)
, _heel_fsr(is_left ? logic_micro_pins::fsr_sense_left_heel_pin : logic_micro_pins::fsr_sense_right_heel_pin) // Check if it is the left and use the appropriate pin for the side.
, _toe_fsr(is_left ? logic_micro_pins::fsr_sense_left_toe_pin : logic_micro_pins::fsr_sense_right_toe_pin)
{
    _data = exo_data;
    
    _is_left = is_left;
    
    // This data object is set for the specific leg so we don't have to keep checking the side.
    _leg_data = _is_left ? &(_data->left_leg) : &(_data->right_leg);
    #ifdef LEG_DEBUG
        logger::print(_is_left ? "Left " : "Right ");
        logger::println("Leg :: Constructor : _data set");
    #endif
    _prev_heel_contact_state = true; // initialized to true so we don't get a strike the first time we read
    _prev_toe_contact_state = true;
    _prev_toe_contact_state_toe_off = true;
    
    for (int i = 0; i<_num_steps_avg; i++)
    {
        _step_times[i] = 0;
    }
    _ground_strike_timestamp = 0;
    _prev_ground_strike_timestamp = 0;
    //_expected_step_duration = 0;
    #ifdef LEG_DEBUG
        logger::print(_is_left ? "Left " : "Right ");
        logger::println("Leg :: Constructor : Exit");
    #endif

    _heel_fsr.get_contact_thresholds(_leg_data->heel_fsr_lower_threshold, _leg_data->heel_fsr_upper_threshold);
    _toe_fsr.get_contact_thresholds(_leg_data->toe_fsr_lower_threshold, _leg_data->toe_fsr_upper_threshold);

    inclination_detector = new InclinationDetector();

};

void Leg::disable_motors()
{
    _hip._motor->enable(true);
    _knee._motor->enable(true);
    _ankle._motor->enable(true);
};


void Leg::run_leg()
{
    #ifdef LEG_DEBUG
        logger::print("\nmicros : ");
        logger::println(micros());
        logger::print(_is_left ? "Left " : "Right ");
        logger::println("Leg :: run_leg : checking calibration");
    #endif
    check_calibration();
    #ifdef LEG_DEBUG
        logger::print(_is_left ? "Left " : "Right ");
        logger::println("Leg :: run_leg : reading data");
    #endif
    _check_thresholds();
    // read all the data before we calculate and send the new motor commands
    read_data();
    #ifdef LEG_DEBUG
        logger::print(_is_left ? "Left " : "Right ");
        logger::println("Leg :: run_leg : updating motor commands");
    #endif
    // calculates the new motor commands and sends them.
    update_motor_cmds();

}; 

void Leg::read_data()
{
    // Check the FSRs
    _leg_data->heel_fsr = _heel_fsr.read();
    _leg_data->toe_fsr = _toe_fsr.read();

    _leg_data->ground_strike = _check_ground_strike();
    if (_leg_data->ground_strike)
    {
        _leg_data->expected_step_duration = _update_expected_duration();
    }

    _leg_data->toe_off = _check_toe_off();
    _leg_data->percent_gait = _calc_percent_gait();

    _heel_fsr.get_contact_thresholds(_leg_data->heel_fsr_lower_threshold, _leg_data->heel_fsr_upper_threshold);
    _toe_fsr.get_contact_thresholds(_leg_data->toe_fsr_lower_threshold, _leg_data->toe_fsr_upper_threshold);
    //if (!_is_left) {Serial.println("Checking with angle: " + String(_leg_data->ankle.joint_position));}
    _leg_data->inclination = inclination_detector->check(_leg_data->toe_stance, 
        _leg_data->do_calibration_refinement_toe_fsr, _leg_data->ankle.joint_position);
    //if (!_is_left) {logger::print("Got incline: ", LogLevel::Debug); logger::println((uint8_t)_leg_data->inclination, LogLevel::Debug);}
    
    // Check the joint sensors if the joint is used.
    if (_leg_data->hip.is_used)
    {
        _hip.read_data();
    }
    if (_leg_data->knee.is_used)
    {
        _knee.read_data();
    }
    if (_leg_data->ankle.is_used)
    {
        _ankle.read_data();
    }
};

void Leg::check_calibration()
{
    if (_leg_data->is_used)
    {
        // make sure fsr calibration is done before refinement.
        if (_leg_data->do_calibration_toe_fsr)
        {
            _leg_data->do_calibration_toe_fsr = _toe_fsr.calibrate(_leg_data->do_calibration_toe_fsr);
            _data->set_status(status_defs::messages::fsr_calibration);
        }
        else if (_leg_data->do_calibration_refinement_toe_fsr) 
        {
            _leg_data->do_calibration_refinement_toe_fsr = _toe_fsr.refine_calibration(_leg_data->do_calibration_refinement_toe_fsr);
            _data->set_status(status_defs::messages::fsr_refinement);
        }
        
        if (_leg_data->do_calibration_heel_fsr)
        {
            _leg_data->do_calibration_heel_fsr = _heel_fsr.calibrate(_leg_data->do_calibration_heel_fsr);
            _data->set_status(status_defs::messages::fsr_calibration);
        }
        else if (_leg_data->do_calibration_refinement_heel_fsr) 
        {
            _leg_data->do_calibration_refinement_heel_fsr = _heel_fsr.refine_calibration(_leg_data->do_calibration_refinement_heel_fsr);
            _data->set_status(status_defs::messages::fsr_refinement);
        }
        
        // check torque sensor calibrations if joint is use
        // Check the joint sensors if the joint is used.
        if (_leg_data->hip.is_used)
        {
            _hip.check_calibration();
        }
        if (_leg_data->knee.is_used)
        {
            _knee.check_calibration();
        }
        if (_leg_data->ankle.is_used)
        {
            _ankle.check_calibration();
        }
        
    }        
};

void Leg::_check_thresholds()
{
    _toe_fsr.set_contact_thresholds(_leg_data->toe_fsr_lower_threshold, _leg_data->toe_fsr_upper_threshold);
    _heel_fsr.set_contact_thresholds(_leg_data->heel_fsr_lower_threshold, _leg_data->heel_fsr_upper_threshold);
}

bool Leg::_check_ground_strike()
{
    _leg_data->prev_heel_stance = _prev_heel_contact_state;  //This might not work, needs to be tested
    _leg_data->prev_toe_stance = _prev_toe_contact_state;

    // TODO: Only use the heel fsr if it is connected. Maybe only if the hip is used?
    bool heel_contact_state = _heel_fsr.get_ground_contact();
    bool toe_contact_state = _toe_fsr.get_ground_contact();
    _leg_data->heel_stance = heel_contact_state;
    _leg_data->toe_stance = toe_contact_state;
    bool ground_strike = false;
    
    
    // logger::print("Leg::_check_ground_strike : _prev_heel_contact_state - ");
    // logger::print(_prev_heel_contact_state);
    // logger::print("\n");
    // logger::print("\t_prev_toe_contact_state - ");
    // logger::print(_prev_toe_contact_state);
    // logger::print("\n");
    // only check if in swing
    _leg_data->toe_strike = toe_contact_state > _prev_toe_contact_state;
    if(!_prev_heel_contact_state & !_prev_toe_contact_state) //If we were previously in swing
    {
        //check for rising edge on heel and toe, toe is to account for flat foot landings
        if ((heel_contact_state > _prev_heel_contact_state) | (toe_contact_state > _prev_toe_contact_state))    //If either the heel or toe FSR is on the ground and it previously wasn't on the ground
        {
            ground_strike = true;
            _prev_ground_strike_timestamp = _ground_strike_timestamp;
            _ground_strike_timestamp = millis();
        }
    }

    _prev_heel_contact_state = heel_contact_state;
    _prev_toe_contact_state = toe_contact_state;
    
    return ground_strike;
};

bool Leg::_check_toe_off()
{
    bool toe_off = false;
    if(_prev_toe_contact_state_toe_off) //If we were previously in stance
    {
        //check for falling edge on toe
        toe_off = (_leg_data->toe_stance < _prev_toe_contact_state_toe_off);
    }

    _prev_toe_contact_state_toe_off = _leg_data->toe_stance;

    return toe_off;
}

float Leg::_calc_percent_gait()
{
    int timestamp = millis();
    int percent_gait = -1;
    // only calulate if the expected step duration has been established.
    if (_leg_data->expected_step_duration>0)
    {
        percent_gait = 100 * ((float)timestamp - _ground_strike_timestamp) / _leg_data->expected_step_duration;
        percent_gait = min(percent_gait, 100); // set saturation.
        // logger::print("Leg::_calc_percent_gait : percent_gait_x10 = ");
        // logger::print(percent_gait_x10);
        // logger::print("\n");
    }
    return percent_gait;
};

float Leg::_update_expected_duration()
{
    unsigned int step_time = _ground_strike_timestamp - _prev_ground_strike_timestamp;
    float expected_step_duration = _leg_data->expected_step_duration;
		
    if (0 == _prev_ground_strike_timestamp) // if the prev time isn't set just return.
    {
        return expected_step_duration;
    }
    uint8_t num_uninitialized = 0;
    // check that everything is set.
    for (int i = 0; i<_num_steps_avg; i++)
    {
        num_uninitialized += (_step_times[i] == 0);
    }
    
    // get the max and min values of the array for determining the window for expected values.
    unsigned int* max_val = std::max_element(_step_times, _step_times + _num_steps_avg);
    unsigned int* min_val = std::min_element(_step_times, _step_times + _num_steps_avg);
    
    if  (num_uninitialized > 0)  // if all the values haven't been replaced
    {
        // shift all the values and insert the new one
        for (int i = 1; i<_num_steps_avg; i++)
        {
            _step_times[i] = _step_times[i-1];
        }
        _step_times[0] = step_time;
        
        // logger::print("Leg::_update_expected_duration : _step_times not fully initialized- [\t");
        // for (int i = 0; i<_num_steps_avg; i++)
        // {
            // logger::print(_step_times[i]);
            // logger::print("\t");
        // }
        // logger::print("\t]\n");
        
        
    }
    // consider it a good step if the ground strike falls within a window around the expected duration.
    // Then shift the step times and put in the new value.
    else if ((step_time <= (_leg_data->expected_duration_window_upper_coeff * *max_val)) & (step_time >= (_leg_data->expected_duration_window_lower_coeff * *min_val))) // and (armed_time > ARMED_DURATION_PERCENT * self.expected_duration)): # a better check can be used.  If the person hasn't stopped or the step is good update the vector.  
    {
        int sum_step_times = step_time;
        for (int i = 1; i<_num_steps_avg; i++)
        {
            sum_step_times += _step_times[i-1];
            _step_times[i] = _step_times[i-1];
        }
        _step_times[0] = step_time;
        
        // TODO: Add rate limiter for change in expected duration so it can't make big jumps
        expected_step_duration = sum_step_times/_num_steps_avg;  // Average to the nearest ms
        // logger::print("Leg::_update_expected_duration : _expected_step_duration - ");
        // logger::print(_expected_step_duration);
        // logger::print("\n");
    }
    return expected_step_duration;
};

void Leg::clear_step_time_estimate()
{
    for (int i = 0; i<_num_steps_avg; i++)
    {
        _step_times[i] = 0;
    }
};

void Leg::update_motor_cmds()
{
    // Check the joint sensors if the joint is used.
    if (_leg_data->hip.is_used)
    {
        _hip.run_joint();
    }
    if (_leg_data->knee.is_used)
    {
        _knee.run_joint();
    }
    if (_leg_data->ankle.is_used)
    {
        _ankle.run_joint();
    }
};


float Leg::get_Kt_for_joint(uint8_t id)
{
    float Kt = 0;
    switch (id)
    {
    case (uint8_t)config_defs::joint_id::left_hip:
    case (uint8_t)config_defs::joint_id::right_hip:
        Kt = _hip._motor->get_Kt();
        break;
    case (uint8_t)config_defs::joint_id::left_knee:
    case (uint8_t)config_defs::joint_id::right_knee:
        Kt = _knee._motor->get_Kt();
        break;
    case (uint8_t)config_defs::joint_id::left_ankle:
    case (uint8_t)config_defs::joint_id::right_ankle:
        Kt = _ankle._motor->get_Kt();
        break;
    default:
        // logger::print("ExoData::get_joint_with->No joint with ");
        // logger::print(id);
        // logger::println(" was found.");
        break;
    }
    return Kt;
}



#endif