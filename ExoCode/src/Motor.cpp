/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Arduino.h" //TODO: Remove

#include "Motor.h"
#include "CAN.h"
#include "ErrorManager.h"
#include "error_codes.h"
#include "Logger.h"
#include "ErrorReporter.h"
#include "error_codes.h"
//#define MOTOR_DEBUG

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
{
    _id = id;
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    _enable_pin = enable_pin;
    _prev_motor_enabled = false; 
    _prev_on_state = false;
    
    #ifdef MOTOR_DEBUG
        logger::print("_Motor::_Motor : _enable_pin = ");
        logger::print(_enable_pin);
        logger::print("\n");
    #endif

    
    pinMode(_enable_pin, OUTPUT);
    
    // set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_leg.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_leg.ankle.motor);
            }
            break;
    }
#ifdef MOTOR_DEBUG
    logger::println("_Motor::_Motor : Leaving Constructor");
#endif
};

bool _Motor::get_is_left() 
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};

/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 * 
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) // constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    _KP_MIN = 0.0f;
    _KP_MAX = 500.0f;
    _KD_MIN = 0.0f;
    _KD_MAX = 5.0f;
    _P_MAX = 12.5f;

    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
    j_data->motor.kt = this->get_Kt();

    _enable_response = false;

#ifdef MOTOR_DEBUG
    logger::println("_CANMotor::_CANMotor : Leaving Constructor");
#endif
};

void _CANMotor::transaction(float torque)
{
    // send data and read response 
    send_data(torque);
    read_data();
    check_response();
};



void _CANMotor::read_data()
{
    // read data from motor
    bool searching = true;
    uint32_t start = micros();
    // only send and receive data if enabled
    if (_motor_data->enabled)
    {
        CAN* can = can->getInstance();
        do
        {
            int direction_modifier = _motor_data->flip_direction ? -1 : 1;

            //if (!_motor_data->is_left)
            //{
            //    direction_modifier *= -1;
            //}

            CAN_message_t msg = can->read();
            if (msg.buf[0] == uint32_t(_motor_data->id))
            {
                // unpack data
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3]) << 4 | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
                // set data in ExoData object
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);

                //if (_motor_data->is_left)
                //{ 
                //    Serial.print(_motor_data->i);
                //    Serial.print(',');
                //    Serial.print(300);
                //    Serial.print(',');
                //    Serial.print("\n");
                //}

                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                // reset timout_count because we got a valid message
                _motor_data->timeout_count = 0;
                return;
            }
            searching = ((micros() - start) < _timeout);
        }
        while(searching);
        // if we get here, we timed out
        _handle_read_failure();
    }
    return;
};



void _CANMotor::send_data(float torque)
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif

    int direction_modifier = _motor_data->flip_direction ? -1 : 1;

    //if (!_motor_data->is_left)
    //{
    //    direction_modifier *= -1;
    //}

    _motor_data->t_ff = torque;
    const float current = torque / get_Kt();

    // read data from ExoData object, constraint it, and package it
    float p_sat = constrain(direction_modifier * _motor_data->p_des, -_P_MAX, _P_MAX);
    float v_sat = constrain(direction_modifier * _motor_data->v_des, -_V_MAX, _V_MAX);
    float kp_sat = constrain(_motor_data->kp, _KP_MIN, _KP_MAX);
    float kd_sat = constrain(_motor_data->kd, _KD_MIN, _KD_MAX);
    float i_sat = constrain(direction_modifier * current, -_I_MAX, _I_MAX);
    _motor_data->last_command = i_sat;
    uint32_t p_int = _float_to_uint(p_sat, -_P_MAX, _P_MAX, 16);
    uint32_t v_int = _float_to_uint(v_sat, -_V_MAX, _V_MAX, 12);
    uint32_t kp_int = _float_to_uint(kp_sat, _KP_MIN, _KP_MAX, 12);
    uint32_t kd_int = _float_to_uint(kd_sat, _KD_MIN, _KD_MAX, 12);
    uint32_t i_int = _float_to_uint(i_sat, -_I_MAX, _I_MAX, 12);
    CAN_message_t msg;
    msg.id = uint32_t(_motor_data->id);
    msg.buf[0] = p_int >> 8;
    msg.buf[1] = p_int & 0xFF;
    msg.buf[2] = v_int >> 4;
    msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.buf[4] = kp_int & 0xFF;
    msg.buf[5] = kd_int >> 4;
    msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
    msg.buf[7] = i_int & 0xFF;

    //logger::print("_CANMotor::send_data::t_sat:: ");
    //logger::print(t_sat);
    //logger::print("\n");
    //logger::print("T:"+String(t_sat)+",");
    
    // only send messages if enabled
    if (_motor_data->enabled)
    {
        // set data in motor
        CAN* can = can->getInstance();
        can->send(msg);
    }
    return;
};

void _CANMotor::check_response()
{
    // only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
    if (_data->user_paused || !active_trial || _data->estop || _error)
    {
        return;
    }

    // Measured current variance should be non-zero
    _measured_current.push(_motor_data->i);
    if (_measured_current.size() > _current_queue_size)
    {
        _measured_current.pop();
        auto pop_vals = utils::online_std_dev(_measured_current);
        if (pop_vals.second < _variance_threshold)//pop_vals.second represents the standard deviation
        {
            _motor_data->enabled = true;
            enable(true);
        }
    }
}

void _CANMotor::on_off()
{
    if (_data->estop || _error)
    {
        _motor_data->is_on = false;
        // logger::print("_CANMotor::on_off(bool is_on) : E-stop pulled - ");
        // logger::print(uint32_t(_motor_data->id));
        // logger::print("\n");
    }
    if (_prev_on_state != _motor_data->is_on) // if was here to save time, can be removed if making problems, or add overide
    {
        if (_motor_data->is_on)
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_on_state);
            // logger::print("_CANMotor::on_off(bool is_on) : Power on- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
        else 
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_off_state);
            // logger::print("_CANMotor::on_off(bool is_on) : Power off- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
    }
    _prev_on_state = _motor_data->is_on;

    #ifdef HEADLESS
    delay(2000);    //Two second delay between motor's turning on and enabeling, we've run into some issues with enabling while in headless mode if this delay is not present. 
    #endif

};

bool _CANMotor::enable()
{
    return enable(false);
};

bool _CANMotor::enable(bool overide)
{
    #ifdef MOTOR_DEBUG
    //  logger::print(_prev_motor_enabled);
    //  logger::print("\t");
    //  logger::print(_motor_data->enabled);
    //  logger::print("\t");
    //  logger::print(_motor_data->is_on);
    //  logger::print("\n");
    #endif
    
    // only change the state and send messages if the enabled state has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide || !_enable_response)
    {
        CAN_message_t msg;
        msg.id = uint32_t(_motor_data->id);
        msg.buf[0] = 0xFF;
        msg.buf[1] = 0xFF;
        msg.buf[2] = 0xFF;
        msg.buf[3] = 0xFF;
        msg.buf[4] = 0xFF;
        msg.buf[5] = 0xFF;
        msg.buf[6] = 0xFF;

        // TODO: Dont reenable after error, or if estop is pulled
        if (_motor_data->enabled && !_error && !_data->estop)
        {
            // !!! A delay check between when turning on power and when timeouts stopped happening gave a delay of 1930 ms rounding to 2000.
            // enable motor
            msg.buf[7] = 0xFC;
        }
        else 
        {
            _enable_response = false;
            // disable motor, the message after this shouldn't matter as the power is cut, and the send() doesn't send a message if not enabled.
            msg.buf[7] = 0xFD;
        }
        CAN* can = can->getInstance();
        can->send(msg);
        delayMicroseconds(500);
        read_data();
        if (_motor_data->timeout_count == 0)
        {
            _enable_response = true;
        }
    }
    _prev_motor_enabled = _motor_data->enabled;
    return _enable_response;
};

void _CANMotor::zero()
{
    CAN_message_t msg;
    msg.id = uint32_t(_motor_data->id);
    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;//From t-motor documentation://Set the current position of the motor to 0 by sending "0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE"
    CAN* can = can->getInstance();
    can->send(msg);
    
    read_data();
};

float _CANMotor::get_Kt()
{
    return _Kt;
};

void _CANMotor::set_error()
{
    _error = true;
};

void _CANMotor::set_Kt(float Kt)
{
    _Kt = Kt;
};


void _CANMotor::_handle_read_failure()
{
    logger::println("CAN Motor - Handle Read Failure", LogLevel::Error);
    _motor_data->timeout_count++;
};

float _CANMotor::_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
};
float _CANMotor::_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
};

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin): // constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 22.0f;
    _V_MAX = 41.87f;
    float kt = 0.068 * 6;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60::AK60 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60_v1_1::AK60_v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin): // constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 13.5f;
    _V_MAX = 23.04f;

    float kt = 1/0.37775;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60_v1_1::AK60_v1_1 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60_v1_1_T::AK60_v1_1_T(config_defs::joint_id id, ExoData* exo_data, int enable_pin) : // constructor: type is the motor type
    _CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 9.0f;
    _V_MAX = 23.04f; 

    float kt = 1;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60_v1_1::AK60_v1_1 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin): // constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 24.0f;
    _V_MAX = 25.65f;

    float kt = 0.091 * 9;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;


#ifdef MOTOR_DEBUG
    logger::println("AK80::AK80 : Leaving Constructor");
#endif
};

AK70::AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin): // constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 23.2f;
    _V_MAX = 15.5f;
    float kt = 0.13 * 10;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;
};


/*
 * Constructor for the PWM Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the PWM (e.g. Maxon) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 * 
 */
MaxonMotor::MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) // constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{

    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));

	// _enable_response = false;
	
#ifdef MOTOR_DEBUG
    logger::println("_PWMMotor::_PWMMotor : Leaving Constructor");
#endif
};

void MaxonMotor::transaction(float torque)
{
    // send data
    send_data(torque);
    master_switch();//only enable the motor when it's an active trial 
	if (!_motor_data->is_left) {
		if (_motor_data->enabled) {
			maxon_manager(true);//"maxon motor reset if error" up and running
		}
		else {//reset the motor error detect function, in case of a user pause during a motor error event
			maxon_manager(false);
		}
	}
};

bool MaxonMotor::enable()
{
	//Serial.print("\nMaxonMotor::enable()");
    return true;//not doing meaningful things for this motor type
};

bool MaxonMotor::enable(bool overide)
{
	// Serial.print("\nMaxonMotor::enable(bool ");
	// Serial.print(overide);
	// Serial.print(")");
	// Serial.print("  _motor_data->id: ");
	// Serial.print(uint32_t(_motor_data->id));
	// Serial.print("  _motor_data->enabled: ");
	// Serial.print(_motor_data->enabled);
	// Serial.print("  _motor_data->motor_type: ");
	// Serial.print(_motor_data->motor_type);
	
	// only change the state and send messages if the enabled state (used as a master switch for this motor) has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide)
    {

        // TODO: Dont reenable after error, or if estop is pulled
        //if (_motor_data->enabled && !_error && !_data->estop)
		if (_motor_data->enabled)//_motor_data->enabled is controlled by active_trial and user_paused, refer to master_switch().
        {
			//Serial.print("  _motor_data->enabled CHANGED! Now enabled.");
            // !!! A delay check between when turning on power and when timeouts stopped happening gave a delay of 1930 ms rounding to 2000.
            // enable motor
			digitalWrite(_enable_pin,HIGH);//relocate in the future
        }
		_enable_response = true;
	}
	if (!overide)//when enable(false), send the disable motor command, set the analogWrite resolution, and send 50% PWM command
        {
			//Serial.print("  _motor_data->enabled CHANGED! Now disabled.");
			_enable_response = false;
			// disable motor, the message after this shouldn't matter as the power is cut, and the send() doesn't send a message if not enabled.
			digitalWrite(_enable_pin,LOW);
			analogWriteResolution(12);
			analogWriteFrequency(A9, 5000);
			analogWrite(A9,2048);
			pinMode(A1,INPUT);
        }
	_prev_motor_enabled = _motor_data->enabled;
    return _enable_response;
	
    #ifdef MOTOR_DEBUG
     logger::print(_prev_motor_enabled);
     logger::print("\t");
     logger::print(_motor_data->enabled);
     logger::print("\t");
     logger::print(_motor_data->is_on);
     logger::print("\n");
    #endif
};

void MaxonMotor::send_data(float torque)//always send motor command regardless of the motor "enable" status
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif
	
	int direction_modifier = _motor_data->flip_direction ? -1 : 1;//corresponding to the flipMotorDirection key value on the SD card

	_motor_data->t_ff = torque;
    _motor_data->last_command = torque;
	
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || //
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
   // if (_data->user_paused || !active_trial || _data->estop || _error)
	if (_data->user_paused || !active_trial)
    {
        analogWrite(A9,2048);//send 50% PWM (0 current)
    }
	else
   {
	if (!_motor_data->is_left) {
		uint16_t post_fuse_torque = max(455,2048+(direction_modifier*1*torque));//set the lowerest allowed PWM command
		post_fuse_torque = min(3890,post_fuse_torque);//set the highest allowed PWM command
		analogWrite(A9,post_fuse_torque);
		// Serial.print("\npost_fuse_torque: ");
		// Serial.print(post_fuse_torque);
	}
   }
	
};

void MaxonMotor::master_switch()
{
   //only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
    //if (_data->user_paused || !active_trial || _data->estop || _error)
	Serial.print("\nexo_status: ");
	Serial.print(exo_status);
	if (_data->user_paused || !active_trial)
    {
		_motor_data->enabled = false;
        enable(false);
    }
	else {
		_motor_data->enabled = true;
        enable(true);
	}
};

//Our implementation of the Maxon motor including the ec motor and the Escon 50_8 Motor Controller would occasionally cause 50_8 to enter error mode, with "Over current" being one of the errors.
//To tackle this issue, we have sucessfully implemented a solution, now encapsulated in maxon_manager().
void MaxonMotor::maxon_manager(bool manager_active) {
	pinMode(37,INPUT_PULLUP);
	//Serial.print("\n!digitalRead(37): ");
	//Serial.print(!digitalRead(37));
	//Serial.print("\nmaxon_manager(bool manager_active): ");
	//Serial.print(manager_active);
	if (!manager_active) {//initialization when the switch is set to FALSE
		do_scan4maxon_err = true;//initialization
		maxon_counter_active = false;//initialization
		//zen_period = 0;//initialization
		Serial.print("\n-----------------maxon_manager(false)");
	}
	else {//only run the error detection and reset code when the switch is set to TRUE
		if ((do_scan4maxon_err) && (!digitalRead(37))) {//scan for motor error conditionally
			do_scan4maxon_err = false;
			maxon_counter_active = true;
			zen_millis = millis();
			Serial.print("\n-----------------(do_scan4maxon_err) && (!digitalRead(37)");
		}
		if (maxon_counter_active) {
			//zen_period++;//use millis();
			if (millis() - zen_millis >= 30) {//this will run 20 iterations after the following one
				do_scan4maxon_err = true;//do continue to scan for motor error
				maxon_counter_active = false;
				//zen_period = 0;
				Serial.print("\n---------maxon_counter_active = false;  millis()= ");
				Serial.print(millis());
				Serial.print("  |  zen_millis= ");
				Serial.print(zen_millis);
			}
			else if (millis() - zen_millis >= 10) {//this will run 8 iterations after maxon_counter_active is set to TRUE
				enable(true);//send enable motor command
				//digitalWrite(33,HIGH);
				Serial.print("\n---enable(true)");
			}
			else if (millis() - zen_millis >= 2) {//this will run 2 iterations after the following one
				enable(false);//send disable motor command
				//digitalWrite(33,LOW);
				Serial.print("\nenable(false)");
			}
		}
	}

}
#endif