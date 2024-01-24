/**
 * @file error_types.h
 * @author Chancelor Cuddeback
 * @brief Defines the abstract class for the error types, and implements the error types.
 * 
 */
#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "JointData.h"
#include "error_codes.h"
#include "Utilities.h"
#include "Logger.h"

// create abstract class for error types
class ErrorType
{
public:
    ErrorType() {};
    virtual bool check(JointData* _data) = 0;
    virtual void handle(JointData* _data) = 0;
};

class TestError : public ErrorType
{
public:
    TestError() : ErrorType() {};

    bool check(JointData* _data)
    {
        //return millis() > 45000;
        return false;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
        logger::println("Test Error", LogLevel::Error);
    }
};

class PoorStateVarianceError : public ErrorType
{
public:
    PoorStateVarianceError() : ErrorType() {};

    bool check(JointData* _data)
    {

        return false;
    }
    void handle(JointData* _data)
    {

    }
};

class PoorTransmissionEfficiencyError : public ErrorType
{
public:
    PoorTransmissionEfficiencyError() : ErrorType() {};

    bool check(JointData* _data)
    {
        // Calcualate motor torque
        const float motor_torque = _data->motor.i * _data->motor.kt;
        // Low pass motor torque
        _data->smoothed_motor_torque = utils::ewma(motor_torque, 
                        _data->smoothed_motor_torque, _data->motor_torque_smoothing);
        // If average motor torque is not close to 0, calculate the transmission efficiency
        const float torque_error = 
            utils::is_close_to(_data->smoothed_motor_torque, 0, _data->close_to_zero_tolerance) ?
                (0) : abs((float(_data->smoothed_motor_torque) - float(_data->torque_reading)) / float(_data->smoothed_motor_torque));
        const uint8_t _id = static_cast<uint8_t>(_data->id);
        _data->torque_error = utils::ewma(torque_error,
                        _data->torque_error, _data->torque_error_smoothing);

        //Serial.print(_id);
        //Serial.print(":");
        //Serial.print(torque_error);
        //Serial.print(_id < 60 ? "\t" : "\n");

        //return abs(torque_error) > 100*(1 - _data->transmission_efficiency_threshold);
		_data->torque_error_max = max(abs(_data->torque_error), abs(_data->torque_error_max));
		
		
		
		
		
		
		
		#if USE_ANGLE_SENSORS
		//Serial.println("USE_ANGLE_SENSORS");
		if (_data->do_calc_motor_pos_offset) {
			//Calculate "true" motor position
			_data->motor_pos_0 = _data->motor_RoM * (1 - (_data->joint_position / _data->joint_RoM));
			_data->motor_pos_offset = _data->motor.p - _data->motor_pos_0;
			_data->do_calc_motor_pos_offset = false;
		}
		//_data->motor_pos_0 = (_data->joint_position * 7.25) / 100;
		/* if (!_data->is_left) {
			Serial.print("\n_data->motor_pos_0 (Relative): " + String(_data->motor_pos_0));
			Serial.print(" | _data->motor.p: " + String(_data->motor.p) + " | _data->motor_pos_offset: " + String(_data->motor_pos_offset));

		} */
		//return ((_data->motor.p > (7.25 + _data->motor_pos_offset - (-0.25))) || (_data->motor.p < _data->motor_pos_offset + (-0.25)));
		#endif
		
		
		
		
		//Transmission efficiency alternative
		_data->motor_diff_1 = _data->motor_diff_2;
		_data->motor_pos_1 = _data->motor_pos_2;
		_data->motor_pos_2 = utils::ewma(_data->motor.p, _data->motor_pos_2, 0.01);
		_data->motor_diff_2 = _data->motor_pos_2 - _data->motor_pos_1;
		if (_data->motor_diff_1 * _data->motor_diff_2 < 0) {
			_data->motor_ref_pos = _data->motor_pos_2;
		}
		
		
		
		
		/* if (!_data->is_left) {
			Serial.print("\n_data->motor_ref_pos: " + String(_data->motor_ref_pos));
			Serial.print(" | _data->motor_pos_1: " + String(_data->motor_pos_1) + " | _data->motor_pos_2: " + String(_data->motor_pos_2));
			Serial.print(" | Right motor position change: ");
			Serial.print((_data->motor_pos_2 - _data->motor_ref_pos));
		} */
		
		/* if (!_data->is_left) {
			Serial.print("\nRight side _data->motor_pos_safety_factor: " + String(_data->motor_pos_safety_factor));
			Serial.print(" | Right side _data->motor_RoM: " + String(_data->motor_RoM) );
		}
		else {
			Serial.print("Left side _data->motor_pos_safety_factor: " + String(_data->motor_pos_safety_factor));
			Serial.print(" | Left side _data->motor_RoM: " + String(_data->motor_RoM) );
		} */
		
		//return false;
		if (_data->motor_RoM == 0) {
			//Serial.print("\nMotor pos error handler DISABLED.");
			return false;
		}
		#if USE_ANGLE_SENSORS
		//return ((_data->motor.p > (7.5 + _data->motor_pos_offset - (-0.2))) || (_data->motor.p < _data->motor_pos_offset + (-0.2))) || (abs(_data->motor_pos_2 - _data->motor_ref_pos) > 9);
		return ((_data->motor.p > (_data->motor_RoM + _data->motor_pos_offset + _data->motor_RoM * (_data->motor_pos_safety_factor - 1))) || (_data->motor.p < _data->motor_pos_offset - _data->motor_RoM * (_data->motor_pos_safety_factor - 1)));
		#else
		return abs(_data->motor_pos_2 - _data->motor_ref_pos) > _data->motor_RoM * _data->motor_pos_safety_factor;
		#endif
		
		
		//return abs(torque_error) > 150;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
        logger::println("Transmission Efficiency Error", LogLevel::Error);
    }
};

class TorqueOutOfBoundsError : public ErrorType
{
public:
    TorqueOutOfBoundsError() : ErrorType() {};

    bool check(JointData* _data)
    {
        return abs(_data->torque_reading) > _data->torque_output_threshold;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
        logger::println("Torque Out of Bounds Error", LogLevel::Error);
    }
};

class TorqueVarianceError : public ErrorType
{
public:
    TorqueVarianceError() : ErrorType() {};

    bool check(JointData* _data)
    {
        // Append new torque reading to the window
        _data->torque_data_window.push(_data->torque_reading);
        // If the window is larger than the max size, pop the oldest value
        if (_data->torque_data_window.size() > _data->torque_data_window_max_size)
        {
            _data->torque_data_window.pop();
            // Calculate the standard deviation of the window
            std::pair<float, float> pop_vals = utils::online_std_dev(_data->torque_data_window);
            // Generate symmetric bounds around the mean
            std::pair<float, float> bounds = std::make_pair(
                pop_vals.first - _data->torque_std_dev_multiple*pop_vals.second,
                pop_vals.first + _data->torque_std_dev_multiple*pop_vals.second);
            // Increment the failure count if the current torque reading is outside the bounds
            _data->torque_failure_count += (int)utils::is_outside_range(_data->torque_reading, bounds.first, bounds.second);
        }
        // If the failure count is greater than the max, return true
        return _data->torque_failure_count >= _data->torque_failure_count_max;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
        logger::println("Torque Variance Error", LogLevel::Error);
    }
};

class ForceVarianceError : public ErrorType
{
public:
    ForceVarianceError() : ErrorType() {};

    bool check(JointData* _data)
    {
        return false;
    }
    void handle(JointData* _data)
    {

    }
};

class TrackingError : public ErrorType
{
public:
    TrackingError() : ErrorType() {};

    bool check(JointData* _data)
    {
        return false;
    }
    void handle(JointData* _data)
    {

    }
};

class MotorTimeoutError : public ErrorType
{
public:
    MotorTimeoutError() : ErrorType() {};

    bool check(JointData* _data)
    {
        const bool timeout_error = _data->motor.timeout_count >= _data->motor.timeout_count_max;
        if (timeout_error) 
        {
            _data->motor.timeout_count = 0;
        }
        return timeout_error;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
        logger::println("Motor Timeout Error", LogLevel::Error);
    }
};


#endif
#endif // ERROR_TYPES_H