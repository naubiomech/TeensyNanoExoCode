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

// create abstract class for error types
class ErrorType
{
public:
    ErrorType() {};
    virtual bool check(JointData* _data) = 0;
    virtual void handle(JointData* _data) = 0;
};

//TODO: implement this types
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
        return false;
    }
    void handle(JointData* _data)
    {

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
    }
};

class TorqueVarianceError : public ErrorType
{
public:
    TorqueVarianceError() : ErrorType() {};

    bool check(JointData* _data)
    {
        _data->torque_data_window.push(_data->torque_reading);
        if (_data->torque_data_window.size() > _data->torque_data_window_max_size)
        {
            _data->torque_data_window.pop();
            std::pair<float, float> pop_vals = utils::online_std_dev(_data->torque_data_window);
            std::pair<float, float> bounds = std::make_pair(
                pop_vals.first - _data->torque_std_dev_multiple*pop_vals.second,
                pop_vals.first + _data->torque_std_dev_multiple*pop_vals.second);
            _data->torque_failure_count += (int)utils::is_outside_range(_data->torque_reading, bounds.first, bounds.second);
        }

        return _data->torque_failure_count >= _data->torque_failure_count_max;
    }
    void handle(JointData* _data)
    {
        _data->motor.enabled = false;
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
    }
};

#endif
#endif // ERROR_TYPES_H