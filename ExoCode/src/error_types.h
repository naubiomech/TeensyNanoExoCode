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

// create abstract class for error types
class ErrorType
{
public:
    ErrorType() {};
    virtual bool check(JointData* _data) = 0;
    virtual void handle(JointData* _data) = 0;
};

//TODO: implement error types
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

class JointPositionOutOfBoundsError : public ErrorType
{
public:
    JointPositionOutOfBoundsError() : ErrorType() {};

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
        return false;
    }
    void handle(JointData* _data)
    {

    }
};

class TorqueVarianceError : public ErrorType
{
public:
    TorqueVarianceError() : ErrorType() {};

    bool check(JointData* _data)
    {
        return false;
    }
    void handle(JointData* _data)
    {

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
        return false;
    }
    void handle(JointData* _data)
    {

    }
};

#endif
#endif // ERROR_TYPES_H