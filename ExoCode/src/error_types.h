/**
 * @file error_types.h
 * @author Chancelor Cuddeback
 * @brief Defines the abstract class for the error types, and implements the error types.
 * 
 */
#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "error_codes.h"

// create abstract class for error types
class ErrorType
{
public:
    ErrorType() {};
    virtual bool check(ExoData* _data) = 0;
    virtual void handle(ExoData* _data) = 0;
};

//TODO: implement error types

class PoorStateVarianceError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {

        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class PoorTransmissionEfficiencyError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class JointPositionOutOfBoundsError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class TorqueOutOfBoundsError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class TorqueVarianceError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class ForceVarianceError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class TrackingError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

class MotorTimeoutError : public ErrorType
{
public:
    Error() : ErrorType() {};

    
    bool check(ExoData* _data)
    {
        return false;
    }
    void handle(ExoData* _data)
    {

    }
};

#endif
#endif // ERROR_TYPES_H