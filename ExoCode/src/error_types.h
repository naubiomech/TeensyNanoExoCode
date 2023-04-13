#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H

#include "ExoData.h"
#include "Exo.h"

#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41) // Nano has issues with these declarations
typedef void (*error_handler_t) (Exo*, ExoData*, int error_code);

typedef int (*error_trigger_t) (Exo*, ExoData*);
#endif

enum ErrorCodes : int
{
    NO_ERROR = 0,

    // Soft Errors
    SOFT_ERROR, // General Soft Error
    POOR_CALIBRATION,
    // Hard Errors
    HARD_ERROR, // General Hard Error
    POOR_STATE_VARIANCE,
    // Fatal Errors
    FATAL_ERROR, // General Fatal Error
    POOR_TRANSMISSION_EFFICIENCY,
    MOTOR_INERTIA_ERROR,
    MOTOR_POSTION_OUT_OF_BOUNDS,
    JOINT_POSITION_OUT_OF_BOUNDS,
    TORQUE_OUT_OF_BOUNDS,
    TORQUE_VARIANCE_ERROR,
    FORCE_VARIANCE_ERROR,
    TRACKING_ERROR,

    // System Errors
    SYSTEM_ERROR, // General System Error
    MOTOR_TIMEOUT,

    ERROR_CODE_LENGTH
};

#endif