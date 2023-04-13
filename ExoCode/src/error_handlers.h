#ifndef ERROR_HANDLERS_H
#define ERROR_HANDLERS_H

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "ExoData.h"
#include "Exo.h"
#include "error_types.h"
#include "uart_commands.h"

// Handles errors
namespace error_handlers
{

    void soft(Exo* exo, ExoData* exo_data, int error_code)
    {
        // TODO: Recalibrate/Warn user

        return;
    }
    void hard(Exo* exo, ExoData* exo_data, int error_code)
    {
        exo_data->for_each_joint([](JointData* joint_data, float* args) {if (joint_data->is_used) joint_data->motor.enabled = 0;});
        return;
    }
    void fatal(Exo* exo, ExoData* exo_data, int error_code)
    {
        exo_data->for_each_joint([](JointData* joint_data, float* args) {if (joint_data->is_used) joint_data->motor.enabled = 0;});
        return;
    }
}

#endif

#endif // defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)