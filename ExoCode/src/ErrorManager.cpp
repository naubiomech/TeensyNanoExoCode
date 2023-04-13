#include "ErrorManager.h"
#include "Logger.h"
#include <Arduino.h>
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

int ErrorManager::_system_error_code = NO_ERROR;
int ErrorManager::_reported_error_code = NO_ERROR;

ErrorManager::ErrorManager(Exo* exo, ExoData* exo_data)
: _exo(exo), _data(exo_data)
{
    _set_handlers = false;
    _set_triggers = false;
}

void ErrorManager::set_system_error(int to_code)
{
    _system_error_code = to_code;
}

int ErrorManager::get_error()
{
    return ErrorManager::_reported_error_code;
}

bool ErrorManager::check()
{
    if ((!_set_handlers || !_set_triggers) && _system_error_code == NO_ERROR)
    {
        return false;
    }

    int soft_error_code = _soft_trigger(_exo, _data);
    if (soft_error_code != NO_ERROR)
    {
        _soft_handler(_exo, _data, soft_error_code);
    }
    int hard_error_code = _hard_trigger(_exo, _data);
    if (hard_error_code != NO_ERROR)
    {
        _hard_handler(_exo, _data, hard_error_code);
    }
    int fatal_error_code = _fatal_trigger(_exo, _data);
    if (fatal_error_code != NO_ERROR)
    {   
        _fatal_handler(_exo, _data, fatal_error_code);
    }
    
    // If _system_error_code != NO_ERROR, check its severity and handle appropriately
    if (_system_error_code != NO_ERROR)
    {
        if (_system_error_code >= FATAL_ERROR)
        {
            _fatal_handler(_exo, _data, _system_error_code);
        }
        else if (_system_error_code >= HARD_ERROR)
        {
            _hard_handler(_exo, _data, _system_error_code);
        }
        else if (_system_error_code >= SOFT_ERROR)
        {
            _soft_handler(_exo, _data, _system_error_code);
        }
    }
    
    // Return the highest error code
    int error_code = max(fatal_error_code, max(hard_error_code, soft_error_code));
    error_code = max(error_code, _system_error_code);
    return _new_error(error_code);
}

void ErrorManager::assign_handlers(error_handler_t soft, error_handler_t hard, error_handler_t fatal)
{
    if (soft == nullptr || hard == nullptr || fatal == nullptr)
    {
        logger::println("ErrorManager::assign_handlers: null pointer passed to function.", LogLevel::Error);
        return;
    }
    _soft_handler = soft;
    _hard_handler = hard;
    _fatal_handler = fatal;
    _set_handlers = true;
}

void ErrorManager::assign_triggers(error_trigger_t soft, error_trigger_t hard, error_trigger_t fatal)
{
    if (soft == nullptr || hard == nullptr || fatal == nullptr)
    {
        logger::println("ErrorManager::assign_triggers: null pointer passed to function.", LogLevel::Error);
        return;
    }
    _soft_trigger = soft;
    _hard_trigger = hard;
    _fatal_trigger = fatal;
    _set_triggers = true;
}

bool ErrorManager::_new_error(int working_error_code)
{
    if (ErrorManager::_system_error_code != ErrorManager::_reported_error_code && ErrorManager::_system_error_code != NO_ERROR)
    {
        logger::println("ErrorManager::_new_error: System Error", LogLevel::Error);
        ErrorManager::_reported_error_code = ErrorManager::_system_error_code;
        ErrorManager::_system_error_code = NO_ERROR;
        return true;
    }
    if (working_error_code != ErrorManager::_reported_error_code && working_error_code != NO_ERROR)
    {
        logger::println("ErrorManager::_new_error: Working Error", LogLevel::Error);
        ErrorManager::_reported_error_code = working_error_code;
        return true;
    }

    return false;
}

#endif
