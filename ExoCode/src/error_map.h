/**
 * @file error_map.h
 * @author Chancelor Cuddeback
 * @brief Maps error codes to error types

 */

#ifndef ERROR_MAPS_H
#define ERROR_MAPS_H
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "error_codes.h"
#include "error_types.h"
#include <map>

std::map<ErrorCodes, ErrorType*> error_map = {

};

#endif
#endif // ERROR_MAPS_H