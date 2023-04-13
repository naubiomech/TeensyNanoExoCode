#ifndef RCBATTERY_H
#define RCBATTERY_H


#include "I2CHandler.h"
#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

class _Battery 
{
    public:
        /* Initialize any variables or peripherals */
        virtual void init() = 0;
        /* Collect and return the parameter of interest */
        virtual float get_parameter() = 0;
};

class SmartBattery: public _Battery
{
    public:
        void init();
        float get_parameter();
    
    private:
        uint8_t data[i2c_cmds::smart::get_battery_voltage::len];
};

class RCBattery: public _Battery
{
    public:
        void init();
        float get_parameter();

    private:
        uint8_t data[i2c_cmds::rc::get_battery_voltage::len];
        const int BusLSB = 4; 
};
#endif
#endif
