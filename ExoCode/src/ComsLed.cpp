#include "ComsLed.h"
#include "Board.h"

#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)

ComsLed::ComsLed()
{
    // set the rgb pins to output
    pinMode(coms_micro_pins::red, OUTPUT);
    pinMode(coms_micro_pins::green, OUTPUT);
    pinMode(coms_micro_pins::blue, OUTPUT);
    // set the initial color to 0
    this->r = 0;
    this->g = 0;
    this->b = 0;
    analogWrite(coms_micro_pins::red, r);
    analogWrite(coms_micro_pins::green, g);
    analogWrite(coms_micro_pins::blue, b);
}

ComsLed* ComsLed::get_instance()
{
    static ComsLed* instance = new ComsLed();
    return instance;
}

void ComsLed::set_color(uint8_t r, uint8_t g, uint8_t b)
{
    this->r = r;
    this->g = g;
    this->b = b;
    analogWrite(coms_micro_pins::red, _handle_active_low(r));
    analogWrite(coms_micro_pins::green, _handle_active_low(g));
    analogWrite(coms_micro_pins::blue, _handle_active_low(b));
}

void ComsLed::get_color(uint8_t* r, uint8_t* g, uint8_t* b)
{
    *r = this->r;
    *g = this->g;
    *b = this->b;
}

uint8_t ComsLed::_handle_active_low(uint8_t value)
{
    if (coms_micro_pins::led_active_low)
    {
        return 255 - value;
    }
    else
    {
        return value;
    }
}

#endif