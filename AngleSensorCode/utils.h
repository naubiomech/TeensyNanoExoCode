#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <stdint.h>

namespace utils
{
    /*
     * Used to convert between floats and bytes 
     * 
     * !! NOT to be used outside of utils
     */
    union FloatByteUnion
    {
        float f;
        uint8_t b[sizeof(float)];
    };
    
    union ShortIntByteUnion
    {
        short int i;
        uint8_t b[sizeof(short int)];
    };
    
    bool is_little_endian()
    {
        FloatByteUnion val;
        switch (sizeof(float))
        {
            case 4: // 32 bit
                val.f = 1.401298464324817e-45;  // only works for 32 bit floats, arduino doesn't handle -0.
                return val.b[0] == 0x01;
                break;
            case 8: // 64 bit
                val.f = 4.94065645841246544176568792868E-324;  // only works for 64 bit floats, arduino doesn't handle -0.
                return val.b[0] == 0x01;
                break;
            default:
                //logger::println("Utilities :: is_little_endian() : System does not appear to be 32 or 64 bit");    
                break;
        }
        
    }
    
    void float_to_uint8(float num_to_convert, uint8_t *converted_bytes)
    {
        FloatByteUnion val;
        val.f = num_to_convert;
        int idx;
        // logger::println(val.f);
        for(uint8_t i = 0; i<sizeof(float); i++)
        {
            if (is_little_endian())
            {
                idx = i;
            }
            else
            {
                idx = sizeof(float)-i-1;
            }
            // logger::println(idx);
            // logger::println(val.b[0],HEX);
            converted_bytes[i] = val.b[idx];
            // logger::println(converted_bytes[i],HEX);
        }
        return;
    }
    
    void uint8_to_float(uint8_t *bytes_to_convert, float *converted_float)
    {
        FloatByteUnion val;
        int idx;
        // logger::println(bytes_to_convert[0],HEX);
        
        // flip the idx if not little endian
        for(uint8_t i = 0; i<sizeof(float); i++)
        {
            if (is_little_endian())
            {
                idx = i;
            }
            else
            {
                idx = sizeof(float)-i-1;
            }
            // logger::println(idx);
            val.b[i] = bytes_to_convert[idx];
            // logger::println(val.b[i],HEX);
        }
        
        *converted_float = val.f;
        // logger::println(*converted_float);
        
         
        return;
    }
    
    void float_to_short_fixed_point_bytes(float num_to_convert, uint8_t *converted_bytes, uint8_t factor)
    {
        ShortIntByteUnion val;
        val.i = (short int) (num_to_convert * factor);
        int idx;
        // logger::println(val.f);
        for(uint8_t i = 0; i<sizeof(short int); i++)
        {
            if (is_little_endian())
            {
                idx = i;
            }
            else
            {
                idx = sizeof(short int)-i-1;
            }
            // logger::println(idx);
            // logger::println(val.b[0],HEX);
            converted_bytes[i] = val.b[idx];
            // logger::println(converted_bytes[i],HEX);
        }
        return;
    }
    
    void short_fixed_point_bytes_to_float(uint8_t *bytes_to_convert, float *converted_val, uint8_t factor)
    {
        ShortIntByteUnion val;
        int idx;
        // logger::println(bytes_to_convert[0],HEX);
        
        // flip the idx if not little endian
        for(uint8_t i = 0; i<sizeof(short int); i++)
        {
            if (is_little_endian())
            {
                idx = i;
            }
            else
            {
                idx = sizeof(short int)-i-1;
            }
            // logger::println(idx);
            val.b[i] = bytes_to_convert[idx];
            // logger::println(val.b[i],HEX);
        }
        
        *converted_val = ((float)val.i/factor);
        // logger::println(*converted_float);
        
         
        return;
    }
}

#endif