#ifndef ANKLEIMU_H
#define ANKLEIMU_H
#include <Adafruit_BNO055.h>

class AnkleIMU
{
    public:
    AnkleIMU(bool is_left);
    float get_global_angle();

    private:
    Adafruit_BNO055 _imu;
    int _addr;
    const bool _is_left;
    bool _initialized{false};
    
};


#endif