# Adding New Sensors

## Location
Because the sensors don't have a common interface it is a bit more straight forward to add new ones.
The main things you will need to decide is what part of the system makes the most sense for the part to live, e.g. exo, side, joint.

## Create the Sensor
The sensor should be its own class.
You will need to create a .h and .cpp file in the src folder.
This should first be done in the [system check folder](/ExoCode/systemCheck) to make sure everything works before migrating it over to the main code.
The sensor class will have all the functions that you need to use the class, below you can find an example to build from.
This example is an outline of what would be in the header(.h) file.

```
class CLASS_NAME
{
    /*
     * public functions are ones that can be called from outside the class.
     */
    public: 
        /*
         * Constuctor for the class
         */
        CLASS_NAME(int pin);// you may not need the pin if it doesn't use an analog input.
        
        /*
         * Checks if the sensor needs to be calibrated and returns if the calibration is done.
         * This should be called every time the containing system is run.
         * Depending on your system you may not need to calibrate.
         */
        bool calibrate(bool do_calibrate);
        
        /*
         * Used to read the sensor and returns the value.
         * If you need multiple returns you will need to pass the place the date will go by pointer: 
         * void read(float* val0, float* val1);
         * Where it will be called using:
         * read(&place_to_store_val0, &place_to_store_val1);
         */
         float read();
    /*
     * Private functions and variables can only be called from within the class.
     */
    private:
        
        /* 
         * Sample private function this will be very dependent on what the system needs.
         */
        void _sample_function();
        
        // The pin the sensor is connected to.
        int _pin;
        
        // sample private variable for the sensor.
        float _sample_var;
};
```

## Migrating to the Main Code.

### Board.h
For each board that will use the sensor add the pin that each sensor will use.
If there are four sensors you will need to define 4 pins.
This assumes it is an analog sensor, if you are using a different type of sensor you can define the things that sensor needs here.
Make sure that that pin is free, this should be easier if you design the PCB first.

### System Containing Sensor Data .h
Create a variable to store the the data from the sensor.
If the sensor will be at the side level this will be in the SideData class, if it is the joint the JointData class, etc.
Additionally if the sensor will need to be calibrated make a variable to store that state;
```
float sensor_reading;
bool sensor_calibrate;
```

### System Containing Sensor Data .cpp
In the constructor related the the .h file you just modified, initialize the value to to what you want it to start at, this will often be 0.

```
sensor_reading = 0;
sensor_calibrate = false;
```

### System Containing Sensor .h
This will be Side.h if the sensor is at the side level or Joint.h if it is at the joint level, etc.
Add the include for the sensor, ```#include Sensor.h```, that you created.
Within the class; Side, Joint, etc.; declare an instance of the class ```SensorClass new_instance_of_sensor;```
Create as many instances as you need.

### System Containing Sensor .cpp
1. Within the .cpp file related to the .h file you just changed you need to call the constructor for the instances of the class you just declared.
This will be done in the initializer list.
For example for a joint level sensor:
```
Joint::Joint(config_defs::joint_id id, uint8_t* config_to_send)
: motor(id, config_to_send)
, controller(id, config_to_send)
, new_instance_of_sensor(pin_to_use)
{
```

2. Add a call to the sensor.read() in the containing classes read referencing the sensor data we already made.
For a joint level sensor:
``` 
_joint_data->sensor_reading = new_instance_of_sensor.read();
```

3. Add a call to the sensor.calibrate(do_calibrate) in the containing classes read referencing the sensor data we already made.
For a joint level sensor:
``` 
_joint_data->sensor_calibrate = new_instance_of_sensor.calibrate(_joint_data->sensor_calibrate);
```

This will get called every time the joint is run and if the calibrate variable is set to true will calibrate the sensor.
