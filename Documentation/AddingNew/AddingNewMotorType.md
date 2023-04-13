# Adding a New Motor Type

## Motor.h
1. Create a class for your new motor type that inherits its interface from the _Motor class
```
class _YourMotorType : public _Motor
{
    public:
        _YourMotorType(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_YourMotorType(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        void on_off();
        bool enable();
        bool enable(bool overide);
        void zero();
    protected:
        ...
```
2. Add any protected member variables or functions that will be needed to operate your motor.
3. Add a specific motor class of this type which inherits the motor type class you just created. 
``` 
class ANewMotor : public _YourMotorType
{
    public:
        ANewMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // constructor: type is the motor type
		~ANewMotor(){};
};
```
4. Add any protected member variables or functions that will be needed to operate your motor.

## Motor.cpp
1. Define the behavior of the member functions of _YourMotorType
    - Ideally these will be shared by all the motors of this type.
2. Define the behavior of the member functions and variables of the ANewMotor.

## Connect to everything else
1. Follow the steps in [AddingNewCanMotor.md](AddingNewCanMotor.md) to connect this new motor to the rest of the code base
    - This is not added here so we only have to update information in one place.