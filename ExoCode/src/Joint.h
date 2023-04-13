/**
 * @file Joint.h
 *
 * @brief Declares a class for controlling and reading sensors for a joint
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Joint_h
#define Joint_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "Motor.h"
#include "Controller.h"
#include "TorqueSensor.h"
#include "ParseIni.h"
#include "board.h"
#include "Joint.h"
#include "config.h"
#include "Utilities.h"
#include "StatusDefs.h"
#include "AnkleAngles.h"
#include "AnkleIMU.h"

#include <stdint.h>

/**
 * @brief Abstract class used to define the interface for the other joints.
 * all joints should have a:
 * void run_joint()
 * void read_data()
 * check_calibration()
 * void set_controller(uint8_t)
 * void set_motor(_Motor*);
 * 
 */
class _Joint
{
	static uint8_t left_torque_sensor_used_count; /**< Used to record how many sensors are already set */
    static uint8_t right_torque_sensor_used_count;/**< Used to record how many sensors are already set */
    
    static uint8_t left_motor_used_count;/**< Used to record how many motors are already set for torque sensor pin assignment */
    static uint8_t right_motor_used_count;/**< Used to record how many motors are already set for torque sensor pin assignment */
    
    
    // TODO: create object for each type of controller (joint type specific) and pointer to current controller.
    // TODO: Create object for specific motor used based on config.
    // TODO: Create joint base class that can then be used for each joint so that hip, knee, and ankle can each have joint specific controllers.
    public:
		/**
         * @brief Constructor 
         * 
         * @param id of the joint being used
         * @param pointer to the full ExoData instance
         */
        _Joint(config_defs::joint_id id, ExoData* exo_data);  // constructor:  
		virtual ~_Joint(){};
        
        /**
         * @brief updates the controller and sends the motor command
         */
        virtual void run_joint() = 0;   
		
        /**
         * @brief reads data from motor and sensors
         */
        virtual void read_data(); 

        /**
         * @brief Checks if we need to do the calibration for the motor and sensors
         * and runs the calibration.
         */
        virtual void check_calibration();         
		
        /**
         * @brief changes the high level controller in Controller, and the low level controller in Motor
         * 
         * @param controller id for that joint
         */
        virtual void set_controller(uint8_t) = 0;  
		
        /**
         * @brief Sets the motor to use.  Not strictly needed since everything stays internal.
         *
         * @param pointer to motor instance
         */
        void set_motor(_Motor* new_motor);
        
        // create some static member functions we can use for the initializer list.
        
        /**
         * @brief Takes in the joint id and exo data, and checks if the current joint is used.
         * If it is used it pulls the next open torque sensor pin for the side, and increments the counter.
         * If the joint is not used, or we have used up all the available torque sensor pins for the side, it sets the pin to a pin that is not connected.
         *
         * @param joint id
         * @param ExoData pointer
         * 
         * @return next pin to assign
         */
        static unsigned int get_torque_sensor_pin(config_defs::joint_id, ExoData*);
        
        /**
         * @brief Takes in the joint id and exo data, and checks if the current joint is used.
         * If it is used it pulls the next open motor enable pin for the side, and increments the counter.
         * If the joint is not used, or we have used up all the available torque sensor pins for the side, it sets the pin to a pin that is not connected.
         *
         * @param joint id
         * @param ExoData pointer
         * 
         * @return next pin to assign
         */
        static unsigned int get_motor_enable_pin(config_defs::joint_id, ExoData*);

        /** MOVE THESE BACK TO PROTECTED WHEN APP IS READY*/
        _Motor* _motor; /**< pointer to the base _Motor class so we can use any motor type.*/
        TorqueSensor _torque_sensor; /**< Torque sensor for the joint*/
        _Controller* _controller; /**< Pointer to the current controller.  Using pointer so we just need to change the object we are pointing to when the controller changes.*/

        
    protected:
        // give access to the larger data object and the joint specific data 
        ExoData* _data;/**< pointer to the full data instance*/
        JointData* _joint_data;/**< pointer to this joints data */
        
        // IO objects for the joint
        //_Motor* _motor; /**< pointer to the base _Motor class so we can use any motor type.*/
		//TorqueSensor _torque_sensor; /**< Torque sensor for the joint*/
		//_Controller* _controller; /**< Pointer to the current controller.  Using pointer so we just need to change the object we are pointing to when the controller changes.*/
        
        // joint info
        config_defs::joint_id _id; /**< joint id */
        bool _is_left; /**< if the joint is on the left side so we don't have to keep calculating it*/
    
};

/**
 * @brief class for the hip joint which contains joint specific controllers.
 */
class HipJoint : public _Joint
{
    public:
        HipJoint(config_defs::joint_id id, ExoData* exo_data);
        ~HipJoint(){};
        
        /**
         * @brief reads the sensors for the joint and sends a torque command, See _Joint
         */
        void run_joint();  
        //void read_data(); // See _Joint 
        
        /**
         * @brief sets the controller that is to be used, See _Joint
         *
         * @param controller id.
         */
        void set_controller(uint8_t);  
    protected:
        // Objects for joint specific controllers

        ZeroTorque _zero_torque; /**< zero torque controller */
        HeelToe _heel_toe; /**< heel toe controller, not currently configured */
        ExtensionAngle _extension_angle; /**< extension angle controller */
        BangBang _bang_bang; /**< bang bang controller */
        LateStance _late_stance; /**<late stance controller */
        GaitPhase _gait_phase; /**<gait phase controller */
        FranksCollinsHip _franks_collins_hip; /**< Franks Collins Hip controller */
        // UserDefined _user_defined; /**< user defined controller*/
        Sine _sine; /**< sine wave controller */
        Stasis _stasis; /**< stasis controller */
        Perturbation _perturbation;    /**< perturbation controller */
        Parabolic _parabolic;    /**< parabolic controller */
        ConstantTorque _constant_torque; /**< constant torque controller*/
        PtbGeneral _ptb_general; /**< Generalized Perturbation Controller>*/
};

/**
 * @brief class for the knee joint which contains joint specific controllers.
 */
class KneeJoint : public _Joint
{
    public:
        KneeJoint(config_defs::joint_id id, ExoData* exo_data);
        ~KneeJoint(){};
        
        /**
         * @brief reads the sensors for the joint and sends a torque command, See _Joint
         */
        void run_joint();  // See _Joint
        //void read_data(); // See _Joint
        
        /**
         * @brief sets the controller that is to be used, See _Joint
         *
         * @param controller id.
         */
        void set_controller(uint8_t);  // See _Joint
	
    protected:
        // Objects for joint specific controllers	
        ZeroTorque _zero_torque; /**< zero torque controller */
        // UserDefined _user_defined; /**< user defined controller*/
        Sine _sine; /**< sine wave controller */
        Stasis _stasis; /**< stasis controller */
        Perturbation _perturbation;    /**< perturbation controller */
        ConstantTorque _constant_torque; /**< constant torque controller*/
        ElbowMinMax _elbow_min_max;
};

/**
 * @brief class for the ankle joint which contains joint specific controllers.
 */ 
class AnkleJoint : public _Joint
{
    public:
        AnkleJoint(config_defs::joint_id id, ExoData* exo_data);
        ~AnkleJoint(){};
        
        /**
         * @brief reads the sensors for the joint and sends a torque command, See _Joint
         */
        void run_joint();  // See _Joint
        //void read_data(); // See _Joint
        
        /**
         * @brief sets the controller that is to be used, See _Joint
         *
         * @param controller id.
         */
        void set_controller(uint8_t);  // See _Joint
		
    protected:
        AnkleIMU _imu;
        const float _imu_sample_rate_us{15000.0f};
        float _previous_sample_us{0.0f};
        AnkleAngles _ankle_angle;
        // Objects for joint specific controllers
        ZeroTorque _zero_torque;  /**< zero torque controller */
        ProportionalJointMoment _proportional_joint_moment;/**< Proportional joint moment controller */
        ZhangCollins _zhang_collins;/**< Zhang Collins controller */
        // UserDefined _user_defined; /**< user defined controller*/
        Sine _sine; /**< sine wave controller */
        Stasis _stasis; /**< stasis controller */
        Perturbation _perturbation;    /**< perturbation controller */
        ConstantTorque _constant_torque; /**< constant torque controller*/
        PtbGeneral _ptb_general; /**< Generalized Perturbation Controller>*/
        PropulsiveAssistive _propulsive_assistive; /**< Propulsive Assistive */
        
};

#endif
#endif