/**
 * @file Controller.h
 *
 * @brief Declares for the different controllers the exo can use. 
 * Controllers should inherit from _Controller class to make sure the interface is the same.
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef Controller_h
#define Controller_h

// Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include <Arduino.h>

#include "ExoData.h"
#include "Board.h"
#include "ParseIni.h"
#include <stdint.h>
#include "Utilities.h"
#include "config.h"
#include "Time_Helper.h"
#include <algorithm>
#include <utility>

/**
 * @brief This class defines the interface for controllers.  
 * All controllers must have a:
 * float calc_motor_cmd() that returns a torque cmd in Nm.  
 * 
 */
class _Controller
{
	public:
        /**
         * @brief Constructor 
         * 
         * @param id of the joint being used
         * @param pointer to the full ExoData instance
         */
        _Controller(config_defs::joint_id id, ExoData* exo_data);
		
        /**
         * @brief Virtual destructor is needed to make sure the correct destructor is called when the derived class is deleted.
         */
        virtual ~_Controller(){};
		//virtual void set_controller(int joint, int controller) = 0; // Changes the controller for an individual joint
        
        /**
         * @brief Virtual function so that each controller must create a function that will calculate the motor command
         * 
         * @return Torque in Nm.
         */
		virtual float calc_motor_cmd() = 0; 
        
        /**
         * @brief Resets the integral sum for the controller
         */
        void reset_integral(); 
        
    protected:
        
        ExoData* _data; /**< pointer to the full data instance*/
        ControllerData* _controller_data; /**< Pointer to the data associated with this controller */
        LegData* _leg_data; /**< pointer for the leg data the controller is associated with */
        JointData* _joint_data; /**< pointer to the joint data the controller is associated with */
         
        config_defs::joint_id _id;  /**< id of the joint this controller is attached to. */
        
        Time_Helper* _t_helper;  /**< instance of the time helper to track when things happen used to check if we have a set time for the PID */
        float _t_helper_context; /**< store the context for the timer helper */
        float _t_helper_delta_t; /**< time time since the last event */



        // Values for the PID controller
        float _integral_val; /**< sum of the error integral */
        float _prev_input;   /**< prev error term for calculating derivative */
        float _prev_de_dt;   /**< prev error derivative used if the timestep is not good*/
        float _prev_pid_time; /**< prev time the PID was called */
        
        /**
         * @brief calculates the current PID contribution to the motor command. 
         * Currently integral is commented out as resetting _integral_val was crashing the system 
         * 
         * @param controller command 
         * @param measured controlled value
         * @param proportional gain
         * @param integral gain
         * @param derivative gain
         */
        float _pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain);
        
        // Values for the Compact Form Model Free Adaptive Controller
        std::pair<float, float> measurements;
        std::pair<float, float> outputs;
        std::pair<float, float> phi; /**< psuedo partial derivative */
        float rho; /**< penalty factor (0,1) */
        float lamda; /**< weighting factor limits delta u */
        float etta; /**< step size constant (0, 1] */
        float mu; /**< weighting factor that limits the variance of u */
        float upsilon; /**< a sufficiently small integer ~10^-5 */
        float phi_1; /**< initial/reset condition for estimation of psuedo partial derivitave */
        
        float _cf_mfac(float reference, float current_measurement);
};

class PropulsiveAssistive : public _Controller
{
    public:
        PropulsiveAssistive(config_defs::joint_id id, ExoData* exo_data);
        ~PropulsiveAssistive(){};

        float calc_motor_cmd();
    private:
        void _update_reference_angles(LegData* leg_data, ControllerData* controller_data, float percent_grf);
};

/**
 * @brief Proportional Joint Moment Controller
 * This controller is for the ankle joint 
 * Applies a plantar torque based on the normalized magnitude of the toe FSR
 *
 * 2022-02 : This controller has been around the lab for a while I don't know the original origin -P.Stegall
 *
 * see ControllerData.h for details on the parameters used.
 */
class ProportionalJointMoment : public _Controller
{
    public:
        ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data);
        ~ProportionalJointMoment(){};
        
        float calc_motor_cmd();
    private:
        std::pair<float, float> _stance_thresholds_left, _stance_thresholds_right;
        
        float _inclination_scaling{1.0f};
};


/**
 * @brief Zero Torque Controller
 * This controller is for the any joint
 * Simply applies zero torque
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ZeroTorque : public _Controller
{
    public:
        ZeroTorque(config_defs::joint_id id, ExoData* exo_data);
        ~ZeroTorque(){};
        
        float calc_motor_cmd();
};

/**
 * @brief Stasis Controller 
 * This controller is for the any joint
 * Simply applies zero torque cmd to motor
 * 
 * see ControllerData.h for details on the parameters used.
 */
class Stasis : public _Controller
{
    public:
        Stasis(config_defs::joint_id id, ExoData* exo_data);
        ~Stasis(){};
        
        float calc_motor_cmd();
};

/**
 * @brief Heel Toe Controller
 * This controller is for the hip joint 
 * Applies torque based on the heel and toe readings with some adjustments for swing
 *
 * 2022-02 : This controller is based on the work of Safoura Sadegh Pour
 * 
 * 2023-01 : Updated version of the controller implamented by Jack Williams (jack.williams@nau.edu for questions)
 * 
 * see ControllerData.h for details on the parameters used.
 */
class HeelToe: public _Controller
{
    public:
        HeelToe(config_defs::joint_id id, ExoData* exo_data);
        ~HeelToe(){};
        
        float calc_motor_cmd();

        float cmd_ff;                           /**< Motor Command Calculated within Controller */

        float percent_gait;                      /**< Records what percentage of the gait cycle we are currently in. Function to estimate this can be found in Leg.cpp */
        float prev_percent_gait;

        float fs;                               /**< Estimation of ground reaction force based on 'Bishe 2021' */
        float fs_previous;                      /**< Stores previous estimate of fs. Used to determine direciton of the slope of the derivative. */

        int state;                              /**< Stores what state the contoller is in, motor command calculation differs by state. */
        int prev_state;                         /**< Stores the previous state of the controller. */

        float state_4_start;                    /**< Stores the percent of the gait cycle where the 4th state begins. */
        float state_4_end;                      /**< Stores the percent of gait cycle where the 4th state ends. */
        float previous_state_4_duration;        /**< Stores the previous duration, in terms of percent gait, of state 4. */
        float prev_cmd;                         /**< Variable that stores last torque command */
        float state_4_start_cmd;                /**< Stores the last command before the start of state 4. */

        float swing_start;                      /**< Stores the starting point of swing, in terms of percent gait. */
        float swing_duration;                   /**< Stores the duration of swing, in terms of percent gait, which can be determined as soon as the starting point of swing occurs. */

        float m;                                /**< Slope of the line used in State 5. */

        /**< Coordinates for Parabola used in State 4. */
        float x1;
        float y1;
        float x2;
        float y2;
        float x3;
        float y3;

        /**< Constants for Parabola used in State 4. */
        float A;
        float B;
        float C;
};

/**
 * @brief Extension Angle Controller
 * This controller is for the hip joint 
 * Applies torque based on the angle of the hip while extending, 
 * then a const torque while flexing
 * 
 * 2022-02 : This controller was conceptualized by Z. Lerner and written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ExtensionAngle: public _Controller
{
    public:
        ExtensionAngle(config_defs::joint_id id, ExoData* exo_data);
        ~ExtensionAngle(){};
        
        float calc_motor_cmd();
    private:
       /**
        * @brief calculates the largest angle that the joint has experienced and updates the _max_angle and _min_angle members.
        * 
        * @param current angle for the joint
        */
        void _update_max_angle(float angle);
        
        /**
        * @brief updates the state for if we are doing the hip or the swing phase control
        * 
        * @param current angle for the joint
        */
        void _update_state(float angle);
        
        /**
        * @brief Resets the _max_angle and _min_angle members to the default values. 
        */
        void _reset_angles();
        
        // the initial angles used for tracking the extent of the range of motions 
        const float _initial_max_angle = utils::degrees_to_radians(10); /**< Initial value to set the max angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        const float _initial_min_angle = utils::degrees_to_radians(-5); /**< Initial value to set the min angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        
        // Used to track the range of motion, angles are in rad.
        float _max_angle; /**< The max angle the joint has experienced */
        float _min_angle; /**< The max angle the joint has experienced */
        
        uint8_t _state; /**< Used to track the state 0 is extension mode, 1 is flexion mode. */
      
};

/**
 * @brief BangBang Controller
 * This controller is for the hip joint, but can potentially be applied to other joints 
 * Applies const torque while extending, 
 * then a const torque while flexing
 * 
 * 2022-04 : Based on extension angle controller written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class BangBang: public _Controller
{
    public:
        BangBang(config_defs::joint_id id, ExoData* exo_data);
        ~BangBang(){};
        
        float calc_motor_cmd();
    private:
        /**
        * @brief calculates the largest angle that the joint has experienced and updates the _max_angle and _min_angle members.
        * 
        * @param current angle for the joint
        */
        void _update_max_angle(float angle);
        
        /**
        * @brief updates the state for if we are doing the hip or the swing phase control
        * 
        * @param current angle for the joint
        */
        void _update_state(float angle);
        
        /**
        * @brief Resets the _max_angle and _min_angle members to the default values. 
        */
        void _reset_angles();
        
        // the initial angles used for tracking the extent of the range of motions 
        const float _initial_max_angle = utils::degrees_to_radians(20); /**< Initial value to set the max angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        const float _initial_min_angle = utils::degrees_to_radians(-5); /**< Initial value to set the min angle to, should be a physiologically feasible value that a person is likely to exceed but won't cause the controller to act strange. */
        
        // Used to track the range of motion, angles are in rad.
        float _max_angle; /**< The max angle the joint has experienced */
        float _min_angle; /**< The max angle the joint has experienced */
        
        uint8_t _state; /**< Used to track the state 0 is extension mode, 1 is flexion mode. */
      
};

/*
 * LateStance Controller
 * This controller is for the hip joint
 * Applies const torque during late stance
 *
 * see ControllerData.h for details on the parameters used.
 */
class LateStance : public _Controller
{
public:
    LateStance(config_defs::joint_id id, ExoData* exo_data);
    ~LateStance(){};

    float calc_motor_cmd();
private:
    void _update_max_angle(float angle);
    void _update_state(float angle);
    void _reset_angles();

    // the initial angles used for tracking the extent of the range of motions 
    const float _initial_max_angle = utils::degrees_to_radians(20);
    const float _initial_min_angle = utils::degrees_to_radians(-5);

    // Used to track the range of motion, angles are in rad.
    float _max_angle;
    float _min_angle;

    // Used to track the state 0 is extension mode, 1 is flexion mode.
    uint8_t _state;

};

/*
 * GaitPhase Controller
 * This controller is for the hip joint
 * Applies flexion or extension torque as a function of the gait phase (needs heel and toe FSRs)
 *
 * see ControllerData.h for details on the parameters used.
 * 
 * This controller is still in development, this is just meant to be a framework for the controller that will be filled out with time
 */
class GaitPhase : public _Controller
{
    public:
        GaitPhase(config_defs::joint_id id, ExoData* exo_data);
        ~GaitPhase(){};

        float calc_motor_cmd();

        float slope;
        float state;

};

/*
 * Parabolic Controller
 * This controller is for the hip joint
 * Applies flexion or extension torque as a function of the gait phase (needs heel and toe FSRs)
 *
 * see ControllerData.h for details on the parameters used.
 *
 * This controller is still in development, this is just meant to be a framework for the controller that will be filled out with time
 */
class Parabolic : public _Controller
{
public:
    Parabolic(config_defs::joint_id id, ExoData* exo_data);
    ~Parabolic() {};

    float calc_motor_cmd();
};

/**
 * @brief Zhang Collins Controller
 * This controller is for the ankle joint 
 * Applies ramp between t0 and t1 to (t1, ts).
 * From t1 to t2 applies a spline going up to (t2,mass*normalized_peak_torque).
 * From t2 to t3 falls to (t3, ts)
 * From t3 to 100% applies zero torque
 * 
 * 2022-02 : This controller was based on:
 * Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. 
 * (2017). Human-in-the-loop optimization of exoskeleton assistance during walking. Science, 356(6344), 1280-1284.
 * and written by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class ZhangCollins: public _Controller
{
    public:
        ZhangCollins(config_defs::joint_id id, ExoData* exo_data);
        ~ZhangCollins(){};
        
        float calc_motor_cmd();

        float _spline_generation(float node1, float node2, float node3, float torque_magnitude, float percent_gait);

        float torque_cmd;
		
		float cmd;
};

/**
 * @brief Franks Collins Controller
 * This controller is for the Hip Joint
 *
 * Is 0 between t0_trough and t1_trough to (t1, 0).
 * From t1_trough to t2_trough applies a spline going down to (t2_trough,mass*normalized_trough_torque).
 * From t2_trough to t3_trough rises to (t3_trough, 0)
 * From t3_trough to t1_peak applies zero torque
 * From t1_peak to t2_peak applies a spline going up to (t2_peak,mass*normalized_peak_torque).
 * From t2_peak to t3_peak falls to (t3_peak, 0)
 *
 * 2022-04 : This controller was based on:
 * Franks, P. W., Bryan, G. M., Martin, R. M., Reyes, R., Lakmazaheri, A. C., & Collins, S. H.
 * (2021). Comparing optimized exoskeleton assistance of the hip, knee, and ankle in single and multi-joint configurations. Wearable Technologies, 2.
 * and written by P. Stegall
 *
 * see ControllerData.h for details on the parameters used.
 */
class FranksCollinsHip: public _Controller
{
    public:
        FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data);
        ~FranksCollinsHip(){};
       
        float calc_motor_cmd();

        float _spline_generation(float node1, float node2, float node3, float torque_magnitude, float shifted_percent_gait);

        float last_percent_gait;
        float last_start_time;
       
};

/**
 * @brief UserDefined Controller
 * This controller uses a simple lookup table with interpolation between values.
 * Assumes table is evenly spaced across percent gait, 0 to 100 where f(0) = f(100)
 * ex. controller_data
 * 
 * 2022-05 : by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
// class UserDefined: public _Controller
// {
    // public:
        // UserDefined(config_defs::joint_id id, ExoData* exo_data);
        // ~UserDefined(){};
        
        // float calc_motor_cmd();
    // private:
        // float _percent_x[controller_defs::user_defined::num_sample_points];
        // const float _step_size = 100/controller_defs::user_defined::num_sample_points;
        
        
// };

/**
 * @brief Sine Controller
 * This controller plays a sine wave with a defined amplitude, period, and phase shift
 * returns amp * sine (frac_of_period * 2 * pi + phase_shift)
 * where frac_of_period is (time % period)/period
 * 
 * 2022-05 : by P. Stegall
 * 
 * see ControllerData.h for details on the parameters used.
 */
class Sine: public _Controller
{
    public:
        Sine(config_defs::joint_id id, ExoData* exo_data);
        ~Sine(){};
        
        float calc_motor_cmd();
    private:
        
        
};

/**
 * @brief Perturbation Controller
 * This controller is meant to work with both the hip and ankle exoskeletons
 * It can apply a torque with a defined amplitude for a user defined duration of time in a user defined direciton (PF/DF)
 *
 * 2022-11 : by J. Williams
 *
 * see ControllerData.h for details on the parameters used.
 */

class Perturbation : public _Controller
{
public:
    Perturbation(config_defs::joint_id id, ExoData* exo_data);
    ~Perturbation() {};

    float calc_motor_cmd();
};

class ConstantTorque : public _Controller
{
public:
    ConstantTorque(config_defs::joint_id id, ExoData* exo_data);
    ~ConstantTorque() {};

    float calc_motor_cmd();

    float current_torque;
    int counter;
};

class ElbowMinMax : public _Controller
{
public:
    ElbowMinMax(config_defs::joint_id id, ExoData* exo_data);
    ~ElbowMinMax() {};

    float calc_motor_cmd();
    
};

class PtbGeneral : public _Controller
{
public:
    PtbGeneral(config_defs::joint_id id, ExoData* exo_data);
    ~PtbGeneral() {};

    float calc_motor_cmd();
};

class CalibrManager : public _Controller
{
public:
    CalibrManager(config_defs::joint_id id, ExoData* exo_data);
    ~CalibrManager() {};

    float calc_motor_cmd();
};

#endif
#endif