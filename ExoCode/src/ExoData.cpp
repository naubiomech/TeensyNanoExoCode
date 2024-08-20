#include "ExoData.h"
#include "error_codes.h"
#include "Logger.h"
#include "ParamsFromSD.h"

/*
 * Constructor for the exo data.
 * Takes the array from the INI parser.
 * Stores the exo status, and the sync LED state.
 * Uses an initializer list for the leg data. 
 */
ExoData::ExoData(uint8_t* config_to_send) 
: left_leg(true, config_to_send)            //Using initializer list for member objects.
, right_leg(false, config_to_send)
{
    this->_status = status_defs::messages::trial_off;
    this->sync_led_state = false;
    this->estop = false;

    this->config = config_to_send;
    this->config_len = ini_config::number_of_keys;

    this->mark = 10;  

    this->error_code = static_cast<int>(NO_ERROR);
    this->error_joint_id = 0;
    this->user_paused = false;
};

void ExoData::reconfigure(uint8_t* config_to_send) 
{
    left_leg.reconfigure(config_to_send);
    right_leg.reconfigure(config_to_send);
};

uint8_t ExoData::get_used_joints(uint8_t* used_joints)
{
    uint8_t len = 0;

    used_joints[len] = ((left_leg.hip.is_used) ? (1) : (0));
    len += left_leg.hip.is_used;
    used_joints[len] = ((left_leg.knee.is_used) ? (1) : (0));
    len += left_leg.knee.is_used;
    used_joints[len] = ((left_leg.ankle.is_used) ? (1) : (0));
    len += left_leg.ankle.is_used;
    used_joints[len] = ((left_leg.elbow.is_used) ? (1) : (0));
    len += left_leg.elbow.is_used;
    used_joints[len] = ((right_leg.hip.is_used) ? (1) : (0));
    len += right_leg.hip.is_used;
    used_joints[len] = ((right_leg.knee.is_used) ? (1) : (0));
    len += right_leg.knee.is_used;
    used_joints[len] = ((right_leg.ankle.is_used) ? (1) : (0));
    len += right_leg.ankle.is_used;
    used_joints[len] = ((right_leg.elbow.is_used) ? (1) : (0));
    len += right_leg.elbow.is_used;
    return len;
};

JointData* ExoData::get_joint_with(uint8_t id)
{
    JointData* j_data = NULL;
    switch (id)
    {
    case (uint8_t)config_defs::joint_id::left_hip:
        j_data = &left_leg.hip;
        break;
    case (uint8_t)config_defs::joint_id::left_knee:
        j_data = &left_leg.knee;
        break;
    case (uint8_t)config_defs::joint_id::left_ankle:
        j_data = &left_leg.ankle;
        break;
    case (uint8_t)config_defs::joint_id::left_elbow:
        j_data = &left_leg.elbow;
        break;
    case (uint8_t)config_defs::joint_id::right_hip:
        j_data = &right_leg.hip;
        break;
    case (uint8_t)config_defs::joint_id::right_knee:
        j_data = &right_leg.knee;
        break;
    case (uint8_t)config_defs::joint_id::right_ankle:
        j_data = &right_leg.ankle;
        break; 
    case (uint8_t)config_defs::joint_id::right_elbow:
        j_data = &right_leg.elbow;
        break;
    default:
        // logger::print("ExoData::get_joint_with->No joint with ");
        // logger::print(id);
        // logger::println(" was found.");
        break;
    }
    return j_data;
};

void ExoData::set_status(uint16_t status_to_set)
{
    //If the status is already error, don't change it
    if (this->_status == status_defs::messages::error)
    {
        return;
    }
    this->_status = status_to_set;
}

uint16_t ExoData::get_status(void)
{
    return this->_status;
}

void ExoData::set_default_parameters()
{
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    this->for_each_joint([this](JointData* j_data, float* args)
        {
            if (j_data->is_used)
            {
                set_controller_params((uint8_t)j_data->id, j_data->controller.controller, 0, this);
            }
        }
    );
#endif
}

void ExoData::set_default_parameters(uint8_t id)
{
    #if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    float f_id = static_cast<float>(id);
    this->for_each_joint(
        [this](JointData* j_data, float* args) 
        {
            if (j_data->is_used && (uint8_t)j_data->id == static_cast<uint8_t>(args[0]))
            {
                set_controller_params((uint8_t)j_data->id, j_data->controller.controller, 0, this);
            }
        },
        &f_id
    );
    #endif
}

void ExoData::start_pretrial_cal()
{
    //Calibrate the Torque Sensors
    this->for_each_joint([](JointData* j_data, float* args) {j_data->calibrate_torque_sensor = j_data->is_used;});
}

void ExoData::print()
{
    logger::print("\t Status : ");
    logger::println(_status);
    logger::print("\t Sync LED : ");
    logger::println(sync_led_state);
    
    if (left_leg.is_used)
    {
        logger::print("\tLeft :: FSR Calibration : ");
        logger::print(left_leg.do_calibration_heel_fsr);
        logger::println(left_leg.do_calibration_toe_fsr);
        logger::print("\tLeft :: FSR Refinement : ");
        logger::print(left_leg.do_calibration_refinement_heel_fsr);
        logger::println(left_leg.do_calibration_refinement_toe_fsr);
        logger::print("\tLeft :: Percent Gait : ");
        logger::println(left_leg.percent_gait);
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(left_leg.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(left_leg.toe_fsr);
        
        if(left_leg.hip.is_used)
        {
            logger::println("\tLeft :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.hip.controller.parameter_set);
        }
        
        if(left_leg.knee.is_used)
        {
            logger::println("\tLeft :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.knee.controller.parameter_set);
        }
        if(left_leg.ankle.is_used)
        {
            logger::println("\tLeft :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.ankle.controller.parameter_set);
        }

        if (left_leg.elbow.is_used)
        {
            logger::println("\tLeft :: Elbow");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_leg.elbow.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_leg.elbow.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_leg.elbow.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_leg.elbow.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_leg.elbow.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_leg.elbow.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_leg.elbow.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_leg.elbow.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_leg.elbow.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_leg.elbow.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_leg.elbow.controller.parameter_set);
        }
    }
    
    if (right_leg.is_used)
    {
        logger::print("\tRight :: FSR Calibration : ");
        logger::print(right_leg.do_calibration_heel_fsr);
        logger::println(right_leg.do_calibration_toe_fsr);
        logger::print("\tRight :: FSR Refinement : ");
        logger::print(right_leg.do_calibration_refinement_heel_fsr);
        logger::println(right_leg.do_calibration_refinement_toe_fsr);
        logger::print("\tRight :: Percent Gait : ");
        logger::println(right_leg.percent_gait);
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(right_leg.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(right_leg.toe_fsr);
        
        if(right_leg.hip.is_used)
        {
            logger::println("\tRight :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.hip.controller.parameter_set);
            
        }
        
        if(right_leg.knee.is_used)
        {
            logger::println("\tRight :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.knee.controller.parameter_set);
        }
        
        if(right_leg.ankle.is_used)
        {
            logger::println("\tRight :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.ankle.controller.parameter_set);
        }

        if (right_leg.elbow.is_used)
        {
            logger::println("\tRight :: Elbow");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_leg.elbow.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_leg.elbow.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_leg.elbow.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_leg.elbow.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_leg.elbow.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_leg.elbow.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_leg.elbow.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_leg.elbow.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_leg.elbow.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_leg.elbow.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_leg.elbow.controller.parameter_set);
        }
    }
   
};

