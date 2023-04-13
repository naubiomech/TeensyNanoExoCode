/*
   Code to test the Status LED
   P. Stegall April 2022
*/
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)
  // general headers for using the system
  #include "src\Board.h"
  #include "src\Utilities.h"
  #include "src\ParseIni.h"
  
  // header for the component we are checking.
  #include "src\Motor.h"
  #include "src\ExoData.h"
  #include "src\Joint.h"
  #include <math.h>
  
  namespace config_info
  {
    uint8_t (config_to_send)[ini_config::number_of_keys];
  }
  
  void setup()
  {
    Serial.begin(115200);
    while(!Serial)
    {
      ;
    }
    // enable the estop pullup.
    pinMode(logic_micro_pins::motor_stop_pin,INPUT_PULLUP);
  }
  
  void loop()
  {
    config_info::config_to_send[config_defs::board_name_idx] = (uint8_t)config_defs::board_name::AK_board;
    config_info::config_to_send[config_defs::board_version_idx] = (uint8_t)config_defs::board_version::zero_three;
    config_info::config_to_send[config_defs::battery_idx] = (uint8_t)config_defs::battery::dumb;
    config_info::config_to_send[config_defs::exo_name_idx] = (uint8_t)config_defs::exo_name::bilateral_hip_ankle;
    config_info::config_to_send[config_defs::exo_side_idx] = (uint8_t)config_defs::exo_side::bilateral;
    config_info::config_to_send[config_defs::hip_idx] = (uint8_t)config_defs::motor::AK60;
    config_info::config_to_send[config_defs::knee_idx] = (uint8_t)config_defs::motor::not_used;
    config_info::config_to_send[config_defs::ankle_idx] = (uint8_t)config_defs::motor::AK80;
    config_info::config_to_send[config_defs::hip_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::knee_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::ankle_gear_idx] = (uint8_t)config_defs::gearing::gearing_1_1;
    config_info::config_to_send[config_defs::exo_hip_default_controller_idx] = (uint8_t)config_defs::hip_controllers::disabled;
    config_info::config_to_send[config_defs::exo_knee_default_controller_idx] = (uint8_t)config_defs::knee_controllers::disabled;
    config_info::config_to_send[config_defs::exo_ankle_default_controller_idx] = (uint8_t)config_defs::ankle_controllers::disabled;
    config_info::config_to_send[config_defs::hip_flip_dir_idx] = (uint8_t)config_defs::flip_dir::right;
    config_info::config_to_send[config_defs::knee_flip_dir_idx] = (uint8_t)config_defs::flip_dir::neither;
    config_info::config_to_send[config_defs::ankle_flip_dir_idx] = (uint8_t)config_defs::flip_dir::neither;
    
    static ExoData exo_data(config_info::config_to_send);
    
    static bool first_run = true;
    static int R0_pin;
    static int R1_pin;
    static int L0_pin;
    static int L1_pin;
    if (first_run)
    {
        R0_pin = _Joint::get_motor_enable_pin(config_defs::joint_id::right_hip, &exo_data);
        R1_pin = _Joint::get_motor_enable_pin(config_defs::joint_id::right_ankle, &exo_data);
        L0_pin = _Joint::get_motor_enable_pin(config_defs::joint_id::left_hip, &exo_data);
        L1_pin = _Joint::get_motor_enable_pin(config_defs::joint_id::left_ankle, &exo_data);
    
        logger::print("R0 Enable Pin = ");
        logger::println(R0_pin);
        logger::print("R1 Enable Pin = ");
        logger::println(R1_pin);
        logger::print("L0 Enable Pin = ");
        logger::println(L0_pin);
        logger::print("L1 Enable Pin = ");
        logger::println(L1_pin);
    }

    // these should be changed to match the ID of the motors.
    static AK60 right_hip(config_defs::joint_id::right_hip, &exo_data, R0_pin);//_Joint::get_motor_enable_pin(config_defs::joint_id::right_hip, &exo_data));
    static AK60 left_hip(config_defs::joint_id::left_hip, &exo_data, L0_pin);//_Joint::get_motor_enable_pin(config_defs::joint_id::left_hip, &exo_data));
    static AK80 right_ankle(config_defs::joint_id::right_ankle, &exo_data, R1_pin);//_Joint::get_motor_enable_pin(config_defs::joint_id::right_ankle, &exo_data));
    static AK80 left_ankle(config_defs::joint_id::left_ankle, &exo_data, L1_pin);//_Joint::get_motor_enable_pin(config_defs::joint_id::left_ankle, &exo_data));
    

    static int motor_enable_time = millis();
    int time_to_stay_on_ms = 100000;
          
    ////static bool first_run = true;
    if (first_run)
    {
        first_run = false;
  
        left_hip._motor_data->enabled = true;
        right_hip._motor_data->enabled = true;
        left_ankle._motor_data->enabled = true;
        right_ankle._motor_data->enabled = true;

        left_hip.on_off(left_hip._motor_data->enabled);
        right_hip.on_off(right_hip._motor_data->enabled);
        left_ankle.on_off(left_ankle._motor_data->enabled);
        right_ankle.on_off(right_ankle._motor_data->enabled);
    }
    

      int timestamp = millis();
//      logger::print("Superloop : time since enable = ");
//      logger::print(timestamp - motor_enable_time);
//      logger::print("\n");

      if (time_to_stay_on_ms < (timestamp - motor_enable_time))
      {
          left_hip._motor_data->enabled = left_hip._motor_data->enabled ^ true;
          right_hip._motor_data->enabled = right_hip._motor_data->enabled ^ true;
          left_ankle._motor_data->enabled = left_ankle._motor_data->enabled ^ true;
          right_ankle._motor_data->enabled = right_ankle._motor_data->enabled ^ true;
          motor_enable_time = timestamp;
      }

      left_hip.on_off(left_hip._motor_data->enabled);
      right_hip.on_off(right_hip._motor_data->enabled);
      left_ankle.on_off(left_ankle._motor_data->enabled);
      right_ankle.on_off(right_ankle._motor_data->enabled);
  }


#endif
